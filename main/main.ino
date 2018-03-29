

/*******************************************************************************
 *
 *   James Bithell
 *
 *   LightHack with Nunchuck
 *
 *   Original Code (c) 2017 by ETC
 *   Remainder (c) 2018 James Bithell
 *
 ******************************************************************************/

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <OSCBoards.h>
#include <OSCBundle.h>
#include <OSCData.h>
#include <OSCMatch.h>
#include <OSCMessage.h>
#include <OSCTiming.h>
#ifdef BOARD_HAS_USB_SERIAL
#include <SLIPEncodedUSBSerial.h>
SLIPEncodedUSBSerial SLIPSerial(thisBoardsSerialUSB);
#else
#include <SLIPEncodedSerial.h>
SLIPEncodedSerial SLIPSerial(Serial);
#endif
#include <string.h>

#include <RGBConverter.h>

#include <Wire.h>
#include "WiiLib.h"

WiiChuck chuck = WiiChuck(); //http://playground.arduino.cc/Main/WiiChuckClass
 

 /*******************************************************************************
  * Macros and Constants
  ******************************************************************************/
#define NEXT_BTN      6
#define LAST_BTN      7
#define SHIFT_BTN    8

#define REDLED        3
#define GREENLED      4
#define BLUELED      5

#define SUBSCRIBE    ((int32_t)1)
#define UNSUBSCRIBE    ((int32_t)0)

#define EDGE_DOWN    ((int32_t)1)
#define EDGE_UP      ((int32_t)0)

#define FORWARD     0
#define REVERSE     1

// Change these values to switch which direction increase/decrease pan/tilt
#define PAN_DIR     FORWARD
#define TILT_DIR      FORWARD

// Use these values to make the encoder more coarse or fine. This controls
// the number of wheel "ticks" the device sends to Eos for each tick of the
// encoder. 1 is the default and the most fine setting. Must be an integer.
#define PAN_SCALE     1
#define TILT_SCALE      1

#define SIG_DIGITS      3  // Number of significant digits displayed

#define OSC_BUF_MAX_SIZE  512

const String HANDSHAKE_QUERY = "ETCOSC?";
const String HANDSHAKE_REPLY = "OK";

//See displayScreen() below - limited to 10 chars (after 6 prefix chars)
const String VERSION_STRING = "1.0.0.5";

// Change these values to alter how long we wait before sending an OSC ping
// to see if Eos is still there, and then finally how long before we
// disconnect and show the splash screen
// Values are in milliseconds
#define PING_AFTER_IDLE_INTERVAL  2500
#define TIMEOUT_AFTER_IDLE_INTERVAL 5000

/*******************************************************************************
 * Local Types
 ******************************************************************************/
enum WHEEL_TYPE { TILT, PAN };
enum WHEEL_MODE { COARSE, FINE };

struct Encoder
{
  uint8_t pinA;
  uint8_t pinB;
  int pinAPrevious;
  int pinBPrevious;
  float pos;
  uint8_t direction;
};
struct Encoder panWheel;
struct Encoder tiltWheel;

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

bool connectedToEos = false;
unsigned long lastMessageRxTime = 0;
bool timeoutPingSent = false;
uint8_t backlightColourRed = 0;
uint8_t backlightColourGreen = 0;
uint8_t backlightColourBlue = 0;
uint8_t backlightColourBrightness = 255;
  

/*******************************************************************************
 * Local Functions
 ******************************************************************************/

/*******************************************************************************
 * Issues all our subscribes to Eos. When subscribed, Eos will keep us updated
 * with the latest values for a given parameter.
 *
 * Parameters:  none
 *
 * Return Value: void
 *
 ******************************************************************************/
void issueSubscribes()
{
  // Add a filter so we don't get spammed with unwanted OSC messages from Eos
  
  OSCMessage filter("/eos/filter/add");
  filter.add("/eos/out/param/*");
  filter.add("/eos/out/color/*");
  filter.add("/eos/out/active/*");
  filter.add("/eos/out/ping");
  SLIPSerial.beginPacket();
  filter.send(SLIPSerial);
  SLIPSerial.endPacket();

  // subscribe to Eos pan & tilt updates
  OSCMessage subPan("/eos/subscribe/param/pan");
  subPan.add(SUBSCRIBE);
  SLIPSerial.beginPacket();
  subPan.send(SLIPSerial);
  SLIPSerial.endPacket();

  OSCMessage subTilt("/eos/subscribe/param/tilt");
  subTilt.add(SUBSCRIBE);
  SLIPSerial.beginPacket();
  subTilt.send(SLIPSerial);
  SLIPSerial.endPacket();  
}

/*******************************************************************************
 * Given a valid OSCMessage (relevant to Pan/Tilt), we update our Encoder struct
 * with the new position information.
 *
 * Parameters:
 *  msg - The OSC message we will use to update our internal data
 *  addressOffset - Unused (allows for multiple nested roots)
 *
 * Return Value: void
 *
 ******************************************************************************/
void parsePanUpdate(OSCMessage& msg, int addressOffset)
{
  panWheel.pos = msg.getOSCData(0)->getFloat();
  connectedToEos = true; // Update this here just in case we missed the handshake
}

void parseTiltUpdate(OSCMessage& msg, int addressOffset)
{
  tiltWheel.pos = msg.getOSCData(0)->getFloat();
  connectedToEos = true; // Update this here just in case we missed the handshake
}

void parseColorUpdate(OSCMessage& msg, int addressOffset)
{ 
  hsv2rgb(msg.getOSCData(0)->getFloat(),msg.getOSCData(1)->getFloat());
  setBacklight();
  connectedToEos = true; // Update this here just in case we missed the handshake
}

void parseIntensityUpdate(OSCMessage& msg, int addressOffset)
{
  backlightColourBrightness = int((msg.getOSCData(0)->getFloat())*255);
  if (backlightColourBrightness < 50 && backlightColourBrightness > 0) {
    backlightColourBrightness = 50;
  }
  connectedToEos = true; // Update this here just in case we missed the handshake
  setBacklight();
}

/*******************************************************************************
 * Given an unknown OSC message we check to see if it's a handshake message.
 * If it's a handshake we issue a subscribe, otherwise we begin route the OSC
 * message to the appropriate function.
 *
 * Parameters:
 *  msg - The OSC message of unknown importance
 *
 * Return Value: void
 *
 ******************************************************************************/
void parseOSCMessage(String& msg)
{
  // check to see if this is the handshake string
  if (msg.indexOf(HANDSHAKE_QUERY) != -1)
  {
    // handshake string found!
    SLIPSerial.beginPacket();
    SLIPSerial.write((const uint8_t*)HANDSHAKE_REPLY.c_str(), (size_t)HANDSHAKE_REPLY.length());
    SLIPSerial.endPacket();

    // Let Eos know we want updates on some things
    issueSubscribes();

    // Make our splash screen go away
    connectedToEos = true;
  }
  else
  {
    // prepare the message for routing by filling an OSCMessage object with our message string
    OSCMessage oscmsg;
    oscmsg.fill((uint8_t*)msg.c_str(), (int)msg.length());
    // route pan/tilt messages to the relevant update function
    oscmsg.route("/eos/out/param/pan", parsePanUpdate);
    oscmsg.route("/eos/out/param/tilt", parseTiltUpdate);
    oscmsg.route("/eos/out/color/hs", parseColorUpdate);
    //oscmsg.route("/eos/out/active/wheel/1", parseIntensityUpdate);
  }
}

/*******************************************************************************
 * Initializes a given encoder struct to the requested parameters.
 *
 * Parameters:
 *  encoder - Pointer to the encoder we will be initializing
 *  pinA - Where the A pin is connected to the Arduino
 *  pinB - Where the B pin is connected to the Arduino
 *  direction - Determines if clockwise or counterclockwise is "forward"
 *
 * Return Value: void
 *
 ******************************************************************************/
void initEncoder(struct Encoder* encoder, uint8_t pinA, uint8_t pinB, uint8_t direction)
{
  encoder->pinA = pinA;
  encoder->pinB = pinB;
  encoder->pos = 0;
  encoder->direction = direction;

  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);

  encoder->pinAPrevious = digitalRead(pinA);
  encoder->pinBPrevious = digitalRead(pinB);
}

/*******************************************************************************
 * Checks if the encoder has moved by comparing the previous state of the pins
 * with the current state. If they are different, we know there is movement.
 * In the event of movement we update the current state of our pins.
 *
 * Parameters:
 *  encoder - Pointer to the encoder we will be checking for motion
 *
 * Return Value:
 *  encoderMotion - Returns the 0 if the encoder has not moved
 *                1 for forward motion
 *               -1 for reverse motion
 *
 ******************************************************************************/
int8_t updateEncoder(struct Encoder* encoder)
{
  int8_t encoderMotion = 0;
  int pinACurrent = digitalRead(encoder->pinA);
  int pinBCurrent = digitalRead(encoder->pinB);

  // has the encoder moved at all?
  if (encoder->pinAPrevious != pinACurrent)
  {
    // Since it has moved, we must determine if the encoder has moved forwards or backwards
    encoderMotion = (encoder->pinAPrevious == encoder->pinBPrevious) ? -1 : 1;

    // If we are in reverse mode, flip the direction of the encoder motion
    if (encoder->direction == REVERSE)
      encoderMotion = -encoderMotion;
  }
  encoder->pinAPrevious = pinACurrent;
  encoder->pinBPrevious = pinBCurrent;

  return encoderMotion;
}

/*******************************************************************************
 * Sends a message to Eos informing them of a wheel movement.
 *
 * Parameters:
 *  type - the type of wheel that's moving (i.e. pan or tilt)
 *  ticks - the direction and intensity of the movement
 *
 * Return Value: void
 *
 ******************************************************************************/
void sendWheelMove(WHEEL_TYPE type, float ticks)
{
  String wheelMsg("/eos/wheel");

  if (digitalRead(SHIFT_BTN) == LOW)
    wheelMsg.concat("/fine");
  else
    wheelMsg.concat("/coarse");

  if (type == PAN)
    wheelMsg.concat("/pan");
  else if (type == TILT)
    wheelMsg.concat("/tilt");
  else
    // something has gone very wrong
    return;

  OSCMessage wheelUpdate(wheelMsg.c_str());
  wheelUpdate.add(ticks);
  SLIPSerial.beginPacket();
  wheelUpdate.send(SLIPSerial);
  SLIPSerial.endPacket();
}

/*******************************************************************************
 * Sends a message to Eos informing them of a key press.
 *
 * Parameters:
 *  down - whether a key has been pushed down (true) or released (false)
 *  key - the key that has moved
 *
 * Return Value: void
 *
 ******************************************************************************/
void sendKeyPress(bool down, String key)
{
  key = "/eos/key/" + key;
  OSCMessage keyMsg(key.c_str());

  if (down)
    keyMsg.add(EDGE_DOWN);
  else
    keyMsg.add(EDGE_UP);

  SLIPSerial.beginPacket();
  keyMsg.send(SLIPSerial);
  SLIPSerial.endPacket();
}

/*******************************************************************************
 * Checks the status of all the buttons relevant to Eos (i.e. Next & Last)
 *
 * NOTE: This does not check the shift key. The shift key is used in tandem with
 * the encoder to determine coarse/fine mode and thus does not report to Eos
 * directly.
 *
 * Parameters: none
 *
 * Return Value: void
 *
 ******************************************************************************/
void checkButtons()
{
  static int nextKeyState = HIGH;
  static int lastKeyState = HIGH;

  // Has the button state changed
  if (digitalRead(NEXT_BTN) != nextKeyState)
  {
    // Notify Eos of this key press
    if (nextKeyState == LOW)
    {
      sendKeyPress(false, "NEXT");
      nextKeyState = HIGH;
    }
    else
    {
      sendKeyPress(true, "NEXT");
      nextKeyState = LOW;
    }
  }

  if (digitalRead(LAST_BTN) != lastKeyState)
  {
    if (lastKeyState == LOW)
    {
      sendKeyPress(false, "LAST");
      lastKeyState = HIGH;
    }
    else
    {
      sendKeyPress(true, "LAST");
      lastKeyState = LOW;
    }
  }
}

/*******************************************************************************
 * Convert Colour from Hue and Saturation to RGB

 ******************************************************************************/

float hsv2rgb(double h, double s) 
{
    uint8_t r,g,b;
    if (h == 360 && s == 0) {
      r = 255;
      g = 255;
      b = 255;
    } else {
      r = 0;
      g = 0;
      b = 0;
    }
    backlightColourRed = r;
    backlightColourGreen = g;
    backlightColourBlue = b;
}

/*******************************************************************************
 * Sets the LED Colour

 ******************************************************************************/

void setBacklight() {
  // normalize the red LED - its brighter than the rest!
  backlightColourRed = map(backlightColourRed, 0, 255, 0, 100);
  backlightColourGreen = map(backlightColourGreen, 0, 255, 0, 150);
  backlightColourRed = map(backlightColourRed, 0, 255, 0, backlightColourBrightness);
  backlightColourGreen = map(backlightColourGreen, 0, 255, 0, backlightColourBrightness);
  backlightColourBlue = map(backlightColourBlue, 0, 255, 0, backlightColourBrightness);
  // common anode so invert!
  backlightColourRed = map(backlightColourRed, 0, 255, 255, 0);
  backlightColourGreen = map(backlightColourGreen, 0, 255, 255, 0);
  backlightColourBlue = map(backlightColourBlue, 0, 255, 255, 0);
  analogWrite(REDLED, backlightColourRed);
  analogWrite(GREENLED, backlightColourGreen);
  analogWrite(BLUELED, backlightColourBlue);
  
  OSCMessage ping("/eos/ping");
  ping.add("COLOR SET");
  ping.add(backlightColourRed);
  ping.add(backlightColourBlue);
  ping.add(backlightColourGreen);
  SLIPSerial.beginPacket();
  ping.send(SLIPSerial);
  SLIPSerial.endPacket();
}
 

/*******************************************************************************
 * Here we setup our encoder, lcd, and various input devices. We also prepare
 * to communicate OSC with Eos by setting up SLIPSerial. Once we are done with
 * setup() we pass control over to loop() and never call setup() again.
 *
 * NOTE: This function is the entry function. This is where control over the
 * Arduino is passed to us (the end user).
 *
 * Parameters: none
 *
 * Return Value: void
 *
 ******************************************************************************/
void setup()
{
  SLIPSerial.begin(115200);
  // This is a hack around an Arduino bug. It was taken from the OSC library
  //examples
#ifdef BOARD_HAS_USB_SERIAL
  while (!SerialUSB);
#else
  while (!Serial);
#endif

  // This is necessary for reconnecting a device because it needs some time
  // for the serial port to open, but meanwhile the handshake message was
  // sent from Eos
  SLIPSerial.beginPacket();
  SLIPSerial.write((const uint8_t*)HANDSHAKE_REPLY.c_str(), (size_t)HANDSHAKE_REPLY.length());
  SLIPSerial.endPacket();
  // Let Eos know we want updates on some things
  issueSubscribes();

  initEncoder(&panWheel, A0, A1, PAN_DIR);
  initEncoder(&tiltWheel, A3, A4, TILT_DIR);

  pinMode(NEXT_BTN, INPUT_PULLUP);
  pinMode(LAST_BTN, INPUT_PULLUP);
  pinMode(SHIFT_BTN, INPUT_PULLUP);

  pinMode(REDLED, OUTPUT);
  pinMode(GREENLED, OUTPUT);
  pinMode(BLUELED, OUTPUT);
  setBacklight();

}

/*******************************************************************************
 * Here we service, monitor, and otherwise control all our peripheral devices.
 * First, we retrieve the status of our encoders and buttons and update Eos.
 * Next, we check if there are any OSC messages for us.
 * Finally, we update our display (if an update is necessary)
 *
 * NOTE: This function is our main loop and thus this function will be called
 * repeatedly forever
 *
 * Parameters: none
 *
 * Return Value: void
 *
 ******************************************************************************/
void loop()
{
  static String curMsg;
  int size;
  // get the updated state of each encoder
  int32_t panMotion = updateEncoder(&panWheel);
  int32_t tiltMotion = updateEncoder(&tiltWheel);

  // Scale the result by a scaling factor
  panMotion *= PAN_SCALE;
  tiltMotion *= TILT_SCALE;

  // check for next/last updates
  checkButtons();

  // now update our wheels
  if (tiltMotion != 0)
    sendWheelMove(TILT, tiltMotion);

  if (panMotion != 0)
    sendWheelMove(PAN, panMotion);

  // Then we check to see if any OSC commands have come from Eos
  // and update the display accordingly.
  size = SLIPSerial.available();
  if (size > 0)
  {
    // Fill the msg with all of the available bytes
    while (size--)
      curMsg += (char)(SLIPSerial.read());
  }
  if (SLIPSerial.endofPacket())
  {
    parseOSCMessage(curMsg);
    lastMessageRxTime = millis();
    // We only care about the ping if we haven't heard recently
    // Clear flag when we get any traffic
    timeoutPingSent = false;
    curMsg = String();
  }

  if(lastMessageRxTime > 0) 
  {
    unsigned long diff = millis() - lastMessageRxTime;
    //We first check if it's been too long and we need to time out
    if(diff > TIMEOUT_AFTER_IDLE_INTERVAL) 
    {
      connectedToEos = false;
      lastMessageRxTime = 0;
      timeoutPingSent = false;
    }

    //It could be the console is sitting idle. Send a ping once to
    // double check that it's still there, but only once after 2.5s have passed
    if(!timeoutPingSent && diff > PING_AFTER_IDLE_INTERVAL) 
    {
        OSCMessage ping("/eos/ping");
        ping.add("JBITHELLhello"); // This way we know who is sending the ping
        SLIPSerial.beginPacket();
        ping.send(SLIPSerial);
        SLIPSerial.endPacket();
        timeoutPingSent = true;
    }
  }

}

