/*******************************************************************************
 *
 *   Nunchuck as moving light controller LightHack
 *   
 *   https://github.com/ETCLabs/lighthack
 *
 *   (c) 2018 ETC &James Bithell
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

#include <Wire.h>
#include "WiiLib.h"

WiiChuck chuck = WiiChuck(); //http://playground.arduino.cc/Main/WiiChuckClass
 

 /*******************************************************************************
  * Macros and Constants
  ******************************************************************************/
#define REDLED        3
#define GREENLED      4
#define BLUELED      5
//LCD PIN 15 TO 5V 

#define SUBSCRIBE    ((int32_t)1)
#define UNSUBSCRIBE    ((int32_t)0)

#define EDGE_DOWN    ((int32_t)1)
#define EDGE_UP      ((int32_t)0)

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

 * Global Variables
 ******************************************************************************/
unsigned long zLastPressed, cLastPressed;
unsigned long lastMessageRxTime = 0;
bool timeoutPingSent = false;
uint8_t backlightColourRed = 0;
uint8_t backlightColourGreen = 0;
uint8_t backlightColourBlue = 0;
float backlightColourBrightness = 1;
  

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
  filter.add("/eos/out/color/*");
  filter.add("/eos/out/ping");
  SLIPSerial.beginPacket();
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

void parseColorUpdate(OSCMessage& msg, int addressOffset)
{ 
       
  if (msg.getOSCData(0)->getFloat() == 0.000 && msg.getOSCData(1)->getFloat() == 0.000) { //Test for things that basically don't have a colour - such as a dimmer
    backlightColourBrightness = 0;
  } else {
    hsv2rgb(msg.getOSCData(0)->getFloat(),msg.getOSCData(1)->getFloat());
    if (backlightColourBrightness != 1) {
      backlightColourBrightness = 1;
    }
  }
 
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
    oscmsg.route("/eos/out/color/hs", parseColorUpdate);
  }
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
void sendWheelMove(int x, int y)
{
  String wheelMsgA = "/eos/wheel";
  String wheelMsgB = "/eos/wheel";

  if (chuck.buttonZ && chuck.buttonC) {
    wheelMsgA.concat("/fine");
    wheelMsgB.concat("/fine");
    x = x*3; //Calibration Factor for fine
    y = y*1.5; //Calibration Factor  - Y has smaller range on most moving lights
  } else {
    wheelMsgA.concat("/coarse");
    wheelMsgB.concat("/coarse");
    x = x*0.3; //Calibration Factor for coarse
    y = y*0.15; //Calibration Factor - Y has smaller range on most moving lights
  }
  wheelMsgA.concat("/pan");
  wheelMsgB.concat("/tilt");
  
  OSCMessage wheelUpdateA(wheelMsgA.c_str());
  wheelUpdateA.add(x);
  SLIPSerial.beginPacket();
  wheelUpdateA.send(SLIPSerial);
  SLIPSerial.endPacket();

  OSCMessage wheelUpdateB(wheelMsgB.c_str());
  wheelUpdateB.add(y);
  SLIPSerial.beginPacket();
  wheelUpdateB.send(SLIPSerial);
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
void sendKeyPress(String key)
{
  key = "/eos/key/" + key;
  OSCMessage keyMsg(key.c_str());
  keyMsg.add(EDGE_DOWN);
  SLIPSerial.beginPacket();
  keyMsg.send(SLIPSerial);
  SLIPSerial.endPacket();
  
  OSCMessage keyMsgUp(key.c_str());
  keyMsgUp.add(EDGE_UP);
  SLIPSerial.beginPacket();
  keyMsgUp.send(SLIPSerial);
  SLIPSerial.endPacket();
  
}

/*******************************************************************************
 * Checks the status of all the buttons relevant to Eos (i.e. Next & Last)
 *
 * Parameters: none
 *
 * Return Value: void
 *
 ******************************************************************************/
void checkButtons()
{
  if (chuck.buttonZ && !chuck.buttonC && zLastPressed < (millis()-500)) {
    delay(1000);
    if (!chuck.buttonC) { //If after 300 seconds the C button isn't depressed it means this isn't a false trigger as they try and push both buttons together - so go ahead and send a keypress
      sendKeyPress("NEXT");
      zLastPressed = millis();
    }
  }
  if (chuck.buttonC && !chuck.buttonZ && cLastPressed < (millis()-500)) {
    delay(1000);
    if (!chuck.buttonZ) { //If after 500 seconds the Z button isn't depressed it means this isn't a false trigger as they try and push both buttons together - so go ahead and send a keypress
      sendKeyPress("LAST");
      cLastPressed = millis();
    }
  }
  if (chuck.buttonC && chuck.buttonZ) {
    zLastPressed = millis()+200;
    cLastPressed = millis()+200; //Make an even bigger delay after you release both keys to stop it jumping straight to a button 
  }
}

/*******************************************************************************
 * Convert Colour from Hue and Saturation to RGB
 * Stolen from https://www.kasperkamperman.com/blog/arduino/arduino-programming-hsb-to-rgb/

 ******************************************************************************/
void hsv2rgb(int hue, int sat) { 
  int val = 255;
  sat = sat*2.55; //Get it to 255 as max 
  
  int r;
  int g;
  int b;
  int base;
 
  if (sat == 0) { // Acromatic color (gray). Hue doesn't mind.
    r=val;
    g=val;
    b=val;  
  } else  { 
 
    base = ((255 - sat) * val)>>8;
 
    switch(hue/60) {
      case 0:
          r = val;
          g = (((val-base)*hue)/60)+base;
          b = base;
      break;
   
      case 1:
          r = (((val-base)*(60-(hue%60)))/60)+base;
          g = val;
          b = base;
      break;
   
      case 2:
          r = base;
          g = val;
          b = (((val-base)*(hue%60))/60)+base;
      break;
   
      case 3:
          r = base;
          g = (((val-base)*(60-(hue%60)))/60)+base;
          b = val;
      break;
   
      case 4:
          r = (((val-base)*(hue%60))/60)+base;
          g = base;
          b = val;
      break;
   
      case 5:
          r = val;
          g = base;
          b = (((val-base)*(60-(hue%60)))/60)+base;
      break;
      }
    }
    backlightColourRed = r;
    backlightColourGreen = g;
    backlightColourBlue = b;
}

/*******************************************************************************
 * Sets the LED Colour

 ******************************************************************************/

void setBacklight() {  
  backlightColourRed = backlightColourRed*backlightColourBrightness;
  backlightColourGreen = backlightColourGreen*backlightColourBrightness;
  backlightColourBlue = backlightColourBlue*backlightColourBrightness;
  
  backlightColourRed = map(backlightColourRed, 0, 255, 255, 0);
  backlightColourGreen = map(backlightColourGreen, 0, 255, 255, 0);
  backlightColourBlue = map(backlightColourBlue, 0, 255, 255, 0);
  
  analogWrite(REDLED, backlightColourRed);
  analogWrite(GREENLED, backlightColourGreen);
  analogWrite(BLUELED, backlightColourBlue);
  

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

  chuck.begin();
  chuck.update();
  chuck.calibrateJoy();


  zLastPressed = millis();
  cLastPressed = millis();
  
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
 ******************************************************************************/

void loop()
{
  static String curMsg;
  int size;

  delay(20); //Try not to overcheck on the Chuck
  chuck.update(); //Request position of buttons & joystick
  checkButtons();

  // check satus of chuck joystick
  if (chuck.readJoyX() != 0 || chuck.readJoyY() != 0) {
    sendWheelMove(chuck.readJoyX(), chuck.readJoyY());
  }

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
      lastMessageRxTime = 0;
      timeoutPingSent = false;
    }

    //It could be the console is sitting idle. Send a ping once to
    // double check that it's still there, but only once after 2.5s have passed
    if(!timeoutPingSent && diff > PING_AFTER_IDLE_INTERVAL) 
    {
        OSCMessage ping("/eos/ping");
        ping.add("arduinoHello"); // This way we know who is sending the ping
        SLIPSerial.beginPacket();
        ping.send(SLIPSerial);
        SLIPSerial.endPacket();
        timeoutPingSent = true;
    }
  }

}
