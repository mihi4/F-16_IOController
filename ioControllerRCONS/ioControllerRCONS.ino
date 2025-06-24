#include <Arduino.h>
#include <Joystick.h>
#include <Adafruit_MCP23017.h>
#include <Servo.h>

// create joystick with 90 buttons and 3 axis

#define JOYBUTTONS 128  // number of total joystick buttons
#define JOYHATSWITCHES 0
#define JOYHIDID 0x04

Joystick_ Joystick = Joystick_(JOYHIDID, JOYSTICK_TYPE_JOYSTICK, JOYBUTTONS, JOYHATSWITCHES, true, true, true, true, false, false, false, false, false, false, false);

// joystick button numbers to start from

#define AIRCONDSTART 0
#define AIRCONDBTNS 23
#define HUDSTART (AIRCONDSTART+AIRCONDBTNS)
#define HUDBTNS 24
#define AVPWRSTART (HUDSTART+HUDBTNS)
#define AVPWRBTNS 24
#define ANTIICESTART (AVPWRSTART+AVPWRBTNS)
#define ANTIICEBTNS 9
#define KYSTART (ANTIICESTART+ANTIICEBTNS)
#define KYBTNS 15
#define OXYBTNSTART (KYSTART+KYBTNS)
#define OXYBTNS 5

#define VOLUMEPIN A3

#define ENGINEOFFPIN 8
#define ENGINEONPIN 9
#define IFFUPPERPIN 7
#define IFFLOWERPIN 6
#define UHFUPPERPIN 5
#define UHFLOWERPIN 4

// ANTIICE pins (directly connected to ProMicro)
#define ANTIICEPINNUM 6

byte antiIcePins[ANTIICEPINNUM]={4,5,6,7,8,9};

// OXYGEN REGULATOR pins (directly connectd to ProMicro)
#define OXYDILUTEPIN  A10
#define OXYSERVOPIN   A0
#define OXYTESTMASKPIN 14
#define OXYEMERPIN     16
#define OXYONPIN       15

// servo positions
#define OXYZERO 33
#define OXYSTART 61
#define OXYEND 139

// ------------------  23017 configuration

// define mcps

#define MCPNUM 4
#define MCPINPUTS 16
#define NOTUSED 255

#define MCPKY 3
#define MCPAVPWR 2
#define MCPHUD 1
#define MCPAIRCOND 0

#define LEDPIN 17
#define BUTTONHOLDTIME 80 //FIXXXME set to 100 after testing

#define CONFIGHOLDTIME 2000

/*
  addr 0 = A2 low , A1 low , A0 low  000
  addr 1 = A2 low , A1 low , A0 high 001
*/

Adafruit_MCP23017 mcp0;
Adafruit_MCP23017 mcp1;
Adafruit_MCP23017 mcp2;
Adafruit_MCP23017 mcp3;
Adafruit_MCP23017 mcps[MCPNUM] = { mcp0, mcp1, mcp2, mcp3 };
byte mcpAdresses[MCPNUM] = {0, 1, 2, 4};

// status of mcp registers
uint16_t registerMcpCurrent[MCPNUM] = { 0, 0, 0, 0 };
uint16_t registerMcpPrevious[MCPNUM] = { 0, 0, 0, 0 };

//jjjjjjjjjjjjjjjjjjjj  joystick/inputs configuration  jjjjjjjjjjjjjjjjjjjjjjjjjjjjjjj

// bit array with all button states
boolean joyButtons[JOYBUTTONS] = { 0 };

/* InputMapping stores information about each input pin type (rotary position, or toggle switch)
   btnNum: joystick button number to use
   isSingle: true(1)=only one joystick button is used, false(0)=multiple buttons are used
   nextPositive: sets the behaviour of the action when this position is released; ignored when "isSingle" is true.
                 true= the joystick button used when out of this positin is added to btnNum (+1)
                 false=the joystick button used when out of this possition is subtracted from btnNum (-1)
*/
typedef struct InputMapping
{
  byte btnNum;
  boolean isSingle;  // does the input create only one button (rotary switch) or multiple (2/3way switch)
  boolean nextPositive; // if multiple, is the "off" button number +1 or -1 than btnNum
} InputMapping;

// Define all inputs per mcp and their according joystick button values (on/off)
const InputMapping allMappings[MCPNUM][MCPINPUTS] = {
  {    ////////////////////  AIRCONDITION  //////////////////////////
    // NUCLEAR
    {AIRCONDSTART, 0, 1}, // Pin 0 CRAD1 (4) -> CRAD2 (5)

    {AIRCONDSTART+2, 0, 1}, // Pin 1 REL ONLY (6) -> OFF (7)
    {AIRCONDSTART+4, 0, 0}, // Pin 2 ARM/REL (8) -> OFF (7)

    {AIRCONDSTART+5, 1, 0}, // Pin 3 OFF (0)
    {AIRCONDSTART+6, 1, 0 }, // Pin 4 NORM (1)
    {AIRCONDSTART+7, 1, 0}, // Pin 5 DUMP (2)
    {AIRCONDSTART+8, 1, 0}, // Pin 6 RAM  (3)

    {NOTUSED, 0, 0}, // pin 7 not used

    // SNSPWR
    {AIRCONDSTART+9,  0, 1}, // Pin 8 LEFT HDPT (9)-> OFF (10)

    {AIRCONDSTART+11, 0 ,1}, // Pin 9 RIGHT HDPT (11)-> OFF (12)

    {AIRCONDSTART+13, 0, 1}, // Pin 10 FCR (13)-> OFF (14)

    {AIRCONDSTART+15, 0, 1}, // Pin 11 RADAR ALT (15)-> STDBY (16)
    {AIRCONDSTART+17, 0, 0}, // Pin 12 OFF (17)-> STDBY (16)

    // ZEROIZE Panel
    {AIRCONDSTART+18, 0, 1}, // Pin 13 OFP (18)-> OFF (19)
    {AIRCONDSTART+20, 0, 0}, // Pin 14 DATA (20)-> OFF (19)

    {AIRCONDSTART+21, 0, 1} // Pin 15 INHIBIT(21)-> VOICEMESSAGE (22)

  },
  {    ////////////////////   HUD   ////////////////////////

    {HUDSTART,   0, 1}, // Pin 0, VV/VAH -> VAH (1)
    {HUDSTART+2, 0, 0}, // Pin 1, OFF -> VAH (1)

    {HUDSTART+3, 0, 1}, // Pin 2, ATT/FPM -> FPM (4)
    {HUDSTART+5, 0, 0}, // Pin 3, OFF -> FPM (4)

    {HUDSTART+6, 0, 1}, // Pin 4, DED DATA -> PFD (7)
    {HUDSTART+8, 0, 0}, // Pin 5, OFF -> PFD (7)

    // DEPR RET
    {HUDSTART+9,  0, 1}, // Pin 6, STBY -> PRI (10)
    {HUDSTART+11, 0, 0}, // Pin 7, OFF -> PRI (10)

    {HUDSTART+12, 0, 1}, // Pin 8, CAS -> TAS (13)
    {HUDSTART+14, 0, 0}, // Pin 9, OFF -> TAS (13)

    {HUDSTART+15, 0, 1}, // Pin 10, ALT RADAR -> BARO (16)
    {HUDSTART+17, 0, 0}, // Pin 11, OFF -> BARO (16)

    {HUDSTART+18, 0, 1}, // Pin 12, DAY (18) -> AUTO BRT (19)
    {HUDSTART+20, 0, 0}, // Pin 13, NIGHT (20) -> AUTO BRT (19)

    {HUDSTART+21, 0, 1}, // Pin 14, TEST STEP -> ON (22)
    {HUDSTART+23, 0, 0} // Pin 15, OFF -> ON (22)
  },
  {  /////////////////////  AVPWR  ///////////////////////
    {AVPWRSTART,   1, 0}, // OFF
    {AVPWRSTART+1, 1, 0 }, // STOR HDG
    {AVPWRSTART+2, 1, 0}, // NORM
    {AVPWRSTART+3, 1, 0}, // NAV
    {AVPWRSTART+4, 1, 0}, // CAL
    {AVPWRSTART+5, 1, 0}, // IN FLT ALIGN
    {AVPWRSTART+6, 1, 0}, // ATT
    // MIDS LVT
    {AVPWRSTART+7, 0, 1}, // ZERO -> OFF (8)
    {AVPWRSTART+9, 0, 0}, // ON-> OFF (8)

    {AVPWRSTART+10, 0, 1}, // MMC -> OFF (11)
    {AVPWRSTART+12, 0, 1}, // ST STA -> OFF (13)
    {AVPWRSTART+14, 0, 1}, // MFD -> OFF (15)
    {AVPWRSTART+16, 0, 1}, // UFC -> OFF (17)
    {AVPWRSTART+18, 0, 1}, // MAP -> OFF (19)
    {AVPWRSTART+20, 0, 1}, // DL -> OFF (21)
    {AVPWRSTART+22, 0, 1} // GPS -> OFF (23)
  },
  { /////////////////////  KY58  /////////////////////////
    // MODE rotary
    {KYSTART,   1, 0}, // P
    {KYSTART+1, 1, 0}, // C
    {KYSTART+2, 1, 0}, // LD
    {KYSTART+3, 1, 0}, // RV

    // TD Rotary
    {KYSTART+4, 1, 0}, // OFF
    {KYSTART+5, 1, 0}, // ON
    {KYSTART+6, 1, 0}, // TD

    {NOTUSED, 0, 0}, // Pin 7 not used

    //FILL Rotary
    {KYSTART+7,  1, 0}, // Z 1-5
    {KYSTART+8,  1, 0}, // 1
    {KYSTART+9,  1, 0}, // 2
    {KYSTART+10, 1, 0}, // 3
    {KYSTART+11, 1, 0}, // 4
    {KYSTART+12, 1, 0}, // 5
    {KYSTART+13, 1, 0}, // 6
    {KYSTART+14, 1, 0}, // Z ALL
  }
 };

//oooooooooooooooooooooooooo  OXYSERVO configuration

Servo oxyIndicator;
unsigned long lastZeroTime = 0;
unsigned long lastConfigCheckTime = 0;
unsigned long ledBlinkPrevious = 0;
bool ledState = LOW;
bool oxyZeroized = false;
byte oxyCheckInterval = 500;

// mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm switch MODE configuration
bool configMode = false;
bool oldTestPos = false;
bool testPosState;
bool pulseMode = false; // in pulseMode, the specific joystickbuttons will be held for

bool prevAntiIceValues[ANTIICEPINNUM] = {0,0,0,0,0,0};  // array to get, which buttons on the antiIce have changed

// ffffffffffffffffffffffffffffffff functions

/*
  Reads the specified input button "input" from mcp "mcp", calculates the expected joystick button and it's value and sets it in the "joyButtons" array.
  This function DOES NOT update the joystick yet, just sets the internal value in the array!
  If pulseMode is selected, it calculates the button to pulse and calls the pulseJoystickButton routine

  NOT USED ANYMORE, instead pulseMCPButtonValue for pulsing, updateInputs for continuous mode is used
*/
void setMCPButtonValue(int mcp, int input) {

  boolean inputValue = !(mcps[mcp].digitalRead(input)); // get state of changed input, negate because of pinned to gnd
  InputMapping mapping = allMappings[mcp][input];
  byte baseBtnNum = mapping.btnNum;
  if (baseBtnNum != NOTUSED) {

    if (!pulseMode) {
      if (!mapping.isSingle) { // 2 or 3way switch connected to input
        boolean valueToSet = !inputValue; // second button is always negation of input
        byte newBtnNum = baseBtnNum-1;
        if (mapping.nextPositive) newBtnNum = baseBtnNum+1;
        // Serial.print("Button ");Serial.print(newBtnNum);Serial.print("-val:");Serial.println(valueToSet);
        joyButtons[newBtnNum] = valueToSet;
      }
      joyButtons[baseBtnNum] = inputValue;
    } else {
      byte button = baseBtnNum;
      if ((!mapping.isSingle) && (!inputValue)) { // 2 or 3way switch connected to input and main input pin OFF
        button = baseBtnNum-1;
        if (mapping.nextPositive) button = baseBtnNum+1;
      }
      pulseJoystickButton(button);
    }
  }
}

void pulseJoystickButton(byte btnNum) {
  Joystick.setButton(btnNum, 1);
  Joystick.sendState();
  delay(BUTTONHOLDTIME);
  Joystick.setButton(btnNum, 0);
  Joystick.sendState();
}


/*
 calculate, which button should be updated based on the changed input, and pulse it
*/
void pulseMCPButtonValue(int mcp, int input) {

  boolean inputValue = !(mcps[mcp].digitalRead(input)); // get state of changed input, negate because of pinned to gnd
  InputMapping mapping = allMappings[mcp][input];
  byte baseBtnNum = mapping.btnNum;
  if (baseBtnNum != NOTUSED) {
    byte button = baseBtnNum;
    if (!mapping.isSingle) { // 2 or 3way switch connected to input
      button = baseBtnNum-1;  //
      if (mapping.nextPositive) button = baseBtnNum+1;
      // Serial.print("Button ");Serial.print(newBtnNum);Serial.print("-val:");Serial.println(valueToSet);
      if (inputValue) button = baseBtnNum;
      pulseJoystickButton(button);
    } else {  // rotary switch input changed
      if (inputValue) {  // is new value ON?
        pulseJoystickButton(button);
      }
    }
  }
}


/*
  Sets all buttons to the current value of the corresponding switch inputs
*/
void updateInputs()  {

  boolean first3pos = true;   // this sets true, if the first of a 3pos switch is ON

  for (byte mcp=0; mcp<MCPNUM; mcp++) { // iterate through all 23017s
    for (byte input=0; input<MCPINPUTS; input++) {
      boolean inputValue = !(mcps[mcp].digitalRead(input)); // get state of input, negate because of pinned to gnd
      InputMapping mapping = allMappings[mcp][input];
      byte baseBtnNum = mapping.btnNum;
      if (baseBtnNum != NOTUSED) {
        joyButtons[baseBtnNum] = inputValue;  // set base button of input to pin value, now calculate if 2nd or middle pos must be set
        if (!mapping.isSingle) { // 2 or 3way switch connected to input
          if (mapping.nextPositive) { // is it the first position config? set middle/secondary pos to true temporarily
            first3pos = inputValue;
            joyButtons[baseBtnNum+1] = !inputValue;
          } else {  // 3rd postition in a 3way switch is also OFF
            if (!first3pos) { // first position is not ON
              joyButtons[baseBtnNum-1] = !inputValue;
              first3pos = false;
            }
          }
        }
      }
    }
  }
}
/*
  Checks, if any input on the mcps has changed and calls the pulseMCPButtonValue, if needed
*/
boolean checkMCPs() {
  // iterate through io registers from all 23017
  for (int i=0; i<MCPNUM; i++) {
    registerMcpCurrent[i] = mcps[i].readGPIOAB();
    //Serial.print(i);Serial.print(": regCurrent is "); // Serial.println(registerMcpCurrent[i]);
    if (registerMcpCurrent[i] != registerMcpPrevious[i]) {
      if (!pulseMode) {
        registerMcpPrevious[i] = registerMcpCurrent[i];
        return true; // let updateInputs routine do it's job in continuous MODE
      }
      // check, which bits have changed and set button values
      for (int x = 0; x < MCPINPUTS; x++) {
        if ((registerMcpCurrent[i] & (1 << x)) != (registerMcpPrevious[i] & (1 << x))) { // x = position of changed bit, check if it's MSB or LSB
          // call routine to check the input and set the specific button
          // send parameters i (mcp) and x (mcp input)
          pulseMCPButtonValue(i, x);
        }
      }
      registerMcpPrevious[i] = registerMcpCurrent[i];
    }
  }
  return false;
}

void updateJoystick() {

  // set Joystickbuttons according to array
  for (int i=0; i<JOYBUTTONS; i++) {
    Joystick.setButton(i, joyButtons[i]);
  }

  Joystick.setXAxis(0);
  Joystick.setYAxis(0);
  Joystick.setZAxis(analogRead(VOLUMEPIN));
  Joystick.setRxAxis(analogRead(OXYDILUTEPIN));
  Joystick.sendState();

}


void checkAntiIce() {
  byte switches[3][2] = { {ENGINEOFFPIN, ENGINEONPIN}, {IFFUPPERPIN, IFFLOWERPIN}, {UHFUPPERPIN, UHFLOWERPIN} };
  byte switchpairnum = 3;

  if (!pulseMode) {
    for (int i=0; i<switchpairnum; i++) {
      uint8_t pin1 = switches[i][0];
      uint8_t pin2 = switches[i][1];
      bool pos1 = digitalRead(pin1);
      bool pos2 = digitalRead(pin2);
      byte baseBtn = ANTIICESTART + (i*3);

      joyButtons[baseBtn] = !pos1;
      joyButtons[baseBtn+2] = !pos2;
      joyButtons[baseBtn+1] = pos1 && pos2;
    }
  } else {  // pulse Mode for switches
    bool currentAntiIceValues[ANTIICEPINNUM] = {0};
    for (byte i=0; i<switchpairnum; i++) {
      currentAntiIceValues[i*2] = digitalRead(switches[i][0]);
      byte baseBtn = ANTIICESTART + (i*3);
      if (currentAntiIceValues[i*2] != prevAntiIceValues[i*2]) {
        if (!currentAntiIceValues[i*2]) { // this position is on
          pulseJoystickButton(baseBtn);
        } else {
          pulseJoystickButton(baseBtn+1);
        }
      }
      currentAntiIceValues[i*2+1] = digitalRead(switches[i][1]);
      if (currentAntiIceValues[i*2+1] != prevAntiIceValues[i*2+1]) {
        if (!currentAntiIceValues[i*2+1]) { // this position is on
          pulseJoystickButton(baseBtn+2);
        } else {
          pulseJoystickButton(baseBtn+1);
        }
      }
    }
    for (byte i=0; i<ANTIICEPINNUM; i++) { prevAntiIceValues[i] = currentAntiIceValues[i]; }
  }
}

void updateOxyRegulator() {
  /* move the needle according to the dilute lever */
  unsigned long timeNow = millis();
  if (digitalRead(OXYONPIN)) {  // power on
    oxyZeroized = false;
    if (!oxyIndicator.attached()) oxyIndicator.attach(OXYSERVOPIN);
    int oxyIndicatorPos = map(analogRead(OXYDILUTEPIN), 200, 850, OXYEND, OXYSTART);
    oxyIndicator.write(oxyIndicatorPos);
  } else {  // no power on the OXY Panel
    timeNow = millis();
    if (oxyZeroized) {
      if ((timeNow - lastZeroTime) > oxyCheckInterval)  oxyIndicator.detach();
    } else {
        if (oxyIndicator.attached()) {
          // Serial.println("writing ZERO");
          oxyIndicator.write(OXYZERO);
          oxyZeroized = true;
          lastZeroTime = timeNow;
        }
    }
  }

  // button functions
  byte baseBtnNum = OXYBTNSTART;

  // ON/OFF switch
  joyButtons[baseBtnNum] = !digitalRead(OXYONPIN);
  joyButtons[baseBtnNum+1] = digitalRead(OXYONPIN);

  // MASK switch
  bool posEMER = digitalRead(OXYEMERPIN);
  bool posTESTMASK = digitalRead(OXYTESTMASKPIN);
  joyButtons[baseBtnNum+2] = !posEMER;
  joyButtons[baseBtnNum+4] = !posTESTMASK;
  joyButtons[baseBtnNum+3] = posEMER && posTESTMASK;

}

void checkConfigSwitch() {
  unsigned long time = millis();
  bool testPos = digitalRead(OXYTESTMASKPIN);
  /*
  this part of the routine uses the already given millis() to let the internal LED blink for debug reasons
  long if pulseMode is ON, fast if OFF
  */
  unsigned long interval = 300 + 1000*pulseMode;
  if(time - ledBlinkPrevious > interval) {
  	// save the last time you blinked the LED
  	ledBlinkPrevious = time;
  	if (ledState == LOW)
  	  ledState = HIGH;
  	else
  	  ledState = LOW;
  	// set the LED with the ledState of the variable:
    digitalWrite(LEDPIN, ledState);
  }

  if (testPos != oldTestPos) {
    lastConfigCheckTime = millis();
  }
  if((millis() - lastConfigCheckTime) > CONFIGHOLDTIME)
  {
    if (testPos != testPosState) {
      testPosState = testPos;
      if (!testPosState) configMode = !configMode;
    }

  }
  //lastConfigCheckTime = time;
  oldTestPos = testPos;
}

// sssssssssssssssssssssssss SETUP sssssssssssssssssssssssssss

void setup() {
  // put your setup code here, to run once:
  /* Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
   Serial.println("F-16 Right Console IOController starting!"); */

  // inititalize 23017s
  for (int i = 0; i<MCPNUM; i++) {
    mcps[i].begin(mcpAdresses[i]);
    for (int x = 0; x<MCPINPUTS; x++) {
      mcps[i].pinMode(x, INPUT);
      mcps[i].pullUp(x, HIGH);
    }
  }

  // initialize antiIcePins
  for (int i=0; i<ANTIICEPINNUM; i++) {
    pinMode(antiIcePins[i], INPUT_PULLUP);
  }

  // initialize OXYGEN pins
  pinMode(OXYDILUTEPIN, INPUT);
  pinMode(OXYSERVOPIN, OUTPUT);
  pinMode(OXYONPIN, INPUT_PULLUP);
  pinMode(OXYTESTMASKPIN, INPUT_PULLUP);
  pinMode(OXYEMERPIN, INPUT_PULLUP);
  pinMode(LEDPIN, OUTPUT);

  // Serial.print("Setting up Servo");
  oxyIndicator.attach(OXYSERVOPIN);
  oxyIndicator.write(OXYZERO);
  delay(400);
  oxyIndicator.detach();
  // Serial.println("- Servo done");

  Joystick.begin(false);
  updateInputs();
  updateJoystick();

}

// oooooooooooooooooooooooooo LOOP oooooooooooooooooooooooooo

void loop() {
  // put your main code here, to run repeatedly:
  checkConfigSwitch();

  if (!configMode) {  // regular behaviour

    checkAntiIce();
    updateOxyRegulator();
    if (checkMCPs()) updateInputs(); //checkMCPs();
    updateJoystick();

  } else {  // do something else in config mode :-)

    oxyIndicator.attach(OXYSERVOPIN);
    oxyIndicator.write(OXYEND);
    delay(300);
    oxyIndicator.detach();
    if (!(mcps[MCPKY].digitalRead(0))) {
      pulseMode = true; // input 0 on mcpKY is P = pulse mode
      memset(joyButtons, 0, sizeof(joyButtons)); // empty Joystick array
      updateJoystick();
    }
    if (!(mcps[MCPKY].digitalRead(1))) {
      pulseMode = false; // input 1 on mcpKY is C = continous mode
      updateInputs();
      updateJoystick();
    }
  }
  delay(10); // only input reading does not need high performance, let chip rest
}
