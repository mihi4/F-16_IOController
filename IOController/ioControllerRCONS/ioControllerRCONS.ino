#include <Arduino.h>
#include <Joystick.h>
#include <Adafruit_MCP23017.h>

// define mcps

#define MCPNUM 4
#define MCPINPUTS 16
#define NOTUSED 255

#define MCPKY 3
#define MCPAVPWR 2
#define MCPHUD 1
#define MCPAIRCOND 0


// create joystick with 90 buttons and 3 axis

#define JOYBUTTONS 128  // number of total joystick buttons
#define JOYHATSWITCHES 0
#define JOYHIDID 0x04

Joystick_ Joystick = Joystick_(JOYHIDID, JOYSTICK_TYPE_JOYSTICK, JOYBUTTONS, JOYHATSWITCHES, true, true, true, true, false, false, false, false, false, false, false);

// button numbers to start from
#define AIRCONDSTART 0
#define HUDSTART 19
#define AVPWRSTART 43
#define KYSTART 67
#define ANTIICESTART 0
// FIXXXME #define ANTIICESTART 82

#define VOLUMEPIN A3

// ANTIICE pins (directly connected to ProMicro)
#define ANTIICEPINS 6

#define ENGINEOFFPIN 4
#define ENGINEONPIN 5
#define IFFUPPERPIN 6
#define IFFLOWERPIN 7
#define UHFUPPERPIN 8
#define UHFLOWERPIN 9
byte antiIcePins[ANTIICEPINS]={4,5,6,7,8,9};

// OXYGEN REGULATOR pins (directly connectd to ProMicro)
#define OXYDILUTEPIN  A10
#define OXYSERVOPIN   A0
#define OXYTESTMASKPIN 15
#define OXYEMERPIN     14
#define OXYONPIN       16

// ------------------  23017 configuration
Adafruit_MCP23017 mcp0; //first chip, used for rotaries and left funkyswitch
Adafruit_MCP23017 mcp1;
Adafruit_MCP23017 mcp2;
Adafruit_MCP23017 mcp3;
Adafruit_MCP23017 mcps[MCPNUM] = { mcp0, mcp1, mcp2, mcp3 };
byte mcpAdresses[MCPNUM] = {0, 1, 2, 3};

// status of mcp registers
uint16_t registerMcpCurrent[MCPNUM] = { 0, 0, 0, 0 };
uint16_t registerMcpPrevious[MCPNUM] = { 0, 0, 0 ,0 };



//jjjjjjjjjjjjjjjjjjjj  joystick/inputs configuration  jjjjjjjjjjjjjjjjjjjjjjjjjjjjjjj

// bit array with all button states
boolean joyButtons[JOYBUTTONS] = { 0 };

struct InputMapping
{
  byte btnNum;
  boolean isSingle;  // does the input create only one button (rotary switch) or multiple (2/3way switch)
  boolean nextPositive; // if multiple, is the "off" button number +1 or -1 than btnNum
};

// Define all inputs per mcp and their according joystick button values (on/off)
struct InputMapping aircondButtons[MCPINPUTS] = {
  // AIRCOND Rotary
  {AIRCONDSTART,   1, 0}, // OFF (0)
  {AIRCONDSTART+1, 1, 0 }, // NORM (1)
  {AIRCONDSTART+2, 1, 0}, // DUMP (2)
  {AIRCONDSTART+3, 1, 0}, // RAM  (3)

  // NUCLEAR
  {AIRCONDSTART+4, 0, 1}, // CRAD1 (4) -> CRAD2 (5)

  {AIRCONDSTART+6, 0, 1}, // REL ONLY (6) -> OFF (7)
  {AIRCONDSTART+8, 0, 0}, // ARM/REL (8) -> OFF (7)

  {NOTUSED, 0, 0}, // pin 7 not used

  // SNSPWR
  {AIRCONDSTART+9,  0, 1}, // LEFT HDPT (9)-> OFF (10)

  {AIRCONDSTART+11, 0 ,1}, // RIGHT HDPT (11)-> OFF (12)

  {AIRCONDSTART+13, 0, 1}, // FCR (13)-> OFF (14)

  {AIRCONDSTART+15, 0, 1}, // RADAR ALT (15)-> STDBY (16)
  {AIRCONDSTART+17, 0, 0}, // OFF (17)-> STDBY (16)

  {NOTUSED, 0, 0},
  {NOTUSED, 0, 0},
  {NOTUSED, 0, 0}
};

struct InputMapping hudButtons[MCPINPUTS] = {  // OFFSet is HUDSTART
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

  {HUDSTART+21, 0, 1}, // Pin 10, TEST STEP -> ON (22)
  {HUDSTART+23, 0, 0} // Pin 11, OFF -> ON (22)
};

struct InputMapping avpwrButtons[MCPINPUTS] = {
  // INS Rotary
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
};

struct InputMapping kyButtons[MCPINPUTS] = {
  // MODE rotary
  {KYSTART,   1, 0}, // P
  {KYSTART+1, 1, 0}, // C
  {KYSTART+2, 1, 0}, // LD
  {KYSTART+3, 1, 0}, // RV

  // TD Rotary
  {KYSTART+4, 0, 1}, // TD -> On
  {KYSTART+6, 0, 0}, // OFF -> On

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

  {NOTUSED, 0, 0}  // Pin 16 not used
};

struct mcpInputs {
  InputMapping inputMappings[MCPNUM];
};

//mcpInputs MCPInputs[MCPNUM]; //= {aircondButtons, avpwrButtons, hudButtons, kyButtons};
//MCPInputs[0].inputMappings = aircondButtons;*/
mcpInputs ac = {aircondButtons};
mcpInputs av = {avpwrButtons};
mcpInputs hud = {hudButtons};
mcpInputs ky = {kyButtons};

mcpInputs MCPInputs[MCPNUM] = {ac, av, hud, ky};


// ffffffffffffffffffffffffffffffff functions

void updateJoystick() {

  // set Joystickbuttons according to array
  for (int i=0; i<JOYBUTTONS; i++) {
    Joystick.setButton(i, joyButtons[i]);
  }

  Joystick.setXAxis(0);
  Joystick.setYAxis(0);
  Joystick.setZAxis(800); // analogRead(VOLUMEPIN));
  Joystick.setRxAxis(analogRead(OXYDILUTEPIN));

  Joystick.sendState();
}

void setMCPButtonValue(int mcp, int input) {

  boolean inputValue = !(mcps[mcp].digitalRead(input)); // get state of changed input, negate because of pinned to gnd

  InputMapping mapping = MCPInputs[mcp].inputMappings[input];
  byte baseBtnNum = mapping.btnNum;

  if (!mapping.isSingle) { // 2 or 3way switch connected to input
    boolean valueToSet = !inputValue; // second button is always negation of input
    byte newBtnNum = baseBtnNum-1;
    if (mapping.nextPositive) newBtnNum = baseBtnNum+1;

    joyButtons[newBtnNum] = valueToSet;
  }
  joyButtons[baseBtnNum] = inputValue;
}


void checkMCPs() {
  // iterate through io registers from all 23017
  for (int i=0; i<MCPNUM; i++) {
    registerMcpCurrent[i] = mcps[i].readGPIOAB();
    if (registerMcpCurrent[i] != registerMcpPrevious[i]) {
      // check, which bits have changed and set button values
      for (int x = 0; x < MCPINPUTS; x++) {
        if ((registerMcpCurrent[i] & (1 << x)) != (registerMcpPrevious[i] & (1 << x))) {
          // x = position of changed bit in, check if it's MSB or LSB
          // call routine to check the input and set the specific button
          // send parameters x (mcp) and i (mcp input)
          setMCPButtonValue(x, i);
        }
      }
      registerMcpPrevious[i] = registerMcpCurrent[i];
    }
  }
}


void checkAntiIce() {
  byte switches[3][2] = { {ENGINEOFFPIN, ENGINEONPIN}, {IFFUPPERPIN, IFFLOWERPIN}, {UHFUPPERPIN, UHFLOWERPIN} };

  byte switchpairnum = sizeof(switches);

  for (int i=0; i<switches; i++) {
    uint8_t pin1 = switches[i][0];
    uint8_t pin2 = switches[i][1];
    bool pos1 = !digitalRead(pin1);
    bool pos2 = !digitalRead(pin2);
    byte baseBtn = ANTIICESTART + (i*3);

    if ( pos1 || pos2 ) { // one of both positions is 1
      joyButtons[baseBtn+1] = false;
    } else {
      joyButtons[baseBtn+1] = true;
    }
    joyButtons[baseBtn] = pos1;
    joyButtons[baseBtn+2] = pos2;
    //Serial.print(i);Serial.print(": baseBtn: ");Serial.print(baseBtn);Serial.print(" pos1:");Serial.print(pos1);Serial.print(" pos2:");Serial.println(pos2);
  }
}

void checkOxyRegulator() {


}


// sssssssssssssssssssssssss SETUP sssssssssssssssssssssssssss

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);

  // inititalize 23017s
  for (int i = 0; i<MCPNUM; i++) {
    mcps[i].begin(mcpAdresses[i]);
    for (int x = 0; x<MCPINPUTS; x++) {
      mcps[i].pinMode(x, INPUT_PULLUP);
      mcps[i].pullUp(x, HIGH);
    }
  }

  // initialize antiIcePins
  for (int i=0; i<ANTIICEPINS; i++) {
    pinMode(i, INPUT_PULLUP);
  }

  // initialize OXYGEN pins
  pinMode(OXYDILUTEPIN, INPUT);
  pinMode(OXYSERVOPIN, OUTPUT);
  pinMode(OXYONPIN, INPUT_PULLUP);
  pinMode(OXYTESTMASKPIN, INPUT_PULLUP);
  pinMode(OXYEMERPIN, INPUT_PULLUP);

  Joystick.begin(false);
}

void loop() {
  // put your main code here, to run repeatedly:
  checkAntiIce();
  //checkMCPs();
  updateJoystick();
  delay(30); // only input reading does not need high performance, let chip rest
 //
}
