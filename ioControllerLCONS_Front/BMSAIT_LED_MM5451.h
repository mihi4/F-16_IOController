// settings and functions to drive LEDs through an MM5451 LED driver chip
// V0.1 13.09.2022

// datenfeld.target defines the MM5451 device (in an array)
// datenfeld.ref2 sets the pin of a single LED
// datenfeld.ref2 sets a timemark for fast blink (2 switches per second)
// datenfeld.ref3 sets a timemark for slow blink (1 switch per second)


///////////////////////////////////////////////////
//
// LEFT CONSOLE VERSION FOR ECM AND AVTR LIGHTS
//
//////////////////////////////////////////////////

#include "MM5451.h"

#define BLINKSPEED 500  //pause (in ms) between on/off for fast blinking. Slow blinking will be 50%

#define LASTLEDPIN 33
#define LAMPS_STARTINDEX 9  // ignore ELEC and PROBEHEAT
#define LAMPCOUNT 24	// lamps to iterate through, exluding the 2 above and the "----" 
#define BUCINDEX 27   // index of BUC lamp, used to control the "-----" lamps

#define ALLLIGHTS1POS   7
#define ALLLIGHTS2POS   8
#define ALLLIGHTS3POS   9

unsigned long LEDTimer;

bool allLightsOn = false; // will be set, if all 3 AllLightsOn bits are set to light up the whole panel

MM5451 mm5451[]={
    MM5451(16,17)
    //MM5451(6,7)
}; // create instances of chip classes, parameters are CLK and DATA pin 

const byte mm5451anz = sizeof(mm5451)/sizeof(mm5451[0]); 

void UpdateBlink()
{
  LEDTimer=millis();
  for(byte a = LAMPS_STARTINDEX; a < LAMPS_STARTINDEX+LAMPCOUNT; a++)  // for(byte a = 0; a < VARIABLENANZAHL; a++)
  {
    if (datenfeld[a].typ==13)
    {
      if (datenfeld[a].ref3==1)
      {
        datenfeld[a].ref3=0;
        if (datenfeld[a].ref4==0)
          {datenfeld[a].ref4=1;}
        else
          {datenfeld[a].ref4=0;}
      }
      else
      {
        datenfeld[a].ref3=1;
      }
    }
  }
}

void LED_On(byte chipIndex, byte ledNumber) {  // this function sets the status of the appropriate output of the chip, but doesn't send it yet
    mm5451[chipIndex].setOutput(ledNumber, true);
}
void LED_Off(byte chipIndex, byte ledNumber) {
    mm5451[chipIndex].setOutput(ledNumber, false);
}

void debugDatabits(byte x) {  // x = number of mm5451anz
	for (int i=0; i<LASTLEDPIN; i++) {
		char msg[10]  = "";
		itoa(i, msg, 10);
		strcat(msg, ( (mm5451[x].getOutput(i))? "ON":"OFF" ));
		SendMessage(msg,1);
	}
}


void SetupLED_MM5451()
{
	SendMessage("setup MM5451",1);
	//for (byte i=0; i<1; i++) {
		for (byte x=0; x<mm5451anz; x++) {
			mm5451[x].lightAll();	
			//debugDatabits(x);
		}	
		delay(800);
		for (byte x=0; x<mm5451anz; x++) {		
			mm5451[x].clearAll();
			//debugDatabits(x);
		}
		delay(800);
	//}
	for (byte x=0; x<mm5451anz; x++) {
        mm5451[x].lightAll();
    }	
    delay(1000);
	for (byte x=0; x<mm5451anz; x++) {
        mm5451[x].clearAll();
		//debugDatabits(x);
	}		
	//delay(1000);
	
/*  for (byte x=0; x<mm5451anz; x++) {
        mm5451[x].clearAll();		
		for (int i = 0; i< LASTLEDPIN; i++) {
			LED_On(x, i);
			mm5451[x].outputDataBits();
			delay(100);			
		}
		delay(1000);
		//debugDatabits(x);
	}	*/

}

void debugTime(int x) {
  char debugmsg[20];
  ltoa(millis(),debugmsg,10);
  SendMessage(debugmsg,1);
}


void UpdateLED_MM5451(byte p)
{    
  // set "----" fields to BUC value;
  if ((datenfeld[BUCINDEX].wert[0]=='T') || (datenfeld[BUCINDEX].wert[0]=='1'))
  {			
	LED_On(0, 21);
	LED_On(0, 25);
	LED_On(0, 26);
	LED_On(0, 27);
	LED_On(0, 29);
	LED_On(0, 30);
  } else {
	LED_Off(0, 21);
	LED_Off(0, 25);
	LED_Off(0, 26);
	LED_Off(0, 27);
	LED_Off(0, 29);
	LED_Off(0, 30);
  }
  // iterate through all LED datenfelds and set Output static
  for (byte i=LAMPS_STARTINDEX; i<LAMPS_STARTINDEX+LAMPCOUNT; i++) {
	if ((datenfeld[i].wert[0]=='T') || (datenfeld[i].wert[0]=='1'))
	{
		LED_On(datenfeld[i].target,datenfeld[i].ref2);
	} else {
		LED_Off(datenfeld[i].target,datenfeld[i].ref2);
	}
  }
  
  // if the first character is T(rue) or 1 (on, no blink), the LED will be turned on
  if ((datenfeld[p].wert[0]=='T') || (datenfeld[p].wert[0]=='1'))
  {
    LED_On(datenfeld[p].target,datenfeld[p].ref2);
  }
  else if (datenfeld[p].wert[0]=='3') //fast blinking LED
  {
    if (datenfeld[p].ref3==1)
    { LED_On(datenfeld[p].target,datenfeld[p].ref2); }
    else
    { LED_Off(datenfeld[p].target,datenfeld[p].ref2); }
  }
  else if (datenfeld[p].wert[0]=='2') //slow blinking LED
  {
    if (datenfeld[p].ref4==1)
    { LED_On(datenfeld[p].target,datenfeld[p].ref2); }
    else
    { LED_Off(datenfeld[p].target,datenfeld[p].ref2);}
  }
  else         // otherwise the LED will be turned off
  {
    LED_Off(datenfeld[p].target,datenfeld[p].ref2);
  }
  
  mm5451[(datenfeld[p].target)].outputDataBits();  // let MM5451 chip send out all LED data bits
  
  if (millis()>LEDTimer+BLINKSPEED)
  { UpdateBlink(); }
   
}
