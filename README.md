Multiple Arduinos (Mega, Nano, ProMicro) will be used to either control inputs or outputs (LEDs, servos, displays).  
Data extraction from either BMS or DCS will be done over BMSAIT.  
Inputs are either direct pins on the APM (ArduinoProMicro) (OXY panel) or MCP23017 IOExtenders (IOEs)

AT THE MOMENT, ONLY THE RIGHT CONSOLE PART IS FINISHED, ALL OTHERS ARE PLACEHOLDERS!

# Right Console

## KY58 
1 APM
* **!!! pins 2/3 for i2c bus !!!**

### direct input pins:
* A3 VOLUME
* A10 OXYGEN DILUTE LEVER
* 14/15/16 OXYGEN LEVERS
* 4/5/6/7/8/9 ANTI-ICE

### output pins
* A0 Servo of OXYGEN needle

### 23017 IOEs
* HUD
* AVPWR
* SNSPWR/NUC/AIRCOND
* KY58

