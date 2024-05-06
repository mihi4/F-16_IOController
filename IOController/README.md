Multiple Arduino ProMicros (APM) will be used to either control inputs or outputs (LEDs, servos, displays).


# Right Console

## KY58 
1 APM
* **!!! pins 2/3 for i2c bus !!!**


### direct input pins:
* A3 VOLUME
* A2 FILL (voltage divider)
* A1 MODE (voltage divider)
* 14/15/16 TD
* 4/5/6/7/8/9 ANTI-ICE

### 23017 IOEs
* HUD
* AVPWR
* SNSPWR/NUC/AIRCOND

# Left Console

## UHF
1 APM
1 internal 23017

### direct input pins
* A0/A1/A2/A3 - frequenzy selectors (voltage divider)
* A10 - channel selector (voltage divider)
* A9 - VOLUME

### 23017
all digital inputs

## AUDIO 1
1 APM
**!!! pins 2/3 for i2c bus !!!**
### direct inputs pins:
* A3,A2,A1,A0,A10,A9 AUDIO 1 potis
* A6,A7,A8 AUDIO 2 potis
* 14/15 control MM5451 on ECM

### 23017 IOEs
* AUDIO 1/2 (10 DIn),  ECM 6 DIn
* AVTR (7 DIn, 2 DOut)
* ENG START, MPO, EPU (9 DI, 5 DO)
* ELEC (3 DI, 9 DO)

~~## (ECM)~~
~~* 1 APM (alternatively use 6 inputs on AUDIO 1/2 23017) and other pins on the AUDIO 1 APM to drive the MM5451~~

~~### direct input pins:~~
~~* 4/5 PWR~~
~~* 6/7 XMIT~~
~~* 8 RESET~~
~~* 9 BIT~~
~~* 14/15 - 1 MM5451 (dim poti direct on MM5451)~~

## EXT LIGHTING
1 APM
**!!! pins 2/3 for i2c bus !!!** (23017 + servocard)
### direct pins
- A0,A1 FORM, AERIAL REFUELING
- A2,A3 ANTI-COLL, MASTER
- A6,A7,A8 MANTRIM
- A10 IFF MASTER
- 14/15/16 IFF 7219

### 23017
- EXTLIGHTS, FUEL, MANTRIM (5+7+1)
- TEST (8+4) + 2 external LEDs
- FLTCTRL (6+2)
