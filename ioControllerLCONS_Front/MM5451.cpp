#include <Arduino.h>

#include "MM5451.h"

#define OUTPUTS 35

MM5451::MM5451(unsigned char clockpin, unsigned char datapin) {
    setClockPin(clockpin);
    setDataPin(datapin);
    pinMode(CLK, OUTPUT);
    pinMode(DATA, OUTPUT);
    digitalWrite(CLK, 0);
    digitalWrite(DATA, 0);
    for (byte x = 0; x < OUTPUTS; x++) {
        databits[x] = 0;
    }
}

void MM5451::setClockPin(unsigned char pin) {
    CLK = pin;
}

void MM5451::setDataPin(unsigned char pin) {
    DATA = pin;
}

void MM5451::pulseCLK() {
  digitalWrite(CLK, 1);
  digitalWrite(CLK, 0);
}

void MM5451::outputDataBits() {
    sendStartBit();    
    for(byte x=0; x < OUTPUTS; x++) {
        digitalWrite(DATA, databits[x]);
        pulseCLK();
    }
}

void MM5451::setOutput(unsigned char pin, bool state) {
    databits[pin] = state;
}

void MM5451::lightAll() {
    sendStartBit();
    digitalWrite(DATA, 1);
    for(byte x=0; x < OUTPUTS; x++) {
        pulseCLK();
    }
}

void MM5451::clearAll() {
    sendStartBit();
    digitalWrite(DATA, 0);
    for(byte x=0; x < OUTPUTS; x++) {
        pulseCLK();
    }
}

void MM5451::sendStartBit() {
    digitalWrite(DATA, 1);
    pulseCLK();
}

