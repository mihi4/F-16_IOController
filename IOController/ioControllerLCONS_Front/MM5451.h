#ifndef MM5451_h
#define MM5451_h

#include <Arduino.h>

class MM5451
{
    public:
        void pulseCLK();
        void outputDataBits();
        void setOutput(unsigned char pin, bool state);
        void clearAll();
        void lightAll();
        void setClockPin(unsigned char pin);
        void setDataPin(unsigned char pin);
    
        MM5451(unsigned char clockpin, unsigned char datapin); 
 
    private:
        void sendStartBit();

        unsigned char CLK;
        unsigned char DATA;  // pins to connect to
        bool databits[35];

};

#endif