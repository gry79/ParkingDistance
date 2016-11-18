#include <Arduino.h>
#include <LedControl.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "font5x7.h"

const unsigned char scrollText[] PROGMEM = {"12141  \0"};

#define NUM_DEVICES 3

/*
 Now we need a LedControl to work with.
 ***** These pin numbers will probably not work with your hardware *****
 pin 8 is connected to the DataIn
 pin 6 is connected to the CLK
 pin 5 is connected to LOAD
 We have only a single MAX72XX.
 */
LedControl lc = LedControl(8, 6, 5, NUM_DEVICES);

VL53L0X sensor;

const long _scrollDelay = 75;   // adjust scrolling speed

unsigned long _bufferLong [14] = {0};
unsigned int _brightness = 4;
unsigned int _kerningPos = 0;

// Rotate the buffer
void rotateBufferLong(){
    for (int a=0;a<7;a++){                      // Loop 7 times for a 5x7 font
        unsigned long x = _bufferLong [a*2];     // Get low buffer entry
        byte b = bitRead(x,31);                 // Copy high order bit that gets lost in rotation
        x = x<<1;                               // Rotate left one bit
        _bufferLong [a*2] = x;                   // Store new low buffer
        x = _bufferLong [a*2+1];                 // Get high buffer entry
        x = x<<1;                               // Rotate left one bit
        bitWrite(x,0,b);                        // Store saved bit
        _bufferLong [a*2+1] = x;                 // Store new high buffer
    }
}

// Display Buffer on LED matrix
void printBufferLong(){
  for (int a=0;a<7;a++){                    // Loop 7 times for a 5x7 font
    unsigned long x = _bufferLong [a*2+1];   // Get high buffer entry
    byte y = x;                             // Mask off first character
    x = _bufferLong [a*2];                   // Get low buffer entry
    lc.setRow(0,a,y);                       // Send row to relevent MAX7219 chip
    y = (x>>24);                            // Mask off second character
    lc.setRow(1,a,y);                       // Send row to relevent MAX7219 chip
    y = (x>>16);                            // Mask off third character
    lc.setRow(2,a,y);                       // Send row to relevent MAX7219 chip
    //y = (x>>8);                             // Mask off forth character
    //lc.setRow(3,a,y);                       // Send row to relevent MAX7219 chip
  }
}

// Load character into scroll buffer
void loadBufferLong(int ascii){
    if (ascii >= 0x20 && ascii <=0x7f){
        for (int a=0;a<7;a++){                      // Loop 7 times for a 5x7 font
            unsigned long c = pgm_read_byte_near(_font5x7 + ((ascii - 0x20) * 8) + a);     // Index into character table to get row data
            unsigned long x = _bufferLong [a*2];     // Load current scroll buffer
            x = x | c;                              // OR the new character onto end of current
            _bufferLong [a*2] = x;                   // Store in buffer
        }
        byte count = pgm_read_byte_near(_font5x7 +((ascii - 0x20) * 8) + 7);     // Index into character table for kerning data
        for (byte x=0; x<count;x++){
            rotateBufferLong();
            printBufferLong();
            delay(_scrollDelay);
        }
    }
}

// Scroll Message
void scrollMessage(const unsigned char * messageString) {
    int counter = 0;
    int myChar=0;
    do {
        // read back a char
        myChar =  pgm_read_byte_near(messageString + counter);
        if (myChar != 0){
            loadBufferLong(myChar);
        }
        counter++;
    }
    while (myChar != 0);
}

void resetKerningPos() {
  _kerningPos = 0;
}

void printText(const char * messageString, unsigned int kernOffset) {
  unsigned int kerningPos = kernOffset;
  unsigned int i = 0;
  unsigned int device = 0;
  byte buf[NUM_DEVICES][8];
  memset(buf, 0, sizeof(buf[0][0]) * NUM_DEVICES * 8);
  while (true) {
    int ascii = messageString[i];
    if (ascii == 0) {
      break;
    }
    if (ascii >= 0x20 && ascii <= 0x7f) {
      unsigned int kerningLength = pgm_read_byte_near(_font5x7 + ((ascii - 0x20) * 8) + 7); // Index into character table for kerning data
      for (int a = 0; a < 7; a++) { // Loop 7 times for a 5x7 font
          unsigned long c = pgm_read_byte_near(_font5x7 + ((ascii - 0x20) * 8) + a); // Index into character table to get row data
          buf[device][a] |= (c >> kerningPos);
          lc.setRow(device, a, buf[device][a]);
          if (device + 1 < NUM_DEVICES) {
            buf[device + 1][a] |= (c << (8 - kerningPos));
            lc.setRow(device + 1, a, buf[device + 1][a]);
          }
      }
      kerningPos += kerningLength;
      if (kerningPos > 7) {
        kerningPos -= 8;
        device++;
      }
    }
    else {
      break;
    }
    i++;
  }
  for (int i = device + 1; i < NUM_DEVICES; i++) {
    lc.clearDisplay(i);
  }
}

void readAndDisplayRange() {
  uint16_t mm = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) {
    printText("----", 1);
  }
  else {
    char buf[10];
    sprintf(buf, "%d", (mm / 1));
    printText(buf, 1);
  }
}

void setup() {
  for (int i = 0; i < NUM_DEVICES; i++) {
    /*
     The MAX72XX is in power-saving mode on startup,
     we have to do a wakeup call
     */
    lc.shutdown(i, false);
    /* Set the brightness to a medium values */
    lc.setIntensity(i, _brightness);
    /* and clear the display */
    lc.clearDisplay(i);
  }

  Serial.begin(9600);
  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);

  // High accuracy
  sensor.setMeasurementTimingBudget(200000);

  // continuous mode, 100ms
  sensor.startContinuous(200);

  //resetKerningPos();
  //printChar('a');
  //printChar('a');
  //printChar('a');
  //scrollMessage(scrollText);
  //printText("220", 1);
}

void loop() {
  readAndDisplayRange();
}
