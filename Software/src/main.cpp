#include <Arduino.h>
#include <LedControl.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Math.h>
#include <Bounce2.h>
#include <EEPROM.h>
#include "font5x7.h"

const unsigned char welcome[] PROGMEM = {"Welcome!     \0"};
const unsigned char goodBye[] PROGMEM = {"Good bye!     \0"};
const unsigned char initComplete[] PROGMEM = {"Jul-O-Meter v1.0     \0"};

#define NUM_DEVICES 3

#define PIN_I2C_DATA 8
#define PIN_I2C_CLK 6
#define PIN_I2C_LOAD 5

#define PIN_LED_WARN 3
#define PIN_BRIGHT_UP 10
#define PIN_BRIGHT_DOWN 9
#define PIN_SET 13

#define POWER_OFF_MS 60000

//#define DIST_BAR

LedControl lc = LedControl(PIN_I2C_DATA, PIN_I2C_CLK, PIN_I2C_LOAD, NUM_DEVICES);

VL53L0X sensor;

Bounce _btnSet = Bounce();
Bounce _btnBriUp = Bounce();
Bounce _btnBriDown = Bounce();

const long _scrollDelay = 30;   // adjust scrolling speed

unsigned long _bufferLong [14] = {0};
unsigned int _brightness = 8;
uint16_t _zeroCorrection = 0;
unsigned int _kerningPos = 0;
byte _highAccuracy = false;
byte _powerSave = true;
byte _warnBlink = false;

void writeSettings() {
  EEPROM.write(0, 0x55);
  EEPROM.write(1, 0x55);

  // Brightness range from 0..15
  if (_brightness > 15) _brightness = 15;

  EEPROM.write(2, _brightness);
  EEPROM.write(3, (_zeroCorrection >> 8) & 0xFF);
  EEPROM.write(4, _zeroCorrection & 0xFF);
}

void readSettings() {
  _brightness = (uint8_t) EEPROM.read(2) & 0xFF;
  _zeroCorrection = ((uint16_t) EEPROM.read(3) << 8) & 0xFFFF;
  _zeroCorrection += (uint16_t) EEPROM.read(4) & 0xFF;
}

void initEEPROM() {
  if (!(EEPROM.read(0) == 0x55 && EEPROM.read(1) == 0x55)) {
    writeSettings();
  }
}

void blinkWarn() {
  digitalWrite(PIN_LED_WARN, HIGH);
  delay(100);
  digitalWrite(PIN_LED_WARN, LOW);
}

void setLongRange() {
  if (_highAccuracy == true) {
    sensor.stopContinuous();
    // lower the return signal rate limit (default is 0.25 MCPS)
    sensor.setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    sensor.startContinuous(200);
    _highAccuracy = false;
  }
}

void setHighAccuracy() {
  if (_highAccuracy == false) {
    sensor.stopContinuous();
    sensor.setSignalRateLimit(0.25);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);
    sensor.startContinuous(200);
    _highAccuracy = true;
  }
}

void setDisplayBrightness() {
  for (int i = 0; i < NUM_DEVICES; i++) {
    lc.setIntensity(i, _brightness);
  }
}

void setPowersaveMode() {
  if (_powerSave == true) return;
  for (int i = 0; i < NUM_DEVICES; i++) {
    lc.clearDisplay(i);
    lc.shutdown(i, true);
  }
  _powerSave = true;
}

void setOperationMode() {
  if (_powerSave == false) return;
  for (int i = 0; i < NUM_DEVICES; i++) {
    lc.shutdown(i, false);
    lc.clearDisplay(i);
    // Brightness
    lc.setIntensity(i, _brightness);
  }
  _powerSave = false;
}

void clearAll() {
  for (int i = 0; i < NUM_DEVICES; i++) {
    lc.clearDisplay(i);
  }
}

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
  clearAll();
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
          if (device + 1 < NUM_DEVICES) {
            buf[device + 1][a] |= (c << (8 - kerningPos));
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
  for (int i = 0; i < NUM_DEVICES; i++) {
    for (int a = 0; a < 7; a++) { // Loop 7 times for a 5x7 font
      lc.setRow(i, a, buf[i][a]);
    }
  }
}

void setup() {
  pinMode(PIN_SET, INPUT);
  pinMode(PIN_BRIGHT_DOWN, INPUT);
  pinMode(PIN_BRIGHT_UP, INPUT);

  _btnSet.attach(PIN_SET);
  _btnSet.interval(10); // 10 ms

  _btnBriUp.attach(PIN_BRIGHT_UP);
  _btnBriUp.interval(10); // 10 ms

  _btnBriDown.attach(PIN_BRIGHT_DOWN);
  _btnBriDown.interval(10); // 10 ms

  pinMode(PIN_LED_WARN, OUTPUT);
  digitalWrite(PIN_LED_WARN, LOW);

  initEEPROM();
  readSettings();

  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);

  // High accuracy
  sensor.setMeasurementTimingBudget(200000);

  setHighAccuracy();

  setOperationMode();

  scrollMessage(initComplete);
}

void loop() {
  uint16_t mmRead = sensor.readRangeContinuousMillimeters();
  uint16_t mm = mmRead;
  if (mm - _zeroCorrection > mm) {
    mm = 0;
  }
  else {
    mm -= _zeroCorrection;
  }
  static uint16_t lastCm = round(mm / 10.0);
  static unsigned long lastUpdate = millis();

  if (sensor.timeoutOccurred()) {
    printText("ERR", 1);
    mm = UINT16_MAX;
  }
  else {
    char buf[10];
    uint16_t cm = round(mm / 10.0);

    if (cm == 0) {
      _warnBlink = true;
    }
    else {
      _warnBlink = false;
      digitalWrite(PIN_LED_WARN, LOW);
    }

    if (cm < 100) {
      sprintf(buf, "%dcm", cm);
    }
    else if (cm >= 100 && cm < 110) {
      sprintf(buf, "1.0m");
    }
    else {
      sprintf(buf, ">1.1m");
    }
    printText(buf, 1);

#ifdef DIST_BAR
    for (int i = 0; i < NUM_DEVICES; i++) {
      int leds = cm - (16 * i);
      byte val = 0x00;
      switch (leds) {
        case  0: val = 0x00; break;
        case  1:
        case  2: val = B10000000; break;
        case  3:
        case  4: val = B11000000; break;
        case  5:
        case  6: val = B11100000; break;
        case  7:
        case  8: val = B11110000; break;
        case  9:
        case 10: val = B11111000; break;
        case 11:
        case 12: val = B11111100; break;
        case 13:
        case 14: val = B11111110; break;
        case 15:
        case 16: val = B11111111; break;
      }
      if (leds > 16) val = B11111111;
      lc.setRow(i, 7, val);
    }
#endif

    if (cm != lastCm) {
      lastUpdate = millis();
      if (_powerSave == true) {
        setOperationMode();
        if (cm > lastCm) {
          scrollMessage(goodBye);
        }
        else if (cm < lastCm) {
          scrollMessage(welcome);
        }
      }
    }
    else if (millis() - lastUpdate > POWER_OFF_MS) {
      setPowersaveMode();
    }

    lastCm = cm;

    if (_warnBlink == true) {
      digitalWrite(PIN_LED_WARN, digitalRead(PIN_LED_WARN) == HIGH ? LOW : HIGH);
    }

    _btnSet.update();

    if (_btnSet.fell()) {
      _zeroCorrection = mmRead;
      writeSettings();
      blinkWarn();
    }
  }

  _btnBriUp.update();
  _btnBriDown.update();

  if (_btnBriUp.fell()) {
    if (_brightness < 15) {
      _brightness++;
      setDisplayBrightness();
      writeSettings();
      blinkWarn();
    }
  }

  if (_btnBriDown.fell()) {
    if (_brightness > 1) {
      _brightness--;
      setDisplayBrightness();
      writeSettings();
      blinkWarn();
    }
  }
}
