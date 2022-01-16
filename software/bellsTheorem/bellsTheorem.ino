

#include <Wire.h>
#include <Bounce2.h>
#include <EEPROM.h> // for saving state, eventually
#include <Adafruit_NeoPixel.h>

/*
   EEPROM notes

   EEPROM.Put(address, object)
   EEPROM.Get(address, object)

   use sizeof(object) to increment the address
   no specific address map, just an array of bytes
*/

#define DEBUG 1

//
// New revision of the board 0.3 now uses pin 20 as a NeoPixel serial control for the LEDs *except* the
// LEDs for the encoders.
//

//  pins used
#define ENCAA 17
#define ENCAB 18
#define SWA 19
#define ENCBA 1
#define ENCBB 0
#define SWB 2
#define ENCCA 5
#define ENCCB 4
#define SWC 3
#define ENCDA 15
#define ENCDB 16
#define SWD 14
#define CVA A15
#define CVB A16
#define CVC A12
#define CVD A13
#define CV_IN A14
#define A 23
#define B 6
#define C 28
#define D 29
#define AB 7
#define BC 9
#define CD 33
#define DA 37
#define ABC 8
#define BCD 25
#define CDA 36
#define DAB 22
#define ABCD 24
#define NP 20
// PixelCount: there are 9, but in the prototype there's an extra one used for level shifting
#define PC 10

// fixed point, 3 digit representation of cosine squared, for integer angles 0 - 90
const unsigned short PROGMEM cos2Table[] = { // 3E8 == 1000 decimal
  0x3E8, 0x3E8, 0x3E6, 0x3E6, 0x3E4, 0x3E0, 0x3DE, 0x3DA, 0x3D4, 0x3D0,
  0x3CA, 0x3C4, 0x3BC, 0x3B5, 0x3AD, 0x3A5, 0x39C, 0x392, 0x388, 0x37F,
  0x374, 0x368, 0x35B, 0x350, 0x343, 0x335, 0x328, 0x31A, 0x30C, 0x2FE,
  0x2EE, 0x2DE, 0x2CF, 0x2C0, 0x2AF, 0x29F, 0x28E, 0x27E, 0x26D, 0x25C,
  0x24B, 0x23A, 0x228, 0x216, 0x205, 0x1F4, 0x1E3, 0x1D1, 0x1C0, 0x1AE,
  0x19D, 0x18C, 0x17B, 0x16A, 0x15A, 0x149, 0x138, 0x129, 0x119, 0x109,
  0xFA, 0xEB, 0xDC, 0xCE, 0xC0, 0xB3, 0xA6, 0x99, 0x8D, 0x80,
  0x75, 0x6A, 0x5F, 0x55, 0x4C, 0x43, 0x3B, 0x33, 0x2B, 0x24,
  0x1E, 0x18, 0x13, 0xF, 0xB, 0x8, 0x5, 0x3, 0x1, 0x0, 0x0
};


// Global variables for the encoders that are saved between calls:
const uint8_t encPins[8] = {ENCAA, ENCAB, ENCBA, ENCBB, ENCCA, ENCCB, ENCDA, ENCDB};//encoder pins defined above
static uint8_t enc_PrevNextCode[4] = {0, 0, 0, 0};//used by enc driver
static uint16_t enc_Store[4] = {0, 0, 0, 0};//used by enc driver
const bool ENC_DIRECTION = 0;//this can be changed in options for wrong way encoders

int enc_buttons[4] = { HIGH, HIGH, HIGH, HIGH }; //normal state
unsigned long enc_button_timers[4] = { 0, 0, 0, 0 };  // milliseconds need big numbers
byte enc_button_pins[4] = { SWA, SWB, SWC, SWD };
Bounce debA = Bounce();
Bounce debB = Bounce();
Bounce debC = Bounce();
Bounce debD = Bounce();
Bounce *debounce[4] = {&debA, &debB, &debC, &debD} ;

byte cv_in_pins[4] = { CVA, CVB, CVC, CVD };
long cv_values[4] = { 0, 0, 0, 0 };

const unsigned short maximumOutput = 1023;
// we start at 0 degrees, so everything is maxed out (until such time as we start taking input)
//short value[13] = { maximumOutput, maximumOutput, maximumOutput, maximumOutput,
//                    maximumOutput, maximumOutput, maximumOutput, maximumOutput,
//                    maximumOutput, maximumOutput, maximumOutput, maximumOutput,
//                    maximumOutput
//                  };
short value[13] = { maximumOutput, 0,0,0,0,0,0,0,0,0,0,0,0 };

// positions correspond to the value table
byte pinTable[13] = {
  A, B, C, D, AB, BC, CD, DA, ABC, BCD, CDA, DAB, ABCD
};

// what are the pixel numbers of each position.  A-D are not used so they are zeroes
byte pixelTable[13] = {
  0, 0, 0, 0, 6, 9, 4, 1, 7, 8, 3, 2, 5
};

unsigned short inputValue = maximumOutput;
unsigned short prevValue = maximumOutput;
boolean valChanged = false; // if nothing changes, why do any extra work?
boolean vc = false; // for use inside the control ISR

const short angleScale = 10;
//const short angleScale = 2; // scale the encoder magnitude, otherwise you spend forever to go 180 degrees
long angle[4] = { 0, 0, 0, 0 };  // absolute angle of each encoder
long prevAngle[4] = { 0, 0, 0, 0 }; // see if we changed
long savedAngle[4] = { 0, 0, 0, 0 }; // "undo" of unintentional clicks
long relativeAngle[6] = { 0, 0, 0, 0, 0, 0 }; // relative angle of each pair A->B, B->C, C->D, D->A, plus A->C and B->D
long angle_sum[4] = { 0, 0, 0, 0 }; // hold angle + cv

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(PC, NP, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

IntervalTimer outputTimer;
IntervalTimer ctlTimer;

// for checking loop times
long lastMicros = 0;
long curMicros = 0;
int count = 0;
// position, temporary
int pos = 0;
short temp = 0;

// do the math in the loop?

void setup() {
  for (int i = 0; i < 4; i++) {
    debounce[i]->attach(enc_button_pins[i], INPUT);
    debounce[i]->interval(25); // 25ms ebounce time
  }

  for (int i = 0; i < 13; i++) {
    analogWriteFrequency(pinTable[i], 146484);
  }
  analogWriteResolution(10);

  for (int i = 0; i < 4; i++) {
    pinMode(cv_in_pins[i], INPUT);
  }
  pinMode(CV_IN, INPUT);
  analogReadRes(10); // 10 bit resolution
  analogReadAveraging(3); // just pick a number to try and reduce jitter

  Serial.begin(115200);
  Serial.println("Bell's Theorem startup");
  Serial.print(F("F_CPU = "));
  Serial.print(F_CPU, DEC);
  Serial.println();

  // NeoPixel LEDs
  strip.begin();
  strip.show();
  strip.setBrightness(50);
  strip.clear();

  outputTimer.begin(doOutput, 20.833); // was 20.833 for 48khz
  outputTimer.priority(10);
  ctlTimer.begin(getControl, 580); // 580 is about the minimum we can use to call this, any faster and it doesn't seem to ever complete
  ctlTimer.priority(50);

  startupSeq(); // eventually something to blink lights and show we're alive
}

//
// model: 4 angles, main input
//    derived -> 13 output values
//
// view: analog write 13 values and display LEDs
//
// control: read encoders and CV ins, update angles
//          read main input and update
//
//   on control change, make update and kick the model to do the calculations
//
// simplified model & task:  single input (encA + CVA), run through cos2 table, single output
// next iteration: two inputs (angle A and main input), same task, same output

void loop() {
//  if (valChanged) {
   // A->B, B->C, C->D, A->D
//      relativeAngle[0] = constrainAngle(angle_sum[1] - angle_sum[0]);
//      relativeAngle[1] = constrainAngle(angle_sum[2] - angle_sum[1]);
//      relativeAngle[2] = constrainAngle(angle_sum[3] - angle_sum[2]);
//      relativeAngle[3] = constrainAngle(angle_sum[3] - angle_sum[0]);
//      // now to do A->C and B->D for doing CDA (actually ACD) and DAB (actually ABD)
//      relativeAngle[4] = constrainAngle(angle_sum[2] - angle_sum[0]); // C - A
//      relativeAngle[5] = constrainAngle(angle_sum[3] - angle_sum[1]); // D - B

  // A, B, C, D
//      value[0] = (short)((inputValue * cos2Table[constrainCos2(angle_sum[0])]) / 1000);
//      value[1] = (short)((inputValue * cos2Table[constrainCos2(angle_sum[1])]) / 1000);
//      value[2] = (short)((inputValue * cos2Table[constrainCos2(angle_sum[2])]) / 1000);
//      value[3] = (short)((inputValue * cos2Table[constrainCos2(angle_sum[3])]) / 1000);
//      // AB, BC, CD, DA (AD)
//      value[4] = (short)((value[0] * cos2Table[constrainCos2(relativeAngle[0])]) / 1000);
//      value[5] = (short)((value[1] * cos2Table[constrainCos2(relativeAngle[1])]) / 1000);
//      value[6] = (short)((value[2] * cos2Table[constrainCos2(relativeAngle[2])]) / 1000);
//      // position 7 is DA, should use A value * AD angle, not D value * AD angle
//      value[7] = (short)((value[0] * cos2Table[constrainCos2(relativeAngle[3])]) / 1000);
//      // ABC, BCD, CDA (ACD), DAB (ABD)
//      value[8] = (short)((value[4] * cos2Table[constrainCos2(relativeAngle[1])]) / 1000);
//      value[9] = (short)((value[5] * cos2Table[constrainCos2(relativeAngle[2])]) / 1000);
//      // AC is special, there is no intermediate AC created before this
//      value[10] = (short)((value[0] * cos2Table[constrainCos2(relativeAngle[4])] * cos2Table[constrainCos2(relativeAngle[2])]) / 1000000);
//      value[11] = (short)((value[4] * cos2Table[constrainCos2(relativeAngle[5])]) / 1000);
//      value[12] = (short)((value[8] * cos2Table[constrainCos2(relativeAngle[2])]) / 1000);
//  } else {
//  }
//  display does not have to be done at full output speed
    for (int i = 4; i < 13; i++) {
      strip.setPixelColor(pixelTable[i], strip.gamma32(strip.ColorHSV(0, 0, value[i] >> 2)));
    }
    strip.show();
}


// abs(90 - abs( x - 90 )) reflects 0 - 180 to 0-90,90-0 and 0 - (-180) to the same
short constrainCos2(short angle) {
  return (abs(90 - abs(abs(angle) - 90)));
}

long constrainAngle(long angle) {  // limit angles to -180 to +180
  if (angle > 180) {
    angle -= 180;
  } else if (angle < -180) {
    angle += 180;
  }
  return (angle);
}

// funky modulus appropriate for wrapping at 0, instead of -modulus; good for wrapping around arrays
// no longer needed, unrolled loops
int mod( int x, int y ){
   return (x<0 ? ((x+1)%y)+y-1 : x%y);
}

//You can make an intervaltimer and do lookups and update from that.
//that becomes your sample rate. say you wanted it to be 48khz, divide
//1 million by 48000, thats your interval. (it can be a float). I usually have
//a control loop timer running at about 3 khz, which reads the controls and
//translates them to values the audio rate timer can use, then the audio rate
//doesn’t have so much work to do. you can set the audio timer to high
//priority so other things don’t block it. -- JM 2021-10-05

void doOutput() {  // violates the rules of "don't call other stuff"
  if (valChanged) {
    valChanged = false;
    for (int i = 0; i < 4; i++) {
      analogWrite(pinTable[i], value[i]);
    }
    for (int i = 4; i < 13; i++) {
      analogWrite(pinTable[i], value[i]);
      // display does not have to happen at full output speed, put it in the loop instead
      //strip.setPixelColor(pixelTable[i], strip.gamma32(strip.ColorHSV(0, 0, value[i] >> 2)));
    }
    //strip.show();
  }
}


void getControl() {
  valChanged = false;

  inputValue = analogRead(CV_IN);
  if (inputValue != prevValue) {
    prevValue = inputValue;
    value[pos] = inputValue; // just straight pass through of the input to output
    vc = true;
  }
//  for (int i=0; i < 4; i++) {
//    debounce[i]->update();
//    if (debounce[i]->fell()) {
//      // Serial.print("got button press "); Serial.print(i); Serial.println();
//      if (savedAngle[i] == 0) { // we don't have a saved angle
//        savedAngle[i] = angle[i];
//        angle[i] = 0;
//      } else {  // we do have a saved angle, let's restore it
//        angle[i] = savedAngle[i];
//        savedAngle[i] = 0;
//      }
//      vc = true;
//    }
//    // might have to take the sum part out and put into the main loop, in which case we'll need 2 "prevAngle" arrays
//    angle[i] = constrainAngle(angle[i] + read_rotary(i) * angleScale);
//    cv_values[i] = (long)(analogRead(cv_in_pins[i]) * .176);  // .176 is ~ 180 degrees / 1023 ; CV is 0 - 180 degrees
//    angle_sum[i] = constrainAngle(cv_values[i] + angle[i]);
//    if (angle_sum[i] != prevAngle[i]) {
//      prevAngle[i] = angle_sum[i];
//      vc = true;
//    }
//  }
//  use rotary to move the "maximumOutput" value through the positions
//  This code works as expected
//    temp = value[pos];
//    value[pos] = 0;
//    pos = mod(pos + read_rotary(0),13); 
//    value[pos] = temp;

  valChanged = vc;
}


// Jim Matheson's encoder driver
// A vald CW or  CCW move returns 1 or -1 , invalid returns 0.
// adapted from original code by John Main - best-microcontroller-projects.com
//
//when you call this you get 1,0,or -1 depending on how the encoder was moved.

int8_t read_rotary(uint8_t num) {
  static int8_t rot_enc_table[] = {0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0};

  enc_PrevNextCode[num] <<= 2;
  if (!ENC_DIRECTION) { //forward
    if (digitalReadFast(encPins[num << 1])) enc_PrevNextCode[num] |= 0x02;
    if (digitalReadFast(encPins[(num << 1) + 1])) enc_PrevNextCode[num] |= 0x01;
  } else {//reverse
    if (digitalReadFast(encPins[num << 1])) enc_PrevNextCode[num] |= 0x02;
    if (digitalReadFast(encPins[(num << 1) + 1])) enc_PrevNextCode[num] |= 0x01;
  }
  enc_PrevNextCode[num] &= 0x0f;

  // If valid then store as 16 bit data.
  if  (rot_enc_table[enc_PrevNextCode[num]] ) {
    enc_Store[num] <<= 4;
    enc_Store[num] |= enc_PrevNextCode[num];
    if ((enc_Store[num] & 0xff) == 0x2b) return 1;
    if ((enc_Store[num] & 0xff) == 0x17) return -1;
  }
  return 0;
}

void startupSeq() {
  return;
}
