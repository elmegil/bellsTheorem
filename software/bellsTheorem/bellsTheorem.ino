/*
 * Copyright (c) 2023 Frogleg Synthesis/Pete Hartman
 * 
 * Many thanks due to Jim Matheson for advice and coaching on how to work with Teensy, as well as some of the hardware details
 * This project wouldn't have been undertaken without the questions from Nyles Miszczyk on Facebook's Synth DIY group, about Bell's Theorem
 * 
 */

#include <Bounce2.h>
#include <EEPROM.h> // for saving state, eventually
//#include <Adafruit_NeoPixel.h>
// convert to use Paul's OctoWS2811 library instead
#include <OctoWS2811.h>
#include <ADC.h>
#include <IntervalTimer.h>


/*
   EEPROM notes

   EEPROM.Put(address, object)
   EEPROM.Get(address, object)

   use sizeof(object) to increment the address
   no specific address map, just an array of bytes
*/

//#define DEBUG 1

// New revision of the board 0.4+ uses pin 20 as a NeoPixel serial control for the LEDs *including* the
// LEDs for the encoders (by way of WS2811 chips)
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
#define PC 13
#define ENA33 10

// fixed point, 3 digit representation of cosine squared, for integer angles 0 - 90
const unsigned short PROGMEM cos2Table[] = { // 3E8 == 1000 decimal
  0x3E8, 0x3E8, 0x3E6, 0x3E6, 0x3E4, 0x3E0, 0x3DE, 0x3DA, 0x3D4, 0x3D0,
  0x3CA, 0x3C4, 0x3BC, 0x3B5, 0x3AD, 0x3A5, 0x39C, 0x392, 0x388, 0x37F,
  0x374, 0x368, 0x35B, 0x350, 0x343, 0x335, 0x328, 0x31A, 0x30C, 0x2FE,
  0x2EE, 0x2DE, 0x2CF, 0x2C0, 0x2AF, 0x29F, 0x28E, 0x27E, 0x26D, 0x25C,
  0x24B, 0x23A, 0x228, 0x216, 0x205, 0x1F4, 0x1E3, 0x1D1, 0x1C0, 0x1AE,
  0x19D, 0x18C, 0x17B, 0x16A, 0x15A, 0x149, 0x138, 0x129, 0x119, 0x109,
  0xFA,  0xEB,  0xDC,  0xCE,  0xC0,  0xB3,  0xA6,  0x99,  0x8D,  0x80,
  0x75,  0x6A,  0x5F,  0x55,  0x4C,  0x43,  0x3B,  0x33,  0x2B,  0x24,
  0x1E,  0x18,  0x13,  0xF,   0xB,   0x8,   0x5,   0x3,   0x1,   0x0,   0x0
};

// Global variables for the encoders that are saved between calls:
const uint8_t encPins[8] = {ENCAA, ENCAB, ENCBA, ENCBB, ENCCA, ENCCB, ENCDA, ENCDB};//encoder pins defined above
static uint8_t enc_PrevNextCode[4] = {0, 0, 0, 0};//used by enc driver
static short enc_Store[4] = {0, 0, 0, 0};//used by enc driver
const bool ENC_DIRECTION = 0;//this can be changed in options for wrong way encoders

int enc_buttons[4] = { HIGH, HIGH, HIGH, HIGH }; //normal state
unsigned long enc_button_timers[4] = { 0, 0, 0, 0 };  // milliseconds need big numbers
byte enc_button_pins[4] = { SWA, SWB, SWC, SWD };
Bounce debA = Bounce();
Bounce debB = Bounce();
Bounce debC = Bounce();
Bounce debD = Bounce();
Bounce *debounce[4] = {&debA, &debB, &debC, &debD} ;

byte resolution = 10;
uint16_t maximumOutput = pow(2,resolution)-1;

// the following arrays all align
// value[0] corresponds to pinTable[0] corresponds to pixelTable[0]
// these are the actual value, the pin for the output to a jack, and the pixel number for an LED
// we start at 0 degrees, so everything is maxed out (until such time as we start taking input)
// first 4 are zero trying to take LEDs on the encoders out of the current draw
uint16_t value[13] = { maximumOutput, maximumOutput, maximumOutput, maximumOutput,
                    maximumOutput, maximumOutput, maximumOutput, maximumOutput,
                    maximumOutput, maximumOutput, maximumOutput, maximumOutput,
                    maximumOutput
                  };

// positions correspond to the value table
byte pinTable[13] = {
  A, B, C, D, AB, BC, CD, DA, ABC, BCD, CDA, DAB, ABCD
};

// 2023-03-16 pixel numbers are being shuffled because the WS2811 chips cannot come before
// any of the standalone LEDs; some brands (e.g. EMSL) of the LEDs don't work if they're
// positioned after the WS2811 chips.
//
// what are the pixel numbers of each position; now includes chip drivers for encoders
// the order below matches the hacked up re-ordered LED order, NOT the final version on PCB

// version 5 == 0.5, 6 == 0.6, presumably 10 will be 1.0 :)
#define hwversion 5

// 0.6 order
byte pixelTable[13] = {
  12, 9, 10, 11, 5, 8, 3, 0, 6, 7, 2, 1, 4
};

// WS2811 setup
// Any group of digital pins may be used
const int numPins = 1;
byte pinList[numPins] = {20};
const int ledsPerStrip = 13;

// These buffers need to be large enough for all the pixels.
// The total number of pixels is "ledsPerStrip * numPins".
// Each pixel needs 3 bytes, so multiply by 3.  An "int" is
// 4 bytes, so divide by 4.  The array is created using "int"
// so the compiler will align it to 32 bit memory.
DMAMEM int displayMemory[ledsPerStrip * numPins * 3 / 4];
int drawingMemory[ledsPerStrip * numPins * 3 / 4];


OctoWS2811 leds(ledsPerStrip, displayMemory, drawingMemory, WS2811_RGB | WS2811_800kHz, numPins, pinList);

// Pauls lib uses RGB absolute values rather than HSV and brightness

// base colors, need to scale for brightness
//#define RED    0xFF0000
//#define GREEN  0x00FF00
//#define BLUE   0x0000FF
//#define YELLOW 0xFFFF00
//#define PINK   0xFF1088
//#define ORANGE 0xE05800
//#define WHITE  0xFFFFFF
//
// decimal 75 (brightness determined previously, 75/255) is 0x4B
#define RED    0x4B0000
#define GREEN  0x004B00
#define BLUE   0x00004B
#define YELLOW 0x4B4B00
#define PINK   0x4B0528
#define ORANGE 0x471A00
#define WHITE  0x6B6B6B
// the knobs seem to need a bit more
#define BRIGHTWHITE 0x6B6B6B
#define knobBright 180
#define ledBright 120

int HueTable[13] = { YELLOW, BLUE, GREEN, RED, YELLOW, YELLOW, YELLOW, YELLOW, ORANGE, ORANGE, ORANGE, ORANGE, RED }; 

uint16_t inputValue = maximumOutput;
uint16_t prevValue = maximumOutput;
boolean inChanged = false; // if nothing changes, why do any extra work?
boolean outChanged = false; // if nothing changes, why do any extra work?

const uint16_t angleScale = 2; // scale the encoder magnitude, otherwise you spend forever to go 180 degrees 
int16_t angle[4] = { 0, 0, 0, 0 };  // absolute angle of each encoder
int16_t prevAngle[4] = { 0, 0, 0, 0 }; // see if we changed
int16_t savedAngle[4] = { 0, 0, 0, 0 }; // "undo" of unintentional clicks
int16_t relativeAngle[6] = { 0, 0, 0, 0, 0, 0 }; // relative angle of each pair A->B, B->C, C->D, D->A, plus A->C and B->D
int16_t angle_sum[4] = { 0, 0, 0, 0 }; // hold angle + cv

IntervalTimer ioTimer, ctlTimer;

int brightness;  // since we are using "white" we need to be able to set this up at each byte of the RGB set and map to correct level

//
// ADC lib setup
//
byte inPins[5] = { CV_IN, CVA, CVB, CVC, CVD };
uint16_t inValues[5] = { 0, 0, 0, 0, 0 };
bool inFlags[5] = { false, false, false, false, false };

ADC *adc = new ADC();

void setup() {
  if (hwversion == 5) { // hw 0.5
    pixelTable[0] = 10;
    pixelTable[2] = 12; 
  } else { // right now just version 0.6
     // do nothing, we set it up correctly in the definition
  }
  
  // enable the 3.3V regulator!
  pinMode(ENA33, OUTPUT);
  digitalWrite(ENA33, HIGH);

  // set up for encoder button debouncing
  for (int i = 0; i < 4; i++) {
    debounce[i]->attach(enc_button_pins[i], INPUT);
    debounce[i]->interval(25); // 25ms debounce time
  }

  // analog write
  for (int i = 0; i < 13; i++) {
    analogWriteFrequency(pinTable[i], 146484.38); // was 146484.38
  }
  analogWriteResolution(resolution);

  // input setup
  for (int i = 0; i < 5; i++) {
    pinMode(inPins[i], INPUT); 
  }
  ////// ADC1 //////
  // Except for A16, the analog inputs I've selected only work with ADC1
  adc->adc1->setAveraging(16); // higher than 16 and we stall out
  adc->adc1->setResolution(resolution);
  // it can be any of the ADC_CONVERSION_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED_16BITS, HIGH_SPEED or VERY_HIGH_SPEED
  // see the documentation for more information
  // additionally the conversion speed can also be ADACK_2_4, ADACK_4_0, ADACK_5_2 and ADACK_6_2,
  // where the numbers are the frequency of the ADC clock in MHz and are independent on the bus speed.
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); // change the conversion speed
  // it can be any of the ADC_MED_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED); // change the sampling speed
  // always call the compare functions after changing the resolution!
  //adc->adc0->enableCompare(1.0/3.3*adc->adc0->getMaxValue(), 0); // measurement will be ready if value < 1.0V
  //adc->adc0->enableCompareRange(1.0*adc->adc0->getMaxValue()/3.3, 2.0*adc->adc0->getMaxValue()/3.3, 0, 1); // ready if value lies out of [1.0,2.0] V
  // If you enable interrupts, notice that the isr will read the result, so that isComplete() will return false (most of the time)
  //adc->adc0->enableInterrupts(adc0_isr);
  adc->adc1->setReference(ADC_REFERENCE::REF_3V3); // probably the default
  adc->adc1->startSingleRead(inPins[0]);

  // for DEBUG if needed
  Serial.begin(115200);
  Serial.println("Bell's Theorem startup");
  Serial.print(F("F_CPU = "));
  Serial.print(F_CPU, DEC);
  Serial.println();
  Serial.println(maximumOutput);

  // OctoWS2811 library
  leds.begin();
  leds.show();

  // interrupt timers for main I/O and controls
  ioTimer.begin(doIO, 20.833); // was 20.833 for 48khz
  ioTimer.priority(10); // was 10
  ctlTimer.begin(getControl, 100); // was 333
  ctlTimer.priority(50); // was 50

  startupSeq(); // eventually something to blink lights and show we're alive
  Serial.println("startup done");
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
// 2022-01-16 currently core function complete, modified from above MVC:
//
// "view" analog read main input, write analog outputs
// "controller" read encoders and angle CVs, update angles
// "model" main loop, handle encoder switch presses, do the math if the input is changed, display the NeoPixels
//         display of the neopixels is in the main loop because 1) it's slower and 2) if the pretty lights lag the actual outputs a slight bit, not a big deal
//
// practical limits seem to be about 50Hz to have reasonable input/output correspondence.  
// 100Hz and higher tends to round off a direct output (input -> output without doing the math)
// There is about 1ms lag doing input -> output without the math


void loop() {
    // update button press
    for (int i=0; i < 4; i++) {
      debounce[i]->update();
      if (debounce[i]->fell()) {
        // Serial.print("got button press "); Serial.print(i); Serial.println();
        if (savedAngle[i] == 0) { // we don't have a saved angle
          savedAngle[i] = angle[i];
          angle[i] = 0;
        } else {  // we do have a saved angle, let's restore it
          angle[i] = savedAngle[i];
          savedAngle[i] = 0;
        }
        inChanged = true;
      }
    }
    // display output doesn't have to track as accurately as the actual output
    if (outChanged) {
      for (int i = 0; i < 13; i++ ) {
        if (i >=4) {
          brightness = (int)((value[i]>>4)*ledBright)/255; // *Bright values set up at the top to make
        } else { // knobs need a bit of a bump in brightness // easier to adjust
          brightness = (int)((value[i]>>4)*knobBright)/255;
        }
        leds.setPixel(pixelTable[i], brightness + (brightness<<8) + (brightness<<16));
        delayMicroseconds(250);
      }
      leds.show();
      outChanged = false;
    }
}


// abs(90 - abs( x - 90 )) reflects 0 - 180 to 0-90,90-0 and 0 - (-180) to the same
// need to make this a fake function define
// maybe this is wrong?
int16_t constrainCos2(int16_t angle) {
  return (abs(90 - abs(abs(angle) - 90)));
}

// should these be +/- 180 or 360 ?
int16_t constrainAngle(int16_t angle) {  // limit angles to -180 to +180 
  if (angle > 180) {
    angle -= 180;
  } else if (angle < -180) {
    angle += 180;
  }
  return (angle);
}

// funky modulus appropriate for wrapping at 0, instead of -modulus; good for wrapping around arrays
// no longer needed, unrolled loops
//int mod( int x, int y ){
//   return (x<0 ? ((x+1)%y)+y-1 : x%y);
//}

//You can make an intervaltimer and do lookups and update from that.
//that becomes your sample rate. say you wanted it to be 48khz, divide
//1 million by 48000, thats your interval. (it can be a float). I usually have
//a control loop timer running at about 3 khz, which reads the controls and
//translates them to values the audio rate timer can use, then the audio rate
//doesn’t have so much work to do. you can set the audio timer to high
//priority so other things don’t block it. -- JM 2021-10-05

void doIO() {  // violates the rules of "don't call other stuff"
  // set up to work with ADC library instead
  static byte current = 0;
  // if position[current] is ready read into the value array, set flag that it's valid, and increment current (with wrap)
  if ((inFlags[current] = adc->adc1->isComplete())) {
    // Serial.println("complete");
    //if (current != 0) { 
      inValues[current] = adc->adc1->readSingle();
    //} else { // don't invert main in (ONLY FOR REV 0.5+) <<< ? need to revisit this 
      inValues[current] = maximumOutput - inValues[current]; 
    //}
    current++;
    if (current > 4) {
      current = 0;
    }
  }
  if (inFlags[0]) {
    inputValue = (uint16_t)(inValues[0]);
    // inputValue = (uint16_t)(2 * abs(inValues[0] - (maximumOutput+1)/2));
    inFlags[0] = false;
  } 
  if ((inputValue != prevValue) || inChanged) {
    prevValue = inputValue;
    relativeAngle[0] = constrainAngle(angle_sum[1] - angle_sum[0]); // B - A
    relativeAngle[1] = constrainAngle(angle_sum[2] - angle_sum[1]); // C - B
    relativeAngle[2] = constrainAngle(angle_sum[3] - angle_sum[2]); // D - C
    relativeAngle[3] = constrainAngle(angle_sum[3] - angle_sum[0]); // D - A
    // now to do A->C and B->D for doing CDA (actually ACD) and DAB (actually ABD)
    relativeAngle[4] = constrainAngle(angle_sum[2] - angle_sum[0]); // C - A
    relativeAngle[5] = constrainAngle(angle_sum[3] - angle_sum[1]); // D - B  
    // A, B, C, D
    value[0] = (uint16_t)((inputValue * cos2Table[constrainCos2(angle_sum[0])]) / 1000); // VA = in * cos2(A)
    value[1] = (uint16_t)((inputValue * cos2Table[constrainCos2(angle_sum[1])]) / 1000); // VB = in * cos2(B)
    value[2] = (uint16_t)((inputValue * cos2Table[constrainCos2(angle_sum[2])]) / 1000); // VC = in * cos2(C)
    value[3] = (uint16_t)((inputValue * cos2Table[constrainCos2(angle_sum[3])]) / 1000); // VD = in * cos2(D)
    // AB, BC, CD, DA (AD)
    value[4] = (uint16_t)((value[0] * cos2Table[constrainCos2(relativeAngle[0])]) / 1000); // VAB = VA * cos2(AB)
    value[5] = (uint16_t)((value[1] * cos2Table[constrainCos2(relativeAngle[1])]) / 1000); // VBC = VB * cos2(BC)
    value[6] = (uint16_t)((value[2] * cos2Table[constrainCos2(relativeAngle[2])]) / 1000); // VCD = VC * cos2(CD)
    // position 7 is DA, should use A value * AD angle, not D value * AD angle
    value[7] = (uint16_t)((value[0] * cos2Table[constrainCos2(relativeAngle[3])]) / 1000); // VDA = VA * cos2(AD)
    // ABC, BCD, CDA (ACD), DAB (ABD)
    value[8] = (uint16_t)((value[4] * cos2Table[constrainCos2(relativeAngle[1])]) / 1000); // VABC = VAB * cos2(BC)
    value[9] = (uint16_t)((value[5] * cos2Table[constrainCos2(relativeAngle[2])]) / 1000); // VBCD = VBC * cos2(CD)
    // AC is special, there is no intermediate AC created before this                      VCDA = VACD = VA * cos2(AC) * cos2(CD)
    value[10] = (uint16_t)((value[0] * cos2Table[constrainCos2(relativeAngle[4])] * cos2Table[constrainCos2(relativeAngle[2])]) / 1000000); // is this overflowing?
    value[11] = (uint16_t)((value[4] * cos2Table[constrainCos2(relativeAngle[5])]) / 1000); // VDAB = VABD = VAB * cos2(BD)
    value[12] = (uint16_t)((value[8] * cos2Table[constrainCos2(relativeAngle[2])]) / 1000); // VABCD = VABC * cos2(CD)
    outChanged = true;
    inChanged = false;
    for (int i = 0; i < 13; i++) { 
      analogWrite(pinTable[i], value[i]);
    }
  }
  // start analogRead of [current] which has been incremented above
  adc->adc1->startSingleRead(inPins[current]);
  // angle CV ins are read in getControl, alongside rotary positions
}

/*
 * Input protection with the MCP600X chips changes my input range to -10V = 0, 0V = half, 10V = full 
 * ... except I don't think it does  I'm dividing the input /3 and buffering it with the 0 - 3.3V op amps
 */
void getControl() {
    for (int k=0; k<4; k++) { // looping through the values table
      angle[k] = constrainAngle(angle[k] + read_rotary(k) * angleScale); // angle[k] has the "absolute" angle of that encoder
      if (inFlags[k+1]) {
        angle_sum[k] = constrainAngle((inValues[k+1] * 180 / maximumOutput) + angle[k]);
        if (angle_sum[k] != prevAngle[k]) {
          prevAngle[k] = angle_sum[k];
          inChanged = true;
        }
        inFlags[k+1] = false;
      }
    }
}


// Jim Matheson's encoder driver
// A vald CW or  CCW move returns 1 or -1 , invalid returns 0.
// adapted from original code by John Main - best-microcontroller-projects.com
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
  for (int i = 0; i < 13; i++) {
    leds.setPixel(pixelTable[i], HueTable[i]);
    leds.show();
    delayMicroseconds(250000); //
  }
  for (int i = 0; i < 13; i++) {
    leds.setPixel(pixelTable[i], 0);
    leds.show();
    delayMicroseconds(200);
  }
  return;
}


//notes on ADC pins
//const uint8_t pin_to_channel[] = { // pg 482
//        7,      // 0/A0  AD_B1_02
//        8,      // 1/A1  AD_B1_03
//        12,     // 2/A2  AD_B1_07
//        11,     // 3/A3  AD_B1_06
//        6,      // 4/A4  AD_B1_01
//        5,      // 5/A5  AD_B1_00
//        15,     // 6/A6  AD_B1_10
//        0,      // 7/A7  AD_B1_11
//        13,     // 8/A8  AD_B1_08
//        14,     // 9/A9  AD_B1_09
//        1,      // 24/A10 AD_B0_12 - maybe only on ADC0
//        2,      // 25/A11 AD_B0_13 - maybe only on ADC0
//        128+3,  // 26/A12 AD_B1_14 - only on ADC1, 3
//        128+4,  // 27/A13 AD_B1_15 - only on ADC1, 4
//        7,      // 14/A0  AD_B1_02
//        8,      // 15/A1  AD_B1_03
//        12,     // 16/A2  AD_B1_07
//        11,     // 17/A3  AD_B1_06
//        6,      // 18/A4  AD_B1_01
//        5,      // 19/A5  AD_B1_00
//        15,     // 20/A6  AD_B1_10
//        0,      // 21/A7  AD_B1_11
//        13,     // 22/A8  AD_B1_08
//        14,     // 23/A9  AD_B1_09
//        1,      // 24/A10 AD_B0_12
//        2,      // 25/A11 AD_B0_13
//        128+3,  // 26/A12 AD_B1_14 - only on ADC1, 3
//        128+4,  // 27/A13 AD_B1_15 - only on ADC1, 4
//#ifdef ARDUINO_TEENSY41
//        255,    // 28
//        255,    // 29
//        255,    // 30
//        255,    // 31
//        255,    // 32
//        255,    // 33
//        255,    // 34
//        255,    // 35
//        255,    // 36
//        255,    // 37
//        128+1,  // 38/A14 AD_B1_12 - only on ADC1, 1
//        128+2,  // 39/A15 AD_B1_13 - only on ADC1, 2
//        9,      // 40/A16 AD_B1_04
//        10,     // 41/A17 AD_B1_05
//#endif
//};
