

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
// there are 9, but in the prototype there's an extra one used for level shifting (???)
#define PC 13
#define ENA33 10




// stuff for bell's Theorem testing
// fixed point, 3 digit representation of cosine squared, for integer angles 0 - 90
const unsigned short PROGMEM cos2Table[] = { // 3E8 == 1000 decimal
  0x3E8,0x3E8,0x3E6,0x3E6,0x3E4,0x3E0,0x3DE,0x3DA,0x3D4,0x3D0,
  0x3CA,0x3C4,0x3BC,0x3B5,0x3AD,0x3A5,0x39C,0x392,0x388,0x37F,
  0x374,0x368,0x35B,0x350,0x343,0x335,0x328,0x31A,0x30C,0x2FE,
  0x2EE,0x2DE,0x2CF,0x2C0,0x2AF,0x29F,0x28E,0x27E,0x26D,0x25C,
  0x24B,0x23A,0x228,0x216,0x205,0x1F4,0x1E3,0x1D1,0x1C0,0x1AE,
  0x19D,0x18C,0x17B,0x16A,0x15A,0x149,0x138,0x129,0x119,0x109,
  0xFA,0xEB,0xDC,0xCE,0xC0,0xB3,0xA6,0x99,0x8D,0x80,
  0x75,0x6A,0x5F,0x55,0x4C,0x43,0x3B,0x33,0x2B,0x24,
  0x1E,0x18,0x13,0xF,0xB,0x8,0x5,0x3,0x1,0x0,0x0
};


// Gamma brightness lookup table <https://victornpb.github.io/gamma-table-generator>
// gamma = 2.5 steps = 1024 range = 0-1023
const uint16_t PROGMEM gamma10[] = {
     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
     0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
     1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,
     2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   3,   3,   3,
     3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   4,   4,   4,   4,   4,   4,
     4,   4,   4,   4,   4,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   6,
     6,   6,   6,   6,   6,   6,   6,   6,   7,   7,   7,   7,   7,   7,   7,   7,
     8,   8,   8,   8,   8,   8,   8,   9,   9,   9,   9,   9,   9,   9,  10,  10,
    10,  10,  10,  10,  11,  11,  11,  11,  11,  11,  12,  12,  12,  12,  12,  12,
    13,  13,  13,  13,  13,  13,  14,  14,  14,  14,  14,  15,  15,  15,  15,  15,
    16,  16,  16,  16,  16,  17,  17,  17,  17,  18,  18,  18,  18,  18,  19,  19,
    19,  19,  20,  20,  20,  20,  20,  21,  21,  21,  21,  22,  22,  22,  22,  23,
    23,  23,  23,  24,  24,  24,  25,  25,  25,  25,  26,  26,  26,  26,  27,  27,
    27,  28,  28,  28,  28,  29,  29,  29,  30,  30,  30,  31,  31,  31,  31,  32,
    32,  32,  33,  33,  33,  34,  34,  34,  35,  35,  35,  36,  36,  36,  37,  37,
    37,  38,  38,  38,  39,  39,  39,  40,  40,  40,  41,  41,  42,  42,  42,  43,
    43,  43,  44,  44,  45,  45,  45,  46,  46,  46,  47,  47,  48,  48,  48,  49,
    49,  50,  50,  50,  51,  51,  52,  52,  53,  53,  53,  54,  54,  55,  55,  56,
    56,  56,  57,  57,  58,  58,  59,  59,  60,  60,  60,  61,  61,  62,  62,  63,
    63,  64,  64,  65,  65,  66,  66,  67,  67,  68,  68,  69,  69,  70,  70,  71,
    71,  72,  72,  73,  73,  74,  74,  75,  75,  76,  76,  77,  77,  78,  78,  79,
    79,  80,  80,  81,  82,  82,  83,  83,  84,  84,  85,  85,  86,  87,  87,  88,
    88,  89,  89,  90,  91,  91,  92,  92,  93,  94,  94,  95,  95,  96,  97,  97,
    98,  98,  99, 100, 100, 101, 102, 102, 103, 103, 104, 105, 105, 106, 107, 107,
   108, 109, 109, 110, 110, 111, 112, 112, 113, 114, 114, 115, 116, 117, 117, 118,
   119, 119, 120, 121, 121, 122, 123, 123, 124, 125, 126, 126, 127, 128, 128, 129,
   130, 131, 131, 132, 133, 133, 134, 135, 136, 136, 137, 138, 139, 139, 140, 141,
   142, 143, 143, 144, 145, 146, 146, 147, 148, 149, 149, 150, 151, 152, 153, 153,
   154, 155, 156, 157, 158, 158, 159, 160, 161, 162, 162, 163, 164, 165, 166, 167,
   167, 168, 169, 170, 171, 172, 173, 173, 174, 175, 176, 177, 178, 179, 180, 180,
   181, 182, 183, 184, 185, 186, 187, 188, 188, 189, 190, 191, 192, 193, 194, 195,
   196, 197, 198, 199, 200, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210,
   211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226,
   227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242,
   243, 244, 245, 247, 248, 249, 250, 251, 252, 253, 254, 255, 256, 257, 258, 260,
   261, 262, 263, 264, 265, 266, 267, 268, 270, 271, 272, 273, 274, 275, 276, 277,
   279, 280, 281, 282, 283, 284, 286, 287, 288, 289, 290, 291, 293, 294, 295, 296,
   297, 298, 300, 301, 302, 303, 304, 306, 307, 308, 309, 311, 312, 313, 314, 315,
   317, 318, 319, 320, 322, 323, 324, 325, 327, 328, 329, 330, 332, 333, 334, 336,
   337, 338, 339, 341, 342, 343, 345, 346, 347, 349, 350, 351, 352, 354, 355, 356,
   358, 359, 360, 362, 363, 364, 366, 367, 369, 370, 371, 373, 374, 375, 377, 378,
   379, 381, 382, 384, 385, 386, 388, 389, 391, 392, 393, 395, 396, 398, 399, 400,
   402, 403, 405, 406, 408, 409, 411, 412, 413, 415, 416, 418, 419, 421, 422, 424,
   425, 427, 428, 430, 431, 433, 434, 436, 437, 439, 440, 442, 443, 445, 446, 448,
   449, 451, 452, 454, 455, 457, 458, 460, 461, 463, 465, 466, 468, 469, 471, 472,
   474, 476, 477, 479, 480, 482, 483, 485, 487, 488, 490, 491, 493, 495, 496, 498,
   500, 501, 503, 504, 506, 508, 509, 511, 513, 514, 516, 518, 519, 521, 523, 524,
   526, 528, 529, 531, 533, 534, 536, 538, 540, 541, 543, 545, 546, 548, 550, 552,
   553, 555, 557, 558, 560, 562, 564, 565, 567, 569, 571, 572, 574, 576, 578, 580,
   581, 583, 585, 587, 588, 590, 592, 594, 596, 597, 599, 601, 603, 605, 607, 608,
   610, 612, 614, 616, 618, 619, 621, 623, 625, 627, 629, 631, 632, 634, 636, 638,
   640, 642, 644, 646, 648, 649, 651, 653, 655, 657, 659, 661, 663, 665, 667, 669,
   671, 673, 674, 676, 678, 680, 682, 684, 686, 688, 690, 692, 694, 696, 698, 700,
   702, 704, 706, 708, 710, 712, 714, 716, 718, 720, 722, 724, 726, 728, 730, 732,
   734, 736, 739, 741, 743, 745, 747, 749, 751, 753, 755, 757, 759, 761, 763, 766,
   768, 770, 772, 774, 776, 778, 780, 782, 785, 787, 789, 791, 793, 795, 797, 800,
   802, 804, 806, 808, 810, 813, 815, 817, 819, 821, 824, 826, 828, 830, 832, 835,
   837, 839, 841, 843, 846, 848, 850, 852, 855, 857, 859, 861, 864, 866, 868, 870,
   873, 875, 877, 880, 882, 884, 886, 889, 891, 893, 896, 898, 900, 903, 905, 907,
   910, 912, 914, 917, 919, 921, 924, 926, 928, 931, 933, 935, 938, 940, 942, 945,
   947, 950, 952, 954, 957, 959, 962, 964, 966, 969, 971, 974, 976, 979, 981, 983,
   986, 988, 991, 993, 996, 998,1001,1003,1006,1008,1011,1013,1016,1018,1021,1023,
  };

// ^^ unused ?


// Global variables for the encoders that are saved between calls:
const uint8_t encPins[8] = {ENCAA, ENCAB, ENCBA, ENCBB, ENCCA, ENCCB, ENCDA, ENCDB};//encoder pins defined above
static uint8_t enc_PrevNextCode[4] = {0, 0, 0, 0};//used by enc driver
static uint16_t enc_Store[4] = {0, 0, 0, 0};//used by enc driver
const bool ENC_DIRECTION = 0;//this can be changed in options for wrong way encoders

int enc_buttons[4] = { HIGH, HIGH, HIGH, HIGH }; //normal state
unsigned long enc_button_timers[4] = { 0, 0, 0, 0 };  // milliseconds need big numbers
byte enc_button_pins[4] = { SWA, SWB, SWC, SWD };
Bounce debA=Bounce();
Bounce debB=Bounce();
Bounce debC=Bounce();
Bounce debD=Bounce();
Bounce *debounce[4] = {&debA, &debB, &debC, &debD} ; 

byte cv_in_pins[4] = { CVA, CVB, CVC, CVD };
long cv_values[4] = { 0, 0, 0, 0 };

byte resolution = 10;
const unsigned short maximumOutput = 0x3FF; //1023
// we start at 0 degrees, so everything is maxed out (until such time as we start taking input)
short value[13] = { maximumOutput, maximumOutput, maximumOutput, maximumOutput,
                    maximumOutput, maximumOutput, maximumOutput, maximumOutput,
                    maximumOutput, maximumOutput, maximumOutput, maximumOutput,
                    maximumOutput };
// positions correspond to the value table
byte pinTable[13] = {
  A, B, C, D, AB, BC, CD, DA, ABC, BCD, CDA, DAB, ABCD
};

// what are the pixel numbers of each position; now includes chip drivers for encoders
byte pixelTable[13] = {
  0, 12, 10, 4, 7, 11, 5, 1, 8, 9, 3, 2, 6
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


// some PIXELS are 400KHz, some are 800KHz.
OctoWS2811 leds(ledsPerStrip, displayMemory, drawingMemory, WS2811_RGB | WS2811_800kHz, numPins, pinList);

// decimal 75 (brightness determined previously, 75/255) is 0x4B
#define RED    0x4B0000
#define GREEN  0x004B00
#define BLUE   0x00004B
#define YELLOW 0x4B4B00
#define PINK   0x4B0528
#define ORANGE 0x471A00
#define MYWHITE  0x4B4B4B

int HueTable[13] = { RED, ORANGE, YELLOW, GREEN, BLUE, MYWHITE, PINK, RED, ORANGE, YELLOW, GREEN, BLUE, MYWHITE }; 


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

  startupSeq(); // eventually something to blink lights and show we're alive
  Serial.println("startup done");
}


void loop() { 
  startupSeq();
  delay(2);
}


void startupSeq() {
  for (int testPixel = 0; testPixel < 13; testPixel++) {
    for (int i = 0; i < 13; i++) {
      Serial.print("setting led ");
      Serial.print(testPixel);
      Serial.print(" to color ");
      Serial.print(i);
      Serial.println();
      leds.setPixel(testPixel, HueTable[i]);
      leds.show();
      delayMicroseconds(300000); //
    }
    //for (int i = 0; i < 13; i++) {
      int i = 0;
      Serial.print("unsetting led ");
      Serial.print(testPixel);
      Serial.println();
      leds.setPixel(testPixel, 0);
      leds.show();
      delayMicroseconds(250);
  }
  return;
}
  
