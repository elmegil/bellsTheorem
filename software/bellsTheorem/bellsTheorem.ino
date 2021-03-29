

#include <Wire.h>
#include <Bounce2.h>
//#include <ADC.h> // super complex and not very well documented afaict
#include <EEPROM.h> // for saving state

/*
 * EEPROM notes
 * 
 * EEPROM.Put(address, object)
 * EEPROM.Get(address, object)
 * 
 * use sizeof(object) to increment the address
 * no specific address map, just an array of bytes
 */

#define DEBUG 1



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

// stuff for bell's Theorem testing
// fixed point, 3 digit representation of cosine squared, for integer angles 0 - 90
unsigned short cos2Table[] = { // 3E8 == 1000 decimal
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

unsigned short inputValue = maximumOutput;                    

const short angleScale = 2; // scale the encoder magnitude, otherwise you spend forever to go 180 degrees
long angle[4] = { 0, 0, 0, 0 };  // absolute angle of each encoder
long savedAngle[4] = { 0, 0, 0, 0 }; // "undo" of unintentional clicks
long relativeAngle[4] = { 0, 0, 0, 0 }; // relative angle of each pair A->B, B->C, C->D, D->A
long angle_sum[4] = { 0, 0, 0, 0 }; // hold angle + cv

void setup() {  
  for (int i=0; i<4; i++) {
    debounce[i]->attach(enc_button_pins[i], INPUT);
    debounce[i]->interval(25); // 25ms ebounce time
  }
  
  for (int i=0; i<13; i++) {
    // pinMode(pinTable[i], OUTPUT); // not necessary and actually "OUTPUT" is a digital output
    analogWriteFrequency(pinTable[i], 146484);
  }
  analogWriteResolution(10);

  for (int i=0; i<4; i++) {
    pinMode(cv_in_pins[i], INPUT);
  }
  pinMode(CV_IN, INPUT);
  analogReadRes(10); // 10 bit resolution
  analogReadAveraging(3); // just pick a number to try and reduce jitter
  
  Serial.begin(115200);
  Serial.println("Bell's Theorem startup");
  Serial.print(F("F_CPU = "));
  Serial.print(F_CPU,DEC);
  Serial.println();
}

// for checking loop times
long lastMicros = 0;
long curMicros = 0;

// more test bits
int testvalue = 0;

void loop() {  
  //curMicros = micros();
  //Serial.print("loop time: "); Serial.print(curMicros - lastMicros); Serial.println();
  // Serial.print("test value: "); Serial.print(testvalue); Serial.println();
  //lastMicros=curMicros;
  for (short i=0; i<13; i++) {
    analogWrite(pinTable[i], testvalue);
  }
//  testvalue+=1;
//  if (testvalue > 100) {
//    testvalue = 0;
//  }
//  delay(100); // ms
  // use duration() method later for mode changes
  //Serial.println("updating buttons");
  // consider long press saving to EEPROM or restoring from EEPROM
  for (int i=0; i < 4; i++) {
    debounce[i]->update();
    if (debounce[i]->fell()) {
      Serial.print("got button press "); Serial.print(i); Serial.println();
      if (savedAngle[i] == 0) { // we don't have a saved angle
        savedAngle[i] = angle[i];
        angle[i] = 0;
      } else {  // we do have a saved angle, let's restore it
        angle[i] = savedAngle[i];
        savedAngle[i] = 0;
      }
    }
    
    long prevAngle = angle[i];
    angle[i] = constrainAngle(angle[i] + read_rotary(i) * angleScale);
    //cv_values[i] = (long)(analogRead(cv_in_pins[i]) * .176);  // .176 is ~ 180 degrees / 1023 ; CV is 0 - 180 degrees
    cv_values[i] = 0;  // THERE IS SIGNIFICNANT JITTER IN THE ANALOG READ HERE
    angle_sum[i] = constrainAngle(cv_values[i] + angle[i]);
    if (prevAngle != angle[i]) {
      Serial.print("angle "); Serial.print(i); Serial.print(" : "); Serial.print(angle[i]); Serial.println();
    }
  }
   
  // ok angle is set, now check the relationship to adjacent lenses
  for (int i = 0; i < 4; i++) {  
    long prevRelAngle = relativeAngle[i];
    relativeAngle[i] = constrainAngle(angle_sum[mod(i+1,4)] - angle_sum[i]); // maximum difference is 360
    if (prevRelAngle != relativeAngle[i]) {
      Serial.print("relative angle "); Serial.print(i); Serial.print(" : "); Serial.print(relativeAngle[i]); Serial.println();
    }
  }

  //inputValue = analogRead(CV_IN); 
  inputValue = 1023; // for now we're going to see how well the various bits work
  
  for (int i = 0; i < 4; i++) {  // and now we want to do the real calculations based on the relative angles + the CV inputs
    value[i] = (short)((inputValue * cos2Table[constrainCos2(angle_sum[i])]) / 1000);                          
    analogWrite(pinTable[i], value[i]);
    value[i+4] = (short)((value[i] * cos2Table[constrainCos2(relativeAngle[i])]) / 1000);
    analogWrite(pinTable[i+4], value[i+4]);
    value[i+8] = (short)((value[i+4] * cos2Table[constrainCos2(relativeAngle[mod(i+1,4)])]) / 1000);
    analogWrite(pinTable[i+8], value[i+8]);
    if (i == 0) {
      Serial.print(value[i]); Serial.print(" "); Serial.print(value[i+4]); Serial.print (" "); Serial.println(value[i+8]);
    }
  }
  // still gotta do ABCD
  value[12] = (short)((value[8] * cos2Table[constrainCos2(relativeAngle[2])])/1000);
  analogWrite(pinTable[12], value[12]);
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
  return(angle);
}

// funky modulus appropriate for wrapping at 0, instead of -modulus; good for wrapping around arrays
int mod( int x, int y ){
   return (x<0 ? ((x+1)%y)+y-1 : x%y);
}

// Jim Matheson's encoder driver
// A vald CW or  CCW move returns 1 or -1 , invalid returns 0.
// adapted from original code by John Main - best-microcontroller-projects.com
//
//when you call this you get 1,0,or -1 depending on how the encoder was moved.

int8_t read_rotary(uint8_t num) {
  static int8_t rot_enc_table[] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};

  enc_PrevNextCode[num] <<= 2;
  if (!ENC_DIRECTION){//forward
    if (digitalReadFast(encPins[num<<1])) enc_PrevNextCode[num] |= 0x02;
    if (digitalReadFast(encPins[(num<<1)+1])) enc_PrevNextCode[num] |= 0x01;
  } else {//reverse
    if (digitalReadFast(encPins[num<<1])) enc_PrevNextCode[num] |= 0x02;
    if (digitalReadFast(encPins[(num<<1)+1])) enc_PrevNextCode[num] |= 0x01;
  }
  enc_PrevNextCode[num] &= 0x0f;

   // If valid then store as 16 bit data.
   if  (rot_enc_table[enc_PrevNextCode[num]] ) {
      enc_Store[num] <<= 4;
      enc_Store[num] |= enc_PrevNextCode[num];
      //if (store==0xd42b) return 1;
      //if (store==0xe817) return -1;
      if ((enc_Store[num]&0xff)==0x2b) return -1;
      if ((enc_Store[num]&0xff)==0x17) return 1;
   }
   return 0;
}
  
