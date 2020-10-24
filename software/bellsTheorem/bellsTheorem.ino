

#include <Wire.h>
#include <Bounce2.h>
//#include <ADC.h> // super complex and not very well documented afaict

#define DEBUG 1

// addresses for the I2C expander
// WIRE APPENDS THE READ/WRITE BIT!!
#define XPADDR 0x70
// control bytes for the I2C expander
// channel 0 is left side, 3 DACs, 0x61, 62, 63
// channel 1 is right side, 4 DACs, 0x60, 61, 62, 63
#define XPCHAN0 0x04 
#define XPCHAN1 0x05
#define XPCHAN_NONE 0x00

/*
 * DAC -> output mapping
 * 
 *  left side 0x61   dual channel   A / D
 *  left side 0x62   dual channel   B / C
 *  left side 0x63   dual channel   DAB / DA
 *  right side 0x60  single channel ABCD
 *  right side 0x61  dual channel   BCD / CD
 *  right side 0x62  dual channel   BC / CDA
 *  right side 0x63  dual channel   AB / ABC
 */

// address triplets in order, A/B/C/D, AB/BC/CD/DA, ABC/BCD/CDA/DAB, ABCD
// 0 = left, 1 = right, DAC address, unit address within the DAC (Vout0 or Vout1)
byte addr[13][3] = { { 0, 0x61, 0x00}, 
                      { 0, 0x62, 0x00},
                      { 0, 0x62, 0x01},
                      { 0, 0x61, 0x01},
                      { 1, 0x63, 0x00},
                      { 1, 0x62, 0x00},
                      { 1, 0x61, 0x01},
                      { 0, 0x63, 0x01},
                      { 1, 0x63, 0x01},
                      { 1, 0x61, 0x00},
                      { 1, 0x62, 0x01},
                      { 0, 0x63, 0x00},
                      { 1, 0x60, 0x00} };

const unsigned short maximumOutput = 0x3FF; //1023
// we start at 0 degrees, so everything is maxed out (until such time as we start taking input)
short value[13] = { maximumOutput, maximumOutput, maximumOutput, maximumOutput,
                    maximumOutput, maximumOutput, maximumOutput, maximumOutput,
                    maximumOutput, maximumOutput, maximumOutput, maximumOutput,
                    maximumOutput };

// other pins used
// latch the DACs
#define LATCH 0

#define ENCAA 23
#define ENCAB 22
#define SWA 21
#define ENCBA 2
#define ENCBB 3
#define SWB 1
#define ENCCA 4
#define ENCCB 5
#define SWC 6
#define ENCDA 15
#define ENCDB 14
#define SWD 16
#define CVA A15
#define CVB A16
#define CVC A10
#define CVD A11
#define CV_IN 38

#define LEFTSIDE 0
#define RIGHTSIDE 1


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

byte xpstatus;
byte error;

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

long long intermediate;
unsigned short inputValue = maximumOutput;                    

const short angleScale = 2; // scale the encoder magnitude, otherwise you spend forever to go 180 degrees
long angle[4] = { 0, 0, 0, 0 };  // absolute angle of each encoder
long savedAngle[4] = { 0, 0, 0, 0 }; // "undo" of unintentional clicks
long relativeAngle[4] = { 0, 0, 0, 0 }; // relative angle of each pair A->B, B->C, C->D, D-A[]

long angle_sum[4] = { 0, 0, 0, 0 }; // hold angle + cv

// try some tricks to force inlining of simple functions  -- but doesn't seem to make much difference?
inline int mod( int x, int y ) __attribute__((always_inline));
inline long constrainAngle(long angle) __attribute__((always_inline));
inline short constrainCos2(short angle) __attribute__((always_inline));

//
// intermediate = inputValue * cos2Table[0] * cos2Table[0] * cos2Table[0]
//
// final = (short)(intermediate / threeLens)

void setup() {  
  pinMode(LATCH, OUTPUT);
  digitalWrite(LATCH, HIGH); // latch occurs on LTH transition
  for (int i=0; i<4; i++) {
    //pinMode(enc_button_pins[i], INPUT_PULLUP);
    debounce[i]->attach(enc_button_pins[i], INPUT_PULLUP);
    debounce[i]->interval(25); // 25ms ebounce time
  }

  for (int i=0; i<4; i++) {
    pinMode(cv_in_pins[i], INPUT);
  }
  analogReadRes(10); // 10 bit resolution
  analogReadAveraging(3); // just pick a number to try and reduce jitter
  
  Serial.begin(115200);
  Serial.println("Bell's Theorem startup");
  Serial.println(SDA);
  Serial.println(SCL);

  Wire.begin();
  Wire.setClock(400000); // try to use Fast mode.... works, but doesn't seem to make the test loop any faster
  Wire.requestFrom(XPADDR,1);
  while (Wire.available()) {
    xpstatus = Wire.read();
  }  
  //Serial.println(xpstatus); // should be all zeroes at this point
  for (int i = 0; i<13; i++) {  // max everyone out to start because all angles are 0
    selectSide(addr[i][0]);
    sendValue(addr[i][1], addr[i][2], maximumOutput);
  }

  selectSide(LEFTSIDE);     // set as default state
}

// for checking loop times
long lastMillis = 0;
long curMillis = 0;

void loop() {  
  long tempEnc;

  curMillis = micros();
  Serial.print("loop time: "); Serial.print(curMillis - lastMillis); Serial.println();
  lastMillis=curMillis;
  // use duration() method later for mode changes
  //Serial.println("updating buttons");
  for (int i=0; i < 4; i++) {
    debounce[i]->update();
    if (debounce[i]->fell()) {
      // Serial.println("got button press");
      if (savedAngle[i] == 0) { // we don't have a saved angle
        savedAngle[i] = angle[i];
        angle[i] = 0;
      } else {  // we do have a saved angle, let's restore it
        angle[i] = savedAngle[i];
        savedAngle[i] = 0;
      }
    }
  }

  for (int i = 0; i < 4; i++) {  //update angles
    tempEnc = constrainAngle(angle[i] + read_rotary(i) * angleScale);
    angle[i] = tempEnc;
  }

  for (int i = 0; i < 4; i++) {
    cv_values[i] = (long)(analogRead(cv_in_pins[i]) * .176);  // .176 is ~ 180 degrees / 1023 ; CV is 0 - 180 degrees
    angle_sum[i] = constrainAngle(cv_values[i] + angle [i]);
  }

//  Serial.print("angle ");
//  for (int x=0; x<4; x++) { Serial.print(angle[x]); Serial.print(" "); } Serial.println();
    
  // ok angle is set, now check the relationship to adjacent lenses
  for (int i = 0; i < 4; i++) {  
    relativeAngle[i] = constrainAngle(angle_sum[mod(i+1,4)] - angle_sum[i]); // maximum difference is 360
    relativeAngle[mod(i-1,4)] = constrainAngle(angle_sum[i] - angle_sum[mod(i-1,4)]);
  }
  
//  Serial.print("relativeAngle ");
//  for (int x=0; x<4; x++) { Serial.print(relativeAngle[x]); Serial.print(" "); } Serial.println();

  inputValue = analogRead(CV_IN);
  
  for (int i = 0; i < 4; i++) {  // and now we want to do the real calculations based on the relative angles + the CV inputs
    intermediate = inputValue * cos2Table[constrainCos2(angle_sum[i])]; 
    // ultimately this will actually just be A/B/C/D no value + 4
    value[i] = (short)(intermediate / 1000);                          
    selectSide(addr[i][0]);
    sendValue(addr[i][1], addr[i][2], value[i]);
    
    intermediate = value[i] * cos2Table[constrainCos2(relativeAngle[i])];
    value[i+4] = (short)(intermediate / 1000);
    selectSide(addr[i+4][0]);
    sendValue(addr[i+4][1], addr[i+4][2], value[i+4]);

    intermediate = value[i+4] * cos2Table[constrainCos2(relativeAngle[mod(i+1,4)])];
    value[i+8] = (short)(intermediate / 1000);
    selectSide(addr[i+8][0]);
    sendValue(addr[i+8][1], addr[i+8][2], value[i+8]);
//    Serial.print("value ");
//    for (int x=0; x<13; x++) { Serial.print(value[x]); Serial.print(" "); } Serial.println();
  }
  // still gotta do ABCD
  intermediate = value[8] * cos2Table[constrainCos2(relativeAngle[2])];
  value[12] = (short)(intermediate/1000);
  selectSide(addr[12][0]);
  sendValue(addr[12][1], addr[12][2], value[12]);
}

void selectSide(byte side) {
  Wire.beginTransmission(XPADDR);
  if (side == 0) {  
    Wire.write(XPCHAN0); // enable channel 0, in our case the left side DACs
  } else {
    Wire.write(XPCHAN1); // enable channel 1, in our case the right side DACs
  }
  error = Wire.endTransmission();
  if (error != 0) {
    Serial.print("endTransmission returned ");
    Serial.println(error);
  }
}

void sendValue(byte dac, byte unit, short value) {
  int check = 0;
  Wire.beginTransmission(dac);
  Wire.write((unit & 0x1F)<<3); // unit!  forgot this
  check = Wire.write((value & 0xFF00)>>8); // byte 1
  if (check != 1) {
    Serial.print("failed to write first byte to DAC ");
    Serial.print(dac);
    Serial.print(" unit ");
    Serial.println(unit);
  } else {
    check = Wire.write(value & 0x00FF); // byte 2
    if (check != 1) {
      Serial.print("failed to write second byte to DAC ");
      Serial.print(dac);
      Serial.print(" unit ");
      Serial.println(unit);
    }
  }
  error = Wire.endTransmission();
  if (error != 0) {
    Serial.print("endTransmission returned ");
    Serial.println(error);
  }
  delayMicroseconds(1);
  digitalWriteFast(LATCH, LOW);
  delayMicroseconds(1);
  digitalWriteFast(LATCH, HIGH);
}

// abs(90 - abs( x - 90 )) reflects 0 - 180 to 0-90,90-0 and 0 - (-180) to the same
inline short constrainCos2(short angle) {
  return (abs(90 - abs(abs(angle) - 90)));
}

inline long constrainAngle(long angle) {  // limit angles to -180 to +180
  if (angle > 180) {
      angle -= 180;
  } else if (angle < -180) {
      angle += 180;
  }
  return(angle);
}

// funky modulus appropriate for wrapping at 0, instead of -modulus; good for wrapping around arrays
inline int mod( int x, int y ){
   return x<0 ? ((x+1)%y)+y-1 : x%y;
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


// old cruft I might reference again someday

  //delay(1000);
//  for (int i = 4; i<12; i++) {  // cycle over all LEDs )when < 13
//    selectSide(addr[i][0]);
//    for (value = 1; value < 1024; value ++) {  
//      sendValue(addr[i][1], addr[i][2], (value-1));
//      sendValue(addr[12][1], addr[12][2], (value-1));
//    }
//    for ( ; value > 0; value--) {   // go back down
//      sendValue(addr[i][1], addr[i][2], (value-1));
//      sendValue(addr[12][1], addr[12][2], (value-1));
//    }
//  }
  //while (1) { };


  
