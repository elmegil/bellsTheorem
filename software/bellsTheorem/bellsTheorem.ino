

#include <Wire.h>
#include <Encoder.h>

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


long long intermediate;
const unsigned short maximumOutput = 0x3FF; //1023
unsigned short inputValue = maximumOutput;
byte angleA, angleB, angleC, angleD; //
const int oneLens = 1000;
const int twoLens = 1000000;
const int threeLens = 1000000000;

/*
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

short value[13] = { maximumOutput, maximumOutput, maximumOutput, maximumOutput,
                    maximumOutput, maximumOutput, maximumOutput, maximumOutput,
                    maximumOutput, maximumOutput, maximumOutput, maximumOutput,
                    maximumOutput };

long angle[4] = { 0, 0, 0, 0 };  // absolute angle of each encoder
long relativeAngle[4] = { 0, 0, 0, 0 }; // relative angle of each pair A->B, B->C, C->D, D-A[]


// for testing convenience, probably won't use these "for real"
#define OUTA 0
#define OUTB 1
#define OUTC 2
#define OUTD 3
#define OUTAB 4
#define OUTBC 5
#define OUTCD 6
#define OUTDA 7
#define OUTABC 8
#define OUTBCD 9
#define OUTCDA 10
#define OUTDAB 11
#define OUTABCD 12
#define LEFTSIDE 0
#define RIGHTSIDE 1

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


// stuff for bell's Theorem testing
// fixed point, 3 digit representation of cosine squared, for integer angles 0 - 90
unsigned short cos2Table[] = {
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

Encoder encA(ENCAB, ENCAA);
Encoder encB(ENCBB, ENCBA);
Encoder encC(ENCCB, ENCCA);
Encoder encD(ENCDB, ENCDA);

Encoder *encoders[] = { &encA, &encB, &encC, &encD };

//
// intermediate = inputValue * cos2Table[0] * cos2Table[0] * cos2Table[0]
//
// final = (short)(intermediate / threeLens)



void setup() {  
  pinMode(LATCH, OUTPUT);
  digitalWrite(LATCH, HIGH); // latch occurs on LTH transition
  
  Serial.begin(115200);
  Serial.println(SDA);
  Serial.println(SCL);

  Wire.begin();
  Wire.setClock(400000); // try to use Fast mode.... works, but doesn't seem to make the test loop any faster
  Wire.requestFrom(XPADDR,1);
  while (Wire.available()) {
    xpstatus = Wire.read();
  }  
  Serial.println(xpstatus); // should be all zeroes at this point
  for (int i = 0; i<13; i++) {  // max everyone out to start because all angles are 0
    selectSide(addr[i][0]);
    sendValue(addr[i][1], addr[i][2], maximumOutput);
  }

  selectSide(LEFTSIDE);     // set as default state
}


void loop() {  
  long tempEnc;
  short relAngle;

  for (int i = 0; i < 4; i++) {
    tempEnc = encoders[i]->read();
    if (tempEnc != angle[i]) { // change!
      Serial.println("=========================================");
      Serial.print("change! ");
      Serial.println(i);
      angle[i] = tempEnc;
      if (angle[i] > 180) {
        angle[i] -= 180;
        encoders[i]->write(angle[i]);
      } else if (angle[i] < -180) {
        angle[i] += 180;
        encoders[i]->write(angle[i]);
      }
      Serial.print("angle ");
//      Serial.print(i);
//      Serial.print(" is ");
//      Serial.println(angle[i]);
      for (int x=0; x<4; x++) { Serial.print(angle[x]); Serial.print(" "); } Serial.println();
      
      // ok angle is set, now check the relationship to adjacent lenses
      relAngle = angle[mod(i+1,4)] - angle[i]; // maximum difference is 360
      if (relAngle > 180) {
        relAngle -= 180;
      } else if (relAngle < -180) {
        relAngle += 180;
      }
      relativeAngle[i] = relAngle;
//      Serial.print("forward relative angle ");
//      Serial.print(i);
//      Serial.print(" is ");
//      Serial.println(relAngle);
      relAngle = angle[i] - angle[mod(i-1,4)];
      if (relAngle > 180) {
        relAngle -= 180;
      } else if (relAngle < -180) {
        relAngle += 180;
      }
      relativeAngle[mod(i-1,4)] = relAngle;
//      Serial.print("backward relative angle ");
//      Serial.print(mod(i-1,4));
//      Serial.print(" is ");
//      Serial.println(relAngle);
      Serial.print("relativeAngle ");
      for (int x=0; x<4; x++) { Serial.print(relativeAngle[x]); Serial.print(" "); } Serial.println();

      intermediate = inputValue * cos2Table[abs(90 - abs(abs(angle[i]) - 90))]; // abs(90 - abs( x - 90 )) reflects 0 - 180 to 0-90,90-0
      // ultimately this will actually just be A/B/C/D no value + 4
      value[i+4] = (short)(intermediate / oneLens);                          // and 0 - (-180) to the same
      selectSide(addr[i+4][0]);
      sendValue(addr[i+4][1], addr[i+4][2], value[i+4]);
//      Serial.print("single lens value ");
//      Serial.print(i);
//      Serial.print(" is ");
//      Serial.println(value[i+4]);
      
      intermediate = value[i+4] * cos2Table[abs(90 - abs(abs(relativeAngle[i]) - 90))];
      value[i+8] = (short)(intermediate / oneLens);
      selectSide(addr[i+8][0]);
      sendValue(addr[i+8][1], addr[i+8][2], value[i+8]);
//      Serial.print("forward double lens value ");
//      Serial.print(i);
//      Serial.print(" is ");
//      Serial.println(value[i+8]);

      intermediate = value[i+4] * cos2Table[abs(90 - abs(abs(relativeAngle[mod(i-1,4)]) - 90))];
      value[mod(i-1,4)+8] = (short)(intermediate / oneLens);
      selectSide(addr[mod(i-1,4)+8][0]);
      sendValue(addr[mod(i-1,4)+8][1], addr[mod(i-1,4)+8][2], value[mod(i-1,4)+8]);
//      Serial.print("backward double lens value ");
//      Serial.print(mod(i-1,4)+8);
//      Serial.print(" is ");
//      Serial.println(value[mod(i-1,4)+8]);
      Serial.print("value ");
      for (int x=0; x<13; x++) { Serial.print(value[x]); Serial.print(" "); } Serial.println();
    }
  }
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

// funky modulus appropriate for wrapping at 0, instead of -modulus; good for wrapping around arrays
int mod( int x, int y ){
   return x<0 ? ((x+1)%y)+y-1 : x%y;
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
