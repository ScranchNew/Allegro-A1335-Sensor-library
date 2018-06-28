/***************************************************
  Arduino library for the Allegro A1335 Magnetic angle sensor
  The A1335 sensor detects the absolute angular position of a 
  permanent magnet, typically a diametrically
  magnetized cylinder on the rotating shaft.

  * by Florian von Bertrab
 ****************************************************/

#include "A1335.h"


bytes_2::bytes_2(){
  data.integer = 0;
}

bytes_2::bytes_2(int16_t integ){
  data.integer = integ;
}

int16_t&   bytes_2::in(){
  return data.integer;
}

byte&      bytes_2::lsBy(byte n){
  if (n > 1) {
    n = 1;
  } else if (n < 0) {
    n = 0;
  }
  return data.bytes[n];
}

byte&      bytes_2::msBy(byte n){
  if (n > 1) {
    n = 1;
  } else if (n < 0) {
    n = 0;
  }
  return data.bytes[1-n];
}

bytes_4::bytes_4(){
  data.integer = 0;
}

bytes_4::bytes_4(int32_t integ){
  data.integer = integ;
}

int32_t&   bytes_4::in(){
  return data.integer;
}

byte&      bytes_4::lsBy(byte n){
  if (n > 3) {
    n = 3;
  } else if (n < 0) {
    n = 0;
  }
  return data.bytes[n];
}

byte&      bytes_4::msBy(byte n){
  if (n > 3) {
    n = 3;
  } else if (n < 0) {
    n = 0;
  }
  return data.bytes[3-n];
}

//--- Normal Write Registers ---//

const byte EWA    = 0x02;   // Extended Write Address
const byte EWD    = 0x04;   // Extended Write Data
const byte EWCS   = 0x08;   // Extended Write Control and Status
const byte ERA    = 0x0A;   // Extended Read Address
const byte ERCS   = 0x0C;   // Extended Read Control and Status
const byte ERD    = 0x0E;   // Extended Read Data
const byte CTRL   = 0x1E;   // Device control
const byte ANG    = 0x20;   // Current angle and related data
const byte STA    = 0x22;   // Device status
const byte ERR    = 0x24;   // Device error status
const byte XERR   = 0x26;   // Extended error status
const byte TSEN   = 0x28;   // Temperature sensor data
const byte FIELD  = 0x2A;   // Magnetic field strength
const byte ERM    = 0x34;   // Device error status masking
const byte XERM   = 0x36;   // Extended error status masking

//--- Extended Write Registers ---//

const int16_t ORATE = 0xFFD0;  // Output Rate


// Control bytes and their respective keycodes:

//                      CTRL(0x1E) KEY(0x1F)
//  Processor State
const byte ipm[]    = { B10000000, 0x46     }; // Idle mode
const byte rpm[]    = { B11000000, 0x46     }; // Run  mode
//  Hard Reset
const byte hre[]    = { B00100000, 0xB9     }; // Hard reset
//  Soft Reset
const byte sre[]    = { B00010000, 0xB9     }; // Soft reset
//  Clear STA
const byte csta[]   = { B00000100, 0x46     }; // Clear (STA)  registers
//  Hard Reset
const byte cxerr[]  = { B00000010, 0x46     }; // Clear (XERR) registers
//  Hard Reset
const byte cerr[]   = { B00000001, 0x46     }; // Clear (ERR)  registers
 

// Angle Register read masks

//                      ANG(0x20)  ANG+1(0x21)
// Register Identifier Code
const byte ria[]    = { B10000000, B00000000}; // Always 0
// Error Flag
const byte efa[]    = { B01000000, B00000000}; // At least one error in register 0x24
// New Flag
const byte nfa[]    = { B00100000, B00000000}; // A new angle is in the angle register
// Parity
const byte par[]    = { B00010000, B00000000}; // Odd parity bit for the whole register
// Angle
const byte ang[]    = { B00001111, B11111111}; // Encoded angle reading (n * 360/4096 = angle in deg.)


// Status Register read masks

//                      STA(0x20)  STA+1(0x21)
// Register Identifier Code
const byte ris[]    = { B11110000, B00000000}; // Always 1000
// Power-On Reset Flag
const byte por[]    = { B00001000, B00000000}; // There was a power-on reset since last field reset
// Soft Reset Flag
const byte srf[]    = { B00000100, B00000000}; // There was a soft reset since last field reset
// New Flag
const byte nfs[]    = { B00000010, B00000000}; // A new angle is in the angle register
// Current Error Flag
const byte efs[]    = { B00000001, B00000000}; // At least one error in register 0x24
// Processor Processing Status
const byte mps[]    = { B00000000, B11110000}; // 0000 Booting; 0001 Idle or Processing angles; 1110 Self-testmode
// Processor Phase Status
const byte phase[]  = { B00000000, B00001111}; // 0000 Idle; 0001 Processing angles; Only in Self-test mode [0100 Built in self test; 0110 ROM checksum; 0111 CVH self test]


// Temperature Register read masks

//                      TSEN(0x28) TSEN+1(0x29)
// Register Identifier Code
const byte rit[]    = { B11110000, B00000000}; // Always 1111
// Temperature
const byte temp[]   = { B00001111, B11111111}; // Encoded temperature reading (n / 8 = temperature in K)


// Field Strenght Register read masks

//                      FIELD(0x28) FIELD+1(0x29)
// Register Identifier Code
const byte rif[]    = { B11110000, B00000000}; // Always 1110
// Magnetic field strenght
const byte field[]  = { B00001111, B11111111}; // Magnetic field strenght reading (n = field strenght in Gauss (1/10000 T))


A1335::A1335(){
  
}

int16_t   A1335::getAddress(){                  // returns I2C address of the sensor
  return address;
}

byte      A1335::getProcessorState(){           // returns current processor state of the sensor
  return processorState;
}

byte      A1335::getOutputRate(){               // returns the current sample rate of the sensor
  return outputRate;
}

byte      A1335::start(int16_t address_){        // Initializes the sensor at the address given and fills the private variables
  Wire.beginTransmission(address);
  byte error = Wire.endTransmission();
  if (error) {
    processorState = 4;
    return error;
  }
  address = address_;
  bytes_2 state(normalRead(STA));
  bytes_4 orate(extendedRead(ORATE));
  
  byte processing_status = (state.msBy(1) & mps[1]) >> 4;
  byte processing_phase =  (state.msBy(1) & phase[1]) >> 0;
  switch (processing_status) {
    case B00000000:
      processorState = 0;
      break;
    case B00000001:
      if (processing_phase == 0){
        processorState = 1;
      } else {
        processorState = 2;
      }
      break;
    case B00001110:
      processorState = 3;
      break;
  }
  outputRate = orate.msBy(3);
  delay(1);
  return error;
}

double    A1335::readAngle(){       // returns Angle in Degrees
  return double(readAngleRaw()) * 360.0 / 4096.0;
}
uint16_t  A1335::readAngleRaw(){    // returns raw angle data
  bytes_2 angReg(normalRead(ANG));
  
  int16_t parityCheck = angReg.in();
  parityCheck ^= parityCheck >> 8;
  parityCheck ^= parityCheck >> 4;
  parityCheck ^= parityCheck >> 2;
  parityCheck ^= parityCheck >> 1;
  parityCheck &= 1;                 // parity Check now contains the parity of all bits in angReg
  if(!parityCheck) {                // odd Parity in this register => a 0 means an error
    return 0;
  }
  
  angReg.msBy(0) &= ang[0];           // mutes bits, that dont contain the angle data
  return angReg.in();
}

double    A1335::readTemp(){        // returns temperature in Kelvin
  return double(readTempRaw())/8.0;
}

uint16_t  A1335::readTempRaw(){     // returns raw temperature data
  bytes_2 tempReg(normalRead(TSEN));
  tempReg.msBy(0) &= temp[0];         // mutes bits, that dont contain temperature data
  return tempReg.in();
}

double    A1335::readField(){       // returns field strenght in Tesla
  return double(readFieldRaw())/10000.0;
}

uint16_t  A1335::readFieldRaw(){    // returns raw field strenght data
  bytes_2 fieldReg(normalRead(FIELD));
  fieldReg.msBy(0) &= field[0];       // mutes bits, that dont contain temperature data
  return fieldReg.in();
}

byte      A1335::readOutputRate(){  // reads the log2() of the sample rate 
  bytes_4 oRate(extendedRead(ORATE));
  return oRate.msBy(3);             // !!!  I don't know yet, which byte holds the output rate  !!!
}

byte      A1335::setOutputRate(byte rate){  // sets the log2() of the sample rate => (ORate = 2^rate)
  if (rate < 0) {
    rate = 0;
  } else if (rate >=8) {
    rate = 7;
  }
  bytes_2 idle_mode;
  bytes_2 run_mode;
  for (int i = 0; i < 2; i++){
    idle_mode.msBy(i) = ipm[i];
    run_mode.msBy(i)  = rpm[i];
  }
  normalWrite(CTRL, idle_mode.in());
  delayMicroseconds(150);
  bytes_4 oRate;
  oRate.msBy(3) = rate;
  extendedWrite(ORATE, oRate.in());       // !!!  I don't know yet, which byte gets the output rate  !!!
  delayMicroseconds(50);
  normalWrite(CTRL, run_mode.in());
  delayMicroseconds(150);
}

byte      A1335::normalWrite(byte reg, int16_t data){      // writes the 2 bytes in "bytes" to the register with address reg to the sensor with I2C address adress.
  bytes_2 data_bytes(data);
  Wire.beginTransmission(address);
  Wire.write(reg);                                        // choose target register
  for (int i = 0; i<2; i++){
    Wire.write(data_bytes.msBy(i));                       // Writes data MSB first
  }
  return Wire.endTransmission();
}

byte      A1335::extendedWrite(int16_t reg, int32_t data){ // writes the 4 bytes in "bytes" to the extended register with address reg to the sensor with I2C address adress.
  bytes_4 data_bytes(data);
  Wire.beginTransmission(address);
  Wire.write(EWA);                                        // choose write Register
  Wire.write(byte(reg >> 8));                             // Fill with target address
  Wire.write(byte(reg));
  for (int i = 0; i<4; i++){
    Wire.write(data_bytes.msBy(i));                       // Writes data MSB first
  }
  Wire.write(0x80);                                       // Confirm write
  Wire.endTransmission();
  delayMicroseconds(10);
  Wire.requestFrom(address, 1);
  return Wire.read();                                     // Returns 1 if it works
}

int16_t   A1335::normalRead(byte reg){
  bytes_2 data;
  Wire.beginTransmission(address);
  Wire.write(reg);                // choose target Register
  Wire.endTransmission();
  Wire.requestFrom(address, 2);
  for(int i=0; i< 2; i++){
    if (Wire.available()){
      data.msBy(i) = Wire.read();   // read data bytes
    }
  }
  return data.in();
}

int32_t   A1335::extendedRead(int16_t reg){
  bytes_4 data;
  byte rstate;
  Wire.beginTransmission(address);
  Wire.write(ERA);                // choose write Register
  Wire.write(byte(reg >> 8));     // Fill with target address
  Wire.write(byte(reg));
  Wire.write(0x80);               // Confirm read
  Wire.endTransmission();
  delayMicroseconds(10);
  Wire.requestFrom(address,5);
  rstate = Wire.read();           // Reads status byte
  for(int i=0; i < 4; i++){
    if (Wire.available()){
      data.msBy(i) = Wire.read();    // Reads data bytes
    }
  }
  return data.in();
}


