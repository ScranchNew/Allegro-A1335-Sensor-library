/***************************************************
  Arduino library for the Allegro A1335 Magnetic angle sensor
  The A1335 sensor detects the absolute angular position of a 
  permanent magnet, typically a diametrically
  magnetized cylinder on the rotating shaft.

  * by Florian von Bertrab
 ****************************************************/

#ifndef A1335_H
#define A1335_H

#if (ARDUINO >= 100)
     #include "Arduino.h"
#else
     #include "WProgram.h"
#endif
#include <Wire.h>

class bytes_2 { // allows easy conversion from two bytes in a given order to a 16 bit int.
public:
  bytes_2();
  bytes_2(int16_t integ);

  int16_t&   in();
  byte&      lsBy(byte n);
  byte&      msBy(byte n);

private:
  union     {
    int16_t integer;
    byte    bytes[2];
  }data;
};

class bytes_4 { // allows easy conversion from four bytes in a given order to a 32 bit int.

public:
  bytes_4();
  bytes_4(int32_t integ);

  int32_t&   in();
  byte&      lsBy(byte n);
  byte&      msBy(byte n);

private:
  union     {
    int32_t integer;
    byte    bytes[4];
  }data;
};

class A1335 {
public:
  A1335();  

  byte      start(int16_t address_);	// starts the sensor at the given address
  
  int16_t   getAddress();			// returns I2C address
  byte      getProcessorState();		// returns processor state:
							// 0 = booting; 1 = idle; 2 = running; 3 = self-test mode; 4 = not found

  byte      getOutputRate();		// returns the log2() of the samperate. E.g. 3 would mean 8 samples per data point.

  
  double    readAngle();			// returns the angle in degrees

  uint16_t  readAngleRaw();			// returns raw angle data (4096 = 360°)

  double    readTemp();			// returns temperature in Kelvin

  uint16_t  readTempRaw();			// returns raw temperature data 8 = 1 K

  double    readField();			// returns field strenght in Tesla

  uint16_t  readFieldRaw();			// returns raw field strenght data 10 = 1mT


  byte      readOutputRate();		// reads the log2() of the sample rate. Does not really work yet!

  byte      setOutputRate(byte rate);// sets the log2() of the sample rate => (ORate = 2^rate). Does not really work yet!
  
  byte      normalWrite(byte reg, int16_t data); // writes 16 bit to a given register

  int16_t   normalRead(byte reg);			 // reads 16 bit from a given register

  
  byte      extendedWrite(int16_t reg, int32_t data); // writes 32 bit to a given extended register

  int32_t   extendedRead(int16_t reg);			 // reads 32 bit from a given extended register


private:
  int16_t   address = 0x0C;             // I2C address
  byte      processorState = 4;         // 0 = booting; 1 = idle; 2 = running; 3 = self-test mode; 4 = not found
  byte      outputRate = 0;                 // log2() of the sample rate in the EEPROM
};

#endif //A1335_H
