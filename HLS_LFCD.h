/*
 *  HLS_LFCD.h Library for getting data from the lidar with that name
 *  
 *  i had to put all the functions in here, instead of a .cpp file, because of template<> complications (and my lack of skill in Cpp).
 *  i know that header files are not technically supposed to have substance (just headers), but it works for now.
 * 
 *  Created by Thijs van Liempd on 16/12/2020.
 *  imperfect, but potentially functional
 */

#ifndef HLS_LFCD_h
#define HLS_LFCD_h

#include "Arduino.h"

//#define lidarDebugSerial Serial

#define HLS_LFCD_SYNC_BYTE 0xFA        //byte at the start of each 6-deg-packet
#define HLS_LFCD_ANGLE_INDEX_MIN 0xA0  //0*6 deg
//#define HLS_LFCD_ANGLE_INDEX_MAX 0xDB  //59*6 deg

template<class T> class HLS_LFCD
{
  private:
    T& _serialPort;
    
    volatile uint8_t _fillUpBuff = 0; //counter to keep track of how many (serial) bytes of current packet are received (1 means start byte is received, 42 means all bytes received)
    volatile uint8_t _sixDegRawBuf[41]; //a buffer to hold the serial data to be decoded
    volatile bool _sixDegRawBufWriting = false; //for multi-threading/core and interrupt safety, DONT access _sixDegRawBuf[volatileDegree] if _sixDegRawBufWriting is TRUE
    //volatile bool _sixDegRawBufReady = false; //flag to let another core/interrupt know that the data needs to be processed/translated
    const uint32_t _spinUpTimout = 2500; //if it still hasnt sent anything after this time (in millis), then return 0 in the startSpinningAndWait() function
    const uint32_t _spinDownPacketTimeout = 1000; //if no new packets have been received in this time, it's probably no longer spinning
    const uint32_t _runPacketTimeout = 500; //if no new packets have been received in this time, it's probably no longer spinning
    uint32_t _packetTimeoutTimer = 0;
    
    int16_t _parseSixDegBuf(uint8_t bufferProgress) { //(checks checksum and) translates serial data into regular values and stores it in lidarData
      if(bufferProgress >= 2) { //angle-index byte received
        int16_t volatileDegreesAdded = volatileDegree; //to record data increase
        int16_t majorAngle = (_sixDegRawBuf[0] - HLS_LFCD_ANGLE_INDEX_MIN) * 6; //get the angle index and calculate packet angle group
        //if(majorAngle > 354) { Serial.print("majorAngle is TOO HIGH:"); Serial.println(majorAngle); return(-1); }
        if(bufferProgress >= 4) { //RPM bytes received
          RPMraw = (((uint16_t) _sixDegRawBuf[2]) << 8) + _sixDegRawBuf[1]; //combine bytes to make number
        }
        if(bufferProgress >= 10) { //at least one measurement point
          int16_t minorAngle = (constrain(bufferProgress, 10, 40)-4)/6; //divide by 6, but make sure it's an interger divide. e.g.: (10-4)/6=1  ,  (40-4)/6=6
          //volatileDegree is how many degrees of data are already in dataArray
          //(majorAngle + minorAngle) is how many degrees of data are available
          if((majorAngle + minorAngle) > volatileDegree) { //while more data is available than captured
            #ifdef lidarDebugSerial
              if(((majorAngle + minorAngle) > volatileDegree) > minorAngle) { lidarDebugSerial.println("packets lost?"); lidarDebugSerial.println(majorAngle); lidarDebugSerial.println(minorAngle); lidarDebugSerial.println(volatileDegree); return(-1); }
            #else
              if(((majorAngle + minorAngle) > volatileDegree) > minorAngle) { return(-1); }
            #endif
            while(dataArrayWriting) {}; //wait (only needed in specific multicore/interrupt type situations)
            dataArrayWriting = true; //for absolute multi core/threading and interrupt safety
            for(uint8_t i=minorAngle-((majorAngle + minorAngle) - volatileDegree); i<minorAngle; i++) { //start i at (minorAngle - disparity), where 'disparity' is the difference between (majorAngle+minorAngle) and volatileDegrees
              dataArray[volatileDegree][0] = (((uint16_t) _sixDegRawBuf[3+i*6 + 1]) << 8) + _sixDegRawBuf[3+i*6]; //combine bytes to make number
              dataArray[volatileDegree][1] = (((uint16_t) _sixDegRawBuf[3+i*6 + 3]) << 8) + _sixDegRawBuf[3+i*6 + 2]; //combine bytes to make number
              //ignore last 2 bytes of the 6-byte-degree data. Those are 'reserved', according to a 2017 datasheet, which is marked as TENTATIVE
              volatileDegree++;
            }
          }
          if(volatileDegree >= 360) { volatileDegree = 0; rotationCount+=1; volatileDegreesAdded -= 360;} //dedgree rollover ; first rotation flag ; return() math fix
          dataArrayWriting = false; //for absolute multi core/threading and interrupt safety
        }
        if(bufferProgress == 42) { //at least one measurement point
          uint16_t sum = HLS_LFCD_SYNC_BYTE;  for(uint8_t i=0; i<39; i++) { sum += _sixDegRawBuf[i]; } //calculate sum
          #ifdef lidarDebugSerial
            if(_sixDegRawBuf[39] != _sixDegRawBuf[40]) { lidarDebugSerial.print("checksums are different:"); lidarDebugSerial.print(_sixDegRawBuf[39]); lidarDebugSerial.print(' '); lidarDebugSerial.println(_sixDegRawBuf[40]); }
          #endif
          ChecksumResult[constrain(majorAngle/6, 0, 59)] = (_sixDegRawBuf[39] == (0xFF - (0xFF & sum))); //if checksum byte == 0xFF - (lower byte of sum)
          //if(checksum checks out...)
          _packetTimeoutTimer = millis();
        }
        //if(postParseCallback) { postParseCallback(this, volatileDegree - volatileDegreesAdded); }
        return(volatileDegree - volatileDegreesAdded); //return data entries (degrees) gained this time around
      } else {
        return(0); //return data entries (degrees) gained this time around
      }
    } 
    
  public:
    volatile bool spinning = false; //set to true once the first sync byte comes through
    volatile uint32_t rotationCount = 0; //mostly used as an indicator that the first rotation is done, but also usefull to know when a new set of datapoints is available
    
    volatile uint16_t RPMraw = 0; //(NOTE: one-decimal float as int, so 3001==300.1) pretty self explanatory, Rotations Per Minute. target is 300RPM, according to 'datasheet'
    
    volatile uint16_t dataArray[360][2]; //360 degrees, each degree has 2 16-bit values
    volatile uint16_t volatileDegree = 0; //the degree(0 to 359) (dataArray index) that will be written to next
    volatile bool dataArrayWriting = false; //for multi-threading/core and interrupt safety, DONT access dataArray[volatileDegree] if dataArrayWriting is TRUE

    volatile bool ChecksumResult[60]; // 360/6=60 packets, 

    void (*postParseCallback)(HLS_LFCD* self, int16_t dataPointsAdded) = NULL;
    
    HLS_LFCD(T& serialPort) : _serialPort(serialPort) {} //init function, pass serial port
    ~HLS_LFCD() { stopSpinning(); } //maybe
    
    #if defined (ESP32) //i'd recommend using Serial1. (the ESP32 has 3 HW serials and by defualt, Serial1 uses pins 9 and 10, which are internal flash pins)
      void begin(uint8_t rx=16, uint8_t tx=17) {
        pinMode(rx, INPUT_PULLUP);
        pinMode(tx, OUTPUT);
        _serialPort.begin(230400, SERIAL_8N1, rx, tx);
      }
    #else
      void begin() {
        _serialPort.begin(115200);
        _initLidarDataStructure();
      }
    #endif
    
    void startSpinning() {
      _serialPort.write('b'); //0x62 
    }
    
    bool startSpinningAndWait(bool allowCallback=false) {
      startSpinning();
      uint32_t spinUpTimer = millis();
      while((!spinning || !rotationCount) && ((millis()-spinUpTimer)<_spinUpTimout)) { run(allowCallback); }
      #ifdef lidarDebugSerial
        lidarDebugSerial.print("time untill spin-up: "); lidarDebugSerial.println(millis()-spinUpTimer);
        if(!(spinning && rotationCount)) { lidarDebugSerial.print("timeout occured, didnt start?  spinning:"); lidarDebugSerial.print(spinning); lidarDebugSerial.print(" rotationCount:"); lidarDebugSerial.println(rotationCount); }
      #endif
      return(spinning && rotationCount); //if it timed out, this will be 0, otherwise it's 1
    }
    
    void stopSpinning() {
      _serialPort.write('e'); //0x65
    }

    bool stopSpinningAndWait(bool allowCallback=false) {
      stopSpinning();
      uint32_t spinDownTimer = millis();
      while(spinning && ((millis()-spinDownTimer)<_spinDownPacketTimeout)) { run(allowCallback); }
      #ifdef lidarDebugSerial
        lidarDebugSerial.print("time untill spin-down: "); lidarDebugSerial.println(millis()-spinDownTimer);
        if(spinning)) { lidarDebugSerial.print("timeout occured, didnt stop?  spinning:"); lidarDebugSerial.println(spinning); }
      #endif
      return(!spinning); //if it timed out, this will be 0, otherwise it's 1
    }
    
    int8_t run(bool allowCallback=true) {
      bool didSomething=false;
      while(_serialPort.available()) {
        while(_sixDegRawBufWriting) {}; //wait (only needed in specific multicore/interrupt type situations)
        _sixDegRawBufWriting = true;
        if(_fillUpBuff) { //if it is not 0
          _sixDegRawBuf[_fillUpBuff-1] = _serialPort.read();
          _fillUpBuff++;
          if(_fillUpBuff == 42) {
            //_sixDegRawBufReady = true; //flag to let another core/interrupt know that the data needs to be processed/translated
            int16_t pointsAdded = _parseSixDegBuf(_fillUpBuff); //note, function can also be called intermittendly.
            _fillUpBuff = 0;
            if((postParseCallback) && allowCallback) { postParseCallback(this, pointsAdded); }
          }
          didSomething=true;
        } else {
          if(_serialPort.read() == HLS_LFCD_SYNC_BYTE) { //sync byte, start of 6-deg-packet
            spinning = true;
            _fillUpBuff = 1; //sync byte received, start collecting the remaining 41 bytes
            didSomething=true;
          }
        }
        _sixDegRawBufWriting = false;
      }
      if(!didSomething) { if((millis() - _packetTimeoutTimer) > _runPacketTimeout) { spinning = false; } }
      return(_fillUpBuff);
    }
    
    float RPM() {
      return(RPMraw/10.0);
    }
};

#endif