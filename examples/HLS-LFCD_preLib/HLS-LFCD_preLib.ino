/*
TODO:
- serial timeout to detect if still spinning (and stopSpinningAndWait())
- serial timeout to detect missed bytes?
- IRAM (and DRAM) attributes?

- tune stop-spinning timout

commands: (not in library, just loop())
'b' (begin) start lidar
'e' (end) stop lidar

'c' (calibrate/compare) save current lidar data to comparative array (in RAM)
'k' (keep) save current comparative array data to flash(/'EEPROM')          TODO
'l' (load) retrieve comparative array data from flash(/'EEPROM')            TODO

*/

//#include "HLS_LFCD.h"

//#define lidarDebugSerial Serial

//#define DACdisplay
#define ILI9341displayAdafruit
//#define ILI9341displayTFT_eSPI

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



HLS_LFCD<HardwareSerial> lidar(Serial2);

uint16_t compareData[360];
bool compareDataSet = false;
uint16_t compareErrorThreshold = 300;
#if defined(ILI9341displayAdafruit) || defined(ILI9341displayTFT_eSPI)
  int16_t compareDataPointsCalculated = 0;
#endif

//if you want to immidietly process data after parsing a serial packet, set a callback function
void newDataCallback(HLS_LFCD<HardwareSerial>* lidarPointer, int16_t dataPointsAdded) {
  //log_v("callback at %d", lidarPointer->volatileDegree); (JUST FOR DEBUGGING IF THE CB FUNC WORKS)
  //log_v("dataPointsAdded: %d", dataPointsAdded);
  //process data here (try to keep ik quick though, as this is all running in 'lidar.run()')
  int16_t newDataIndex = lidarPointer->volatileDegree; //note: volatileDegree is unsigned
  newDataIndex -= dataPointsAdded;  if(newDataIndex < 0) { newDataIndex += 360; }
  if(compareDataSet) {
    for(uint8_t i=0; i<dataPointsAdded; i++) {
      if((newDataIndex+i) > 359) { newDataIndex -= 360; } //super duper safety (should never happen)
      if(compareData[newDataIndex+i] > lidarPointer->dataArray[newDataIndex+i][1]) { //if the measurement is FURTHER than the calibrated point (weird, assuming calibrated datapoints are walls)
        if((compareData[newDataIndex+i] - lidarPointer->dataArray[newDataIndex+i][1]) >= compareErrorThreshold) {
          Serial.print(newDataIndex+i); Serial.print(" -"); Serial.println(compareData[newDataIndex+i] - lidarPointer->dataArray[newDataIndex+i][1]); //report negative measurement error
        }
      } else { //(normal)
        if((lidarPointer->dataArray[newDataIndex+i][1] - compareData[newDataIndex+i]) >= compareErrorThreshold) {
          Serial.print(newDataIndex+i); Serial.write(' '); Serial.println(lidarPointer->dataArray[newDataIndex+i][1] - compareData[newDataIndex+i]); //report positive measurement error
        }
      }
    }
  }
}

void storeCompareData() { //TODO: lidar pointer input argument (universal, not just HWserial??)
  lidar.dataArrayWriting = true;
  for(uint16_t i=0; i<360; i++) { compareData[i] = lidar.dataArray[i][1]; } 
  #if defined(ILI9341displayAdafruit) || defined(ILI9341displayTFT_eSPI)
    compareDataPointsCalculated = 0;
  #endif
  compareDataSet=true;  //Serial.println("compareData set");
  lidar.dataArrayWriting = false;
  Serial.print("calib-start: "); Serial.println(compareErrorThreshold);
  uint32_t checkSum = 0;
  for(uint16_t i=0; i<360; i++) { Serial.print(i); Serial.write(' '); Serial.println(compareData[i]); checkSum += compareData[i]; lidar.run(false); } 
  Serial.print("calib-end: "); Serial.println(checkSum);
}

unsigned long printTimer;
int8_t fewMorePrints = 5;

#if defined(ILI9341displayAdafruit) || defined(ILI9341displayTFT_eSPI) //shared TFT values
  //#define ZOOMVAR 0.1
  #define UNZOOMVAR 30.0
  #define TFT_ROTATION 3

  //#define tft_frame_wait

  #define drawPointCircles 1 //draw circles on the datapoints (as well as lines) with the defined radius
  
  #define SCREENWIDTH 320
  #define SCREENHEIGHT 240
  #define BG_COLOR 0xAD75 //off-white
  #define DATA_POINT_COLOR 0xF800 //red  (only used if drawPointCircles is defined)
  #define DATA_LINE_COLOR 0xFFE0 //yellow
  #define COMPAREDATA_COLOR 0x07E0 //green
  #define CENTER_COLOR 0x001F //blue
  #define TFT_CS   14  // Chip select control pin
  #define TFT_DC   13  // Data Command control pin
  #define SPI_PORT_TO_USE VSPI // can be mapped to any pins, but for max (80MHz) speed, needs default pins: 18,19,23 for VSPI, 14,12,13 for HSPI
  
  //const int16_t screenMid[2] = {SCREENWIDTH/2, SCREENHEIGHT/2};
  const int16_t screenMid[2] = {SCREENHEIGHT/2, SCREENHEIGHT/2};
  uint16_t lastPoint[3]; //holds (pixel) pos of last point to draw line from (and the index of that point)
  uint16_t lastComparePoint[3]; //holds (pixel) pos of last point to draw line from (and the index of that point)

  uint16_t compareDataPoints[360][2]; //dont want to keep recalculating this, that'd be stupid

  #define distAngleToPos(angle, dist, output)  output[0]=cos(angle)*(float)dist; output[1]=sin(angle)*(float)dist
  
  #ifdef UNZOOMVAR
    #define convertToPixelPoint(floatPos, output) output[0]=(uint16_t)(screenMid[0] + ((int16_t)(floatPos[0]/UNZOOMVAR))); output[1]=(uint16_t)(screenMid[1] + ((int16_t)(floatPos[1]/UNZOOMVAR)))
    #ifdef ZOOMVAR
      #warning("found both ZOOMVAR and UNZOOMVAR, using only UNZOOMVAR.")
    #endif
  #elif defined(ZOOMVAR)
    #define convertToPixelPoint(floatPos, output) output[0]=(uint16_t)(screenMid[0] + ((int16_t)(floatPos[0]*ZOOMVAR))); output[1]=(uint16_t)(screenMid[1] + ((int16_t)(floatPos[1]*ZOOMVAR)))
  #else //no zoomvar set, attempt to display whole range
    #error("TBD: determine universal UNZOOMVAR")
    #define convertToPixelPoint(floatPos, output) output[0]=(uint16_t)(screenMid[0] + ((int16_t)floatPos[0])); output[1]=(uint16_t)(screenMid[1] + ((int16_t)floatPos[1]))
  #endif

  #define degRoll(deg) ((deg) < 0 ? (deg+360) : (deg%360))
#endif

#ifdef ILI9341displayAdafruit
  #include <SPI.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_ILI9341.h>
  //SPIClass SPIport = SPIClass(SPI_PORT_TO_USE); //to be implemented, doesnt quite work right now
  //Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPIport, TFT_CS, TFT_DC);
  ////Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPIport, TFT_CS, TFT_DC, TFT_RST);
  Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC); //using V_SPI, pins 18,19,23
  //Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST); //using V_SPI, pins 18,19,23

  void drawLidarDataOnILI9341(void *arg) {
    disableCore0WDT(); //screw that watchdog timer
    HLS_LFCD<HardwareSerial>* lidarPointer = (HLS_LFCD<HardwareSerial>*) arg; //retrieve lidar pointer from the passed argument
    
    tft.begin(40000000);
    #ifdef TFT_ROTATION
      tft.setRotation(TFT_ROTATION);
    #endif
    tft.fillScreen(BG_COLOR);
    log_v("getFreeHeap before displaybuffer: %d", ESP.getFreeHeap());
    GFXcanvas16 displayBuffer = GFXcanvas16(SCREENHEIGHT, SCREENHEIGHT); //due to ESP32's max single allocation and 240*320*2 being larger than that, we can just do this
    log_v("getFreeHeap after displaybuffer: %d", ESP.getFreeHeap());
    //setup complete, loop from here
    uint32_t speedMes[3];
    while(1) {
      speedMes[0] = micros();
      displayBuffer.fillScreen(BG_COLOR);
      displayBuffer.fillCircle(screenMid[0], screenMid[1], 2, CENTER_COLOR);
      for(int16_t i=0; i<360; i++) {
        while(lidarPointer->dataArrayWriting) {} //wait
        int32_t dataPoint = lidarPointer->dataArray[i][1]; //only retrieve distance, other parameter ([0]) is wack
        if(compareDataSet && ((abs(((int32_t)compareData[i])-dataPoint) < compareErrorThreshold) || (dataPoint == 0) || (compareDataPointsCalculated < 360))) {
          if(compareData[i] > 0) {
            if(compareDataPointsCalculated < 360) {
              float floatPos[2];  distAngleToPos(radians(i), compareData[i], floatPos); //calculate position given angle and distance (float floatPos[2])
              convertToPixelPoint(floatPos, compareDataPoints[i]); //convert to pixel using (UN)ZOOMVAR
              compareDataPointsCalculated++; //count up to 360
            }
            displayBuffer.drawLine(lastComparePoint[0], lastComparePoint[1], compareDataPoints[i][0], compareDataPoints[i][1], COMPAREDATA_COLOR); //just ignore the zero-eth line
            lastComparePoint[0] = compareDataPoints[i][0];  lastComparePoint[1] = compareDataPoints[i][1];  lastComparePoint[2] = i; //store last point
            #ifdef drawPointCircles
              displayBuffer.fillCircle(compareDataPoints[i][0], compareDataPoints[i][1], drawPointCircles, COMPAREDATA_COLOR);
            #endif
          } else { if(compareDataPointsCalculated < 360) { compareDataPointsCalculated++; } }
        } else if(dataPoint > 0) {
          float angle=radians(i);
          float floatPos[2];  distAngleToPos(angle, dataPoint, floatPos); //calculate position given angle and distance (float floatPos[2])
          uint16_t pixelPoint[2];  convertToPixelPoint(floatPos, pixelPoint); //convert to pixel using (UN)ZOOMVAR
          if(compareDataSet) { //this if-statement could be formatted as 1 neat (readable) one, but this is more computationally conservative
            if(lastPoint[2] == degRoll(i-1)) { displayBuffer.drawLine(lastPoint[0], lastPoint[1], pixelPoint[0], pixelPoint[1], DATA_LINE_COLOR); }
          } else {
            displayBuffer.drawLine(lastPoint[0], lastPoint[1], pixelPoint[0], pixelPoint[1], DATA_LINE_COLOR);
          }
          lastPoint[0] = pixelPoint[0];  lastPoint[1] = pixelPoint[1];  lastPoint[2] = i; //store last point
          #ifdef drawPointCircles
            displayBuffer.fillCircle(pixelPoint[0], pixelPoint[1], drawPointCircles, DATA_POINT_COLOR);
          #endif
        }
      }
      speedMes[1] = micros();
      //tft.drawRGBBitmap(0, 0, displayBuffer.getBuffer(), SCREENWIDTH, SCREENHEIGHT);
      tft.drawRGBBitmap((SCREENWIDTH-SCREENHEIGHT)/2, 0, displayBuffer.getBuffer(), SCREENHEIGHT, SCREENHEIGHT);
      speedMes[2] = micros();
      //Serial.print(speedMes[1]-speedMes[0]); Serial.write(' '); Serial.println(speedMes[2]-speedMes[1]); //"11483 29108" makes ~24 fps, and since the lidar is 5rps, it's 4-5 frames per measurement
      #ifdef tft_frame_wait
        uint32_t threeSixtyPeriod=200000; if(lidarPointer->RPMraw > 0) { threeSixtyPeriod=600000000L/lidarPointer->RPMraw; }
        if((micros()-speedMes[0]) < threeSixtyPeriod) { //if drawing the frame took less time than 1 rotation (it should)
          uint32_t oldRotationCount = lidarPointer->rotationCount;
          while(oldRotationCount == lidarPointer->rotationCount) { //while waiting for the 
            delay(1); 
            while(lidarPointer->dataArrayWriting) {} //wait
          }
        }
      #endif
    }
  }
  
#elif defined(ILI9341displayTFT_eSPI)
  //this library has cool DMA features, but it's pinout is pretty shit;
  //you have to edit the User_Setup.h file in the library folder to change pinout
  #include <TFT_eSPI.h>
  TFT_eSPI tft = TFT_eSPI();       // Invoke custom library

  TFT_eSprite displayBuffer = TFT_eSprite(&tft);
  uint16_t* displayBufferPointer;
  
  void drawLidarDataOnILI9341(void *arg) {
    disableCore0WDT(); //screw that watchdog timer
    HLS_LFCD<HardwareSerial>* lidarPointer = (HLS_LFCD<HardwareSerial>*) arg; //retrieve lidar pointer from the passed argument

    setup_t tftSetup;  tft.getSetup(tftSetup);
    if((tftSetup.pin_tft_mosi != 23) || (tftSetup.pin_tft_clk != 18) || (tftSetup.pin_tft_cs != TFT_CS) || (tftSetup.pin_tft_dc != TFT_DC) || (tftSetup.port != SPI_PORT_TO_USE)) 
    { log_e("pinout mismatch, edit User_Setup.h in TFT_eSPI library"); 
      log_e("%d, %d, %d, %d, %d, %d", tftSetup.pin_tft_mosi, tftSetup.pin_tft_clk, tftSetup.pin_tft_cs, tftSetup.pin_tft_dc, tftSetup.port);   }
    tft.begin();
    #ifdef TFT_ROTATION
      tft.setRotation(TFT_ROTATION);
    #endif
    tft.fillScreen(BG_COLOR);
    tft.initDMA();
    log_v("getFreeHeap before displaybuffer: %d", ESP.getFreeHeap());
    displayBufferPointer = (uint16_t*) displayBuffer.createSprite(SCREENHEIGHT, SCREENHEIGHT);
    log_v("getFreeHeap after displaybuffer: %d", ESP.getFreeHeap());
    //setup complete, loop from here
    tft.startWrite(); //start writing stuff (and never really stop
    uint32_t speedMes[4];
    while(1) {
      speedMes[0] = micros();
      displayBuffer.fillSprite(BG_COLOR);
      displayBuffer.fillCircle(screenMid[0], screenMid[1], 2, CENTER_COLOR);
      for(int16_t i=0; i<360; i++) {
        while(lidarPointer->dataArrayWriting) {} //wait
        int32_t dataPoint = lidarPointer->dataArray[i][1]; //only retrieve distance, other parameter ([0]) is wack
        if(compareDataSet && ((abs(((int32_t)compareData[i])-dataPoint) < compareErrorThreshold) || (dataPoint == 0) || (compareDataPointsCalculated < 360))) {
          if(compareData[i] > 0) {
            if(compareDataPointsCalculated < 360) {
              float floatPos[2];  distAngleToPos(radians(i), compareData[i], floatPos); //calculate position given angle and distance (float floatPos[2])
              convertToPixelPoint(floatPos, compareDataPoints[i]); //convert to pixel using (UN)ZOOMVAR
              compareDataPointsCalculated++; //count up to 360
            }
            displayBuffer.drawLine(lastComparePoint[0], lastComparePoint[1], compareDataPoints[i][0], compareDataPoints[i][1], COMPAREDATA_COLOR); //just ignore the zero-eth line
            lastComparePoint[0] = compareDataPoints[i][0];  lastComparePoint[1] = compareDataPoints[i][1];  lastComparePoint[2] = i; //store last point
            #ifdef drawPointCircles
              displayBuffer.fillCircle(compareDataPoints[i][0], compareDataPoints[i][1], drawPointCircles, COMPAREDATA_COLOR);
            #endif
          } else { if(compareDataPointsCalculated < 360) { compareDataPointsCalculated++; } }
        } else if(dataPoint > 0) {
          float angle=radians(i);
          float floatPos[2];  distAngleToPos(angle, dataPoint, floatPos); //calculate position given angle and distance (float floatPos[2])
          uint16_t pixelPoint[2];  convertToPixelPoint(floatPos, pixelPoint); //convert to pixel using (UN)ZOOMVAR
          if(compareDataSet) { //this if-statement could be formatted as 1 neat (readable) one, but this is more computationally conservative
            if(lastPoint[2] == degRoll(i-1)) { displayBuffer.drawLine(lastPoint[0], lastPoint[1], pixelPoint[0], pixelPoint[1], DATA_LINE_COLOR); }
          } else {
            displayBuffer.drawLine(lastPoint[0], lastPoint[1], pixelPoint[0], pixelPoint[1], DATA_LINE_COLOR);
          }
          lastPoint[0] = pixelPoint[0];  lastPoint[1] = pixelPoint[1];  lastPoint[2] = i; //store last point
          #ifdef drawPointCircles
            displayBuffer.fillCircle(pixelPoint[0], pixelPoint[1], drawPointCircles, DATA_POINT_COLOR);
          #endif
        }
      }
      speedMes[1] = micros();
      tft.pushImageDMA((SCREENWIDTH-SCREENHEIGHT)/2, 0, SCREENHEIGHT, SCREENHEIGHT, displayBufferPointer);
      speedMes[2] = micros();
//      // the SPI transaction will read the displaybuffer in the background while the CPU continues, so dont edit that buffer while that's going on
//      tft.dmaWait(); // wait until DMA is complete
//      //alternatively, you could double-buffer it, by getting a second TFT_eSprite array. This doesn't make sense though, as drawing the display takes no time compared to the lidar's 200ms between new data
//      speedMes[3] = micros();
      //Serial.print(speedMes[1]-speedMes[0]); Serial.write(' '); Serial.print(speedMes[2]-speedMes[1]); Serial.write(' '); Serial.println(speedMes[3]-speedMes[2]); //drawtime, DMA-queue-time, DMAwait-time: "8587 38 23055"
      #ifdef tft_frame_wait
        uint32_t threeSixtyPeriod=200000; if(lidarPointer->RPMraw > 0) { threeSixtyPeriod=600000000L/lidarPointer->RPMraw; }
        if((micros()-speedMes[0]) < threeSixtyPeriod) { //if drawing the frame took less time than 1 rotation (it should)
          uint32_t oldRotationCount = lidarPointer->rotationCount;
          while(oldRotationCount == lidarPointer->rotationCount) { //while waiting for the new datapoints
            //tft.dmaBusy() //check if spi transfer is still going
            delay(1); 
            while(lidarPointer->dataArrayWriting) {} //wait
          }
        }
      #endif
      tft.dmaWait(); // wait until DMA is complete (if it's still going). if tft_frame_wait is defined, this should never take time
    }
    tft.endWrite(); //will never happen, because of while(1) loop
  }
#endif

#ifdef DACdisplay
  #define UNZOOMVAR 200.0
  void drawLidarDataOnDACforXYplot(void *arg) {
    disableCore0WDT(); //screw that watchdog timer
    HLS_LFCD<HardwareSerial>* lidarPointer = (HLS_LFCD<HardwareSerial>*) arg; //retrieve lidar pointer from the passed argument
    //i could've used the global variable, but this is more fun
    pinMode(25, OUTPUT);   pinMode(26, OUTPUT);
    while(1) {
      for(uint16_t i=0; i<360; i++) {
        while(lidarPointer->dataArrayWriting) {} //wait
        uint16_t dataPoint = lidarPointer->dataArray[i][1]; //only retrieve distance, other parameter ([0]) is wack
        
        float dist = dataPoint; //convert uint16_t to float for exact math (frick speed, right)
        float angle = radians(i);
        float xPoint = cos(angle) * dist;
        float yPoint = sin(angle) * dist;
        dacWrite(25, (uint8_t) 127 + ((int8_t) constrain(xPoint*127.0/UNZOOMVAR, -127, 127)));
        dacWrite(26, (uint8_t) 127 + ((int8_t) constrain(yPoint*127.0/UNZOOMVAR, -127, 127)));
      }
    }
  }
#endif

const uint8_t startStopButton = 0;
bool buttonPressed = false;
uint32_t buttonDebounceTimer = 0;
const uint32_t buttonDebounceInterval = 250;

void setup() {
  Serial.begin(250000);
  Serial.println();
  //lidar.begin();
  lidar.begin(16, 17); //on the ESP32, the (three) serial interfaces can use any (most) pins, so for pin reassignment, fill in (rxPin, txPin) , p.s. i'd recommend using Serial1 for this, as its default pins are 9&10, which are connected to internal flash
  
  lidar.postParseCallback = newDataCallback; //set callback function
  
  pinMode(startStopButton, INPUT_PULLUP);

  #if defined(ILI9341displayAdafruit) || defined(ILI9341displayTFT_eSPI)
    xTaskCreatePinnedToCore(drawLidarDataOnILI9341, "ILI9341plot", 2048, &lidar, 1, NULL, 0); //start drawLidarDataOnILI9341() on core 0 and pass the address of the lidar
  #elif defined(DACdisplay)
    xTaskCreatePinnedToCore(drawLidarDataOnDACforXYplot, "DACplot", 2048, &lidar, 1, NULL, 0); //start drawLidarDataOnDACforXYplot() on core 0 and pass the address of the lidar
  #endif
}

void loop() {
  lidar.run(); //needs to be ran more than 300 times per second, otherwise the serial buffer will fill up
  
  if(!digitalRead(startStopButton)) {
    if(!buttonPressed) {
      if(millis() > buttonDebounceTimer) {
        buttonPressed = true;
        if(lidar.spinning) {
//          lidar.stopSpinning();
//          //Serial.println("button stop");
//          fewMorePrints=3;
          storeCompareData();
        } else {
          lidar.startSpinning();
          Serial.println("button start");
          fewMorePrints=-1;
        }
      }
    }
  } else {
    if(buttonPressed) { buttonDebounceTimer = millis() + buttonDebounceInterval; }
    buttonPressed = false;
  }
  
  while(Serial.available()) {
    char receivedChar = Serial.read();
    if(receivedChar == 'b') { lidar.startSpinning(); /*Serial.println("starting...");*/ fewMorePrints=-1; }
    else if(receivedChar == 'e') { lidar.stopSpinning(); /*Serial.println("stopping...");*/ fewMorePrints=3; }
    else if(receivedChar == 'c') { 
      storeCompareData();
    }
  }
//  if((millis() > printTimer) && fewMorePrints) { //if fewMorePrints is nonzero
//    printTimer = millis() + 500;  if(fewMorePrints > 0) { fewMorePrints--; } // if fewMorePrints is negative, it will just run forever
//    Serial.print(lidar.spinning); Serial.print(' ');
//    Serial.print(lidar.rotationCount); Serial.print(' ');
//    Serial.print(lidar.RPMraw); Serial.print(' ');
//    Serial.print(lidar.RPM()); Serial.print("  ");
//    while(lidar.dataArrayWriting) {} //wait
//    Serial.print(lidar.volatileDegree); Serial.print(' ');
//    Serial.print(lidar.dataArray[0][0]); Serial.print('\t');
//    Serial.print(lidar.dataArray[0][1]); Serial.print(' ');
//    Serial.println(lidar.dataArray[179][1]);
//  }
}
