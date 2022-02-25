#if defined(ILI9341displayAdafruit) || defined(ILI9341displayTFT_eSPI) //shared TFT values
  //#define ZOOMVAR 0.1
  #define UNZOOMVAR 30.0
  #define TFT_ROTATION 3
  #define RECALC_COMPARE_PIXELS  //if defined the loop recalculates the position of compare data points every time (only makes sense if (individual) compare data points change)
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
  //alternatively, you could store only the index of the last compare point and take the array values of that point
  //EXCEPT, it only makes sense to do if RECALC_COMPARE_PIXELS is undefined, if it's defined then you'd have to recalculate 2 points instead of 1

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
        if(compareDataSet 
           && ( (abs(((int32_t)compareData[i])-dataPoint) < compareErrorThreshold) 
                || (dataPoint == 0)
                #ifndef RECALC_COMPARE_PIXELS
                || (compareDataPointsCalculated < 360)
                #endif
                )) {
          if(compareData[i] > 0) {
            #ifndef RECALC_COMPARE_PIXELS
              if(compareDataPointsCalculated < 360) {
            #endif
              float floatPos[2];  distAngleToPos(radians(i), compareData[i], floatPos); //calculate position given angle and distance (float floatPos[2])
              convertToPixelPoint(floatPos, compareDataPoints[i]); //convert to pixel using (UN)ZOOMVAR
            #ifndef RECALC_COMPARE_PIXELS
                compareDataPointsCalculated++; //count up to 360
              }
            #endif
            displayBuffer.drawLine(lastComparePoint[0], lastComparePoint[1], compareDataPoints[i][0], compareDataPoints[i][1], COMPAREDATA_COLOR); //just ignore the zero-eth line
            lastComparePoint[0] = compareDataPoints[i][0];  lastComparePoint[1] = compareDataPoints[i][1];  lastComparePoint[2] = i; //store last point
            #ifdef drawPointCircles
              displayBuffer.fillCircle(compareDataPoints[i][0], compareDataPoints[i][1], drawPointCircles, COMPAREDATA_COLOR);
            #endif
          } else { 
            #ifndef RECALC_COMPARE_PIXELS
              if(compareDataPointsCalculated < 360) { compareDataPointsCalculated++; } 
            #endif
          }
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
        if(compareDataSet 
           && ( (abs(((int32_t)compareData[i])-dataPoint) < compareErrorThreshold) 
                || (dataPoint == 0)
                #ifndef RECALC_COMPARE_PIXELS
                || (compareDataPointsCalculated < 360)
                #endif
                )) {
          if(compareData[i] > 0) {
            #ifndef RECALC_COMPARE_PIXELS
              if(compareDataPointsCalculated < 360) {
            #endif
              float floatPos[2];  distAngleToPos(radians(i), compareData[i], floatPos); //calculate position given angle and distance (float floatPos[2])
              convertToPixelPoint(floatPos, compareDataPoints[i]); //convert to pixel using (UN)ZOOMVAR
            #ifndef RECALC_COMPARE_PIXELS
                compareDataPointsCalculated++; //count up to 360
              }
            #endif
            displayBuffer.drawLine(lastComparePoint[0], lastComparePoint[1], compareDataPoints[i][0], compareDataPoints[i][1], COMPAREDATA_COLOR); //just ignore the zero-eth line
            lastComparePoint[0] = compareDataPoints[i][0];  lastComparePoint[1] = compareDataPoints[i][1];  lastComparePoint[2] = i; //store last point
            #ifdef drawPointCircles
              displayBuffer.fillCircle(compareDataPoints[i][0], compareDataPoints[i][1], drawPointCircles, COMPAREDATA_COLOR);
            #endif
          } else { 
            #ifndef RECALC_COMPARE_PIXELS
              if(compareDataPointsCalculated < 360) { compareDataPointsCalculated++; } 
            #endif
          }
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

//#ifdef compositedisplay
//  #define UNZOOMVAR 20.0
//  const int XRES = 320;
//  const int YRES = 200;
//  const int16_t screenMid[2] = {XRES/2, YRES/2};
//  
//  CompositeOutput composite(CompositeOutput::NTSC, XRES * 2, YRES * 2);
//  uint8_t **frame; uint8_t **backbuffer;
//
//  void fillScreen(uint8_t val) {
//    for(int y = 0; y < yres; y++)
//      for(int x = 0; x < xres; x++)
//        backbuffer[y][x] = val;
//  }
//  void frameBufSwitch() {
//    uint8_t **temp = backbuffer;
//    backbuffer = frame;
//    frame = temp;
//  }
//
//  void drawLidarDataOnComposite(void *arg) {
//    disableCore0WDT(); //screw that watchdog timer
//    HLS_LFCD<HardwareSerial>* lidarPointer = (HLS_LFCD<HardwareSerial>*) arg; //retrieve lidar pointer from the passed argument
//    //i could've used the global variable, but this is more fun
//    composite.init();
//    //pinMode(25, OUTPUT);
//    frame = (uint8_t**)malloc(YRES * sizeof(uint8_t*));  backbuffer = (uint8_t**)malloc(yres * sizeof(uint8_t*));
//    for(int y = 0; y < YRES; y++) 
//    { frame[y] = (uint8_t*)malloc(XRES);  backbuffer[y] = (uint8_t*)malloc(xres); }
//    while(1) {
//      for(uint16_t i=0; i<360; i++) {
//        //while(lidarPointer->dataArrayWriting) {} //cant wait, gotta hurry
//        uint16_t dataPoint = lidarPointer->dataArray[i][1]; //only retrieve distance, other parameter ([0]) is wack
//        
//        float dist = dataPoint; //convert uint16_t to float for exact math (frick speed, right)
//        float angle = radians(i);
//        float xPoint = cos(angle) * dist;
//        float yPoint = sin(angle) * dist;
//      }
//      frameBufSwitch();
//
//      composite.sendFrameHalfResolution(&frame);
//    }
//  }
//#endif

void HLS_LFCD_display_init() {
  #if defined(ILI9341displayAdafruit) || defined(ILI9341displayTFT_eSPI)
    xTaskCreatePinnedToCore(drawLidarDataOnILI9341, "ILI9341plot", 2048, &lidar, 1, NULL, 0); //start drawLidarDataOnILI9341() on core 0 and pass the address of the lidar
  #elif defined(DACdisplay)
    xTaskCreatePinnedToCore(drawLidarDataOnDACforXYplot, "DACplot", 2048, &lidar, 1, NULL, 0); //start drawLidarDataOnDACforXYplot() on core 0 and pass the address of the lidar
//  #elif defined(compositedisplay)
//    xTaskCreatePinnedToCore(drawLidarDataOnComposite, "compositeplot", 2048, &lidar, 1, NULL, 0); //start drawLidarDataOnComposite() on core 0 and pass the address of the lidar
  #else
    #warning("no display method defined")
  #endif
}
