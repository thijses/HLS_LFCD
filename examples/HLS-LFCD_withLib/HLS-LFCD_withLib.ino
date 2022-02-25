/*
TODO:
- serial timeout to detect if still spinning (and stopSpinningAndWait())
- serial timeout to detect missed bytes?
- IRAM (and DRAM) attributes?


*/

#include "HLS_LFCD.h"

//#define lidarDebugSerial Serial

//#define DACdisplay
#define ILI9341displayAdafruit
//#define ILI9341displayTFT_eSPI

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
