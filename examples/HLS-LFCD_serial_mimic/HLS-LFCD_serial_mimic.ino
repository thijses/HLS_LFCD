uint8_t messageArray[42] = {0xFA, //sync byte
                            0xA0, //angle index byte
                            0xC3, 0x0B, //RPM data
                            0xB0, 0x22, 0x8B, 0x00, 0x00, 0x00, //angle index+0deg data
                            0x2C, 0x30, 0x89, 0x00, 0x00, 0x00, //angle index+1deg data
                            0x42, 0x27, 0x8A, 0x00, 0x00, 0x00, //angle index+2deg data
                            0x3C, 0x21, 0x8C, 0x00, 0x00, 0x00, //angle index+3deg data
                            0xC6, 0x0E, 0X98, 0x00, 0x00, 0x00, //angle index+4deg data
                            0X3F, 0X16, 0XA4, 0x00, 0x00, 0x00, //angle index+5deg data
                            0x14, 0x14}; //checksum bytes


#define secondSerial Serial1

bool spinning = false;
unsigned long sendTimer;
unsigned int packetsPerSec = 300; //300RPM/60sec = 5RPS -> 5RPS*60packetsPerRotation = 300 packets per sec
unsigned long sendTimeout = 1000000/packetsPerSec; //(micros) time between sending data
int rotationsToPrint = -1; //-1 means infinite (untill otherwise stopped)

void setup() {
  Serial.begin(74880);
  Serial.print("checksum check: ");
  uint16_t sum = 0;  for(uint8_t i=0; i<40; i++) { sum += messageArray[i]; } //calculate sum
  if(messageArray[40] == (0xFF - (0xFF & sum))) {
    Serial.println("success!");
  } else {
    Serial.print("failed: "); Serial.print(messageArray[40], HEX); Serial.print(" != "); Serial.println(0xFF - (0xFF & sum), HEX);
    Serial.print("sum was: "); Serial.print(sum); Serial.print(" = "); Serial.println(sum, HEX);
  }
  secondSerial.begin(230400);
}

void loop() {
  while(Serial.available()) {
    char receivedChar = Serial.read();
    if(receivedChar == 'b') { spinning = true; sendTimer = micros(); rotationsToPrint=-1; Serial.println("spinning"); }
    else if(receivedChar == 'e') { spinning = false; rotationsToPrint=-1; Serial.println("stopping"); }
    else if((receivedChar >= '1') && (receivedChar <= '9')) {
      messageArray[1] = 0xA0;
      rotationsToPrint=receivedChar - '0';
      Serial.print("spinning "); Serial.print(rotationsToPrint); Serial.println(" rotations");
      spinning = true; sendTimer = micros();
    }
    else if(receivedChar == 's') { //single packet
      sendSixDegPacket();
      Serial.print("sent: "); Serial.print(messageArray[1], HEX); Serial.print(" = "); Serial.println((messageArray[1]-0xA0)*6);
      messageArray[1]++; if(messageArray[1] > 0xDB) { messageArray[1] = 0xA0; }
    }
  }

  if(spinning && ((micros() - sendTimer) > sendTimeout)) {
    sendTimer = micros();
    sendSixDegPacket();
    
    messageArray[1]++; if(messageArray[1] > 0xDB) { 
      messageArray[1] = 0xA0;
      if(rotationsToPrint) { //if it has a positive value
        rotationsToPrint--;  
        if (!(rotationsToPrint)) { //if it is now 0
          spinning = false;
          rotationsToPrint=-1;
          Serial.println("finite rotations complete");
        }
      }
    }
  }
}

void sendSixDegPacket() {
  uint16_t sum = 0;  for(uint8_t i=0; i<40; i++) { sum += messageArray[i]; } //calculate sum
  messageArray[40] = (0xFF - (0xFF & sum));
  messageArray[41] = messageArray[40]; //i dont know of this is correct, but just for now
  secondSerial.write(messageArray, 42);
}
