#include <Arduino_AVRSTL.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <vector>
//#include <AltSoftSerial.h>
//uint8_t CS;
#define COM 0x55

std::vector<int> pos(3, 0);
std::vector<int> pool(3, 0);
int xLength = 1;
int yLength = 1;
bool fwd = true;
SoftwareSerial port1(9, 10);
SoftwareSerial port2(7, 8);
SoftwareSerial port3(5, 6);
SoftwareSerial port4(3, 4);

void setup() {
  Serial.begin(115200);
  port1.begin(115200);
  port2.begin(115200);
  port3.begin(115200);
  port4.begin(115200);

  Wire.begin(1);
  Wire.onRequest(sendPosition);
}

void sendPosition(){
  Wire.write(pos[0]);
  Wire.write(pos[1]);
  if 
}

int getDistance(SoftwareSerial& port){
    unsigned char buffer_RTT[4] = {0};
    int distance = 0;
  
    buffer_RTT[0] = 0xff;
    for (int i=1; i<4; i++){
        buffer_RTT[i] = port.read();
    }
    uint8_t CS = buffer_RTT[0] + buffer_RTT[1] + buffer_RTT[2];
    if(buffer_RTT[3] == CS) {
        distance = (buffer_RTT[1] << 8) + buffer_RTT[2];
    }
    return distance;
}

int getData(SoftwareSerial& port, int num){
  int distance = 0;
  
  port.listen();
  //delay(10);
  port.write(COM);
  
  delay(50); 
  if(port.available()){
      if(port.read() == 0xff) distance = getDistance(port);
  }
  return distance;
}

void loop() {
  int xl = getData(port1, 1);

  int xr = getData(port2, 2);
  //delay(500);
  int yf = getData(port3, 3);

  int yb = getData(port4, 4);
  if(fwd){
    pos[0] = xl+xLength/2;
    pos[1] = yb+yLength/2;
  } else{
    
  }
  pool[0] = xr+xl+xLength;
  pool[1] = yf+yb+yLength;

  // Serial.write(pos);
  Serial.print("x: ");
  Serial.println(pos[0]);
  Serial.print("y: ");
  Serial.println(pos[1]);
  Serial.println();

//  delay(3000);
}
