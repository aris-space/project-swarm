#include <Servo.h>

Servo esc;

void setup() {
  esc.attach(9);  // ESC signal wire connected to pin 9
  Serial.begin(9600);
}

void loop() {
  // esc.writeMicroseconds(1100);  // Slightly above minimum throttle
  // delay(3000);  // Wait for 3 seconds to see if the motor spins
  
  // esc.writeMicroseconds(1500);  // Mid-throttle
  // delay(3000);  // Wait for 3 seconds to see if the motor spins
  
  for(int i = 1000; i < 1300; i++){
    esc.writeMicroseconds(i);
    delay(20);
  }
  delay(1000);
  // esc.writeMicroseconds(2000);  // Full throttle
  // delay(3000);  // Wait for 3 seconds to see if the motor spins
  // Serial.write("1");
  // esc.writeMicroseconds(1000);  // Back to minimum throttle
  // delay(3000);  // Wait for 3 seconds to see if the motor stops
  // Serial.write("2");
}
