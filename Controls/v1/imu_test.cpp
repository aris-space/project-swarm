#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Global variables
float heading = 0.0;
float pitch = 0.0;
float roll = 0.0;

float initialEulerX = 0.0;
float initialEulerY = 0.0;
float initialEulerZ = 0.0;

uint16_t sampling_rate = 100;  // how often to read data from the board

float PreassureSensorOffset = 0.0;  

//Sensor axis
/* 
         +----------+
         |         *| RST   PITCH  ROLL  HEADING
     ADR |*        *| SCL
     INT |*        *| SDA     ^            /->
     PS1 |*        *| GND     |            | 
     PS0 |*        *| 3VO     Y    Z-->    \-X
         |         *| VIN
         +----------+

Vin => 5V
GND => GND
SDA => A4
SCL => A5
*/

// Create BNO055 object
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Helper Functions
void BNO055_calibration(){
    while (!bno.isFullyCalibrated()) {
        uint8_t sys, gyro, accel, mag;
        bno.getCalibration(&sys, &gyro, &accel, &mag);
        Serial.print(F("Calibration: "));
        Serial.print(sys, DEC);
        Serial.print(F(" "));
        Serial.print(gyro, DEC);
        Serial.print(F(" "));
        Serial.print(accel, DEC);
        Serial.print(F(" "));
        Serial.println(mag, DEC);

        delay(sampling_rate);
    }
}

double angle_correction(double angle) {
  // If the angle is less than 0, add 360 to bring it into the 0-360 range
  if (angle < 0) angle += 360;

  // If the angle is greater than or equal to 360, subtract 360 to wrap it
  if (angle >= 360) angle -= 360;

  return angle;
}

void setup(void) {
    // Initialize Serial Monitor
    Serial.begin(115200);

    // Wait for serial port to open
    while (!Serial) delay(10);  

    // Initialize BNO055 sensor
    if (!bno.begin()) {
        Serial.print("No BNO055 detected");
        while (1) {
        }
    }

    // Set the sensor to NDOF mode to ensure all sensors are active
    bno.setMode(OPERATION_MODE_NDOF);

    // Use external crystal for better accuracy 
    bno.setExtCrystalUse(true);
    
    // Calibrate all the sensors before starting the program
    BNO055_calibration();
    Serial.println("Fully calibrated");

    // Give some time to orient the sensor and enter configuration mode
    delay(3000);
    
    Serial.println("Starting offset calculation");

    // Once fully calibrated, read and store the initial Euler angles
    imu::Vector<3> initialEuler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    initialEulerX = initialEuler.x();
    initialEulerY = initialEuler.y();
    initialEulerZ = initialEuler.z();

    Serial.println("Start preassure sensor calibration");

    // Take 10 voltage readings and calculate the average
    float totalVoltage = 0.0;
    for (int i = 0; i < 10; i++) {
        float V = analogRead(0) * 5.00 / 1024;  // Read the sensor voltage
        totalVoltage += V;  // Accumulate voltage values
        delay(sampling_rate);  // Delay between readings to avoid rapid sampling
    }
  
    // Calculate the average voltage as the offset
    PreassureSensorOffset = totalVoltage / 10;

    delay(3000);
}

void loop(void){

    // Get orientation event data
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    // Calculate the heading, pitch and roll
    heading = angle_correction(orientationData.orientation.x - initialEulerX);
    pitch = angle_correction(orientationData.orientation.y - initialEulerY);
    roll = angle_correction(orientationData.orientation.z - initialEulerZ);

    // Print rotation angles to the serial monitor
    Serial.print("Heading: ");
    Serial.print(heading, 4);
    Serial.print("\tPitch: ");
    Serial.print(pitch, 4);
    Serial.print("\tRoll: ");
    Serial.println(roll, 4);

    // Read the preassure sensor voltage
    float V = analogRead(0) * 5.00 / 1024;
  
    // Calculate pressure in kPa using the calibrated offset
    float P = (V - PreassureSensorOffset) * 250;
  
    // Calculate depth in meters based on pressure (P is in kPa)
    float depth = P / 9.81; 

    // Print the depth to the Serial Monitor
    Serial.print("Depth in Water: ");
    Serial.print(depth * 100, 0);  // Print depth with 2 decimal places
    Serial.println(" cm");

    delay(sampling_rate); // Small delay to avoid flooding the serial output
}

