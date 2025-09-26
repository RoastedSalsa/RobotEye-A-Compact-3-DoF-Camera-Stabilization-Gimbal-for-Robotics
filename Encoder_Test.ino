#include <SimpleFOC.h>
#include <Wire.h>

//——— Select your sensor type here ———
// For AS5600 over I2C (default address 0x36):
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

void setup() {
  // 1) Start USB serial for debugging
  Serial.begin(115200);
  while (!Serial) { /* wait for Serial Monitor to open */ }

  // 2) Initialize I²C bus
  Wire.begin();  // uses default SDA/SCL pins

  // 3) Initialize the sensor (no return value to check)
  sensor.init(&Wire);
  // Give it a moment to stabilize:
  delay(100);

  Serial.println("✅ Sensor initialized. Reading angles...");
}

void loop() {
  // 4) Read/update the sensor
  sensor.update();

  // 5) Get the raw angle in radians (0 … 2π)
  float angle_rad = sensor.getAngle();

  // 7) Print to Serial
  Serial.print("Angle (rad): ");
  Serial.println(angle_rad, 4);

  delay(200);  // adjust as needed
}
