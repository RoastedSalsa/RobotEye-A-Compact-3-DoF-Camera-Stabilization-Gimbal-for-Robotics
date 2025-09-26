// --- IMU_and_sending.ino ---
// Reads BNO055 Euler Z (gamma) and sends it every 100ms over Serial1 as a float.

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Create the BNO055 object. The argument "55" is the default I²C address (0x28 or 0x29).
Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  // USB Serial for debugging:
  Serial.begin(115200);
  delay(100);  // Give USB Serial a moment to start

  // Initialize I2C and BNO055:
  if (!bno.begin(OPERATION_MODE_NDOF)) {
    Serial.println("Error: BNO055 not detected. Check wiring!");
    while (1) delay(10);
  }
  // Use external crystal for better stability (if your breakout has it wired):
  bno.setExtCrystalUse(true);

  // Give sensor some time to boot into NDOF fusion mode:
  delay(1000);

  // Initialize Serial1 (UART1) at 115200 to send gamma to Teensy 4.0:
  Serial1.begin(115200);

  Serial.println("BNO055 initialized. Starting to send gamma → Teensy 4.0...");
}

void loop() {
  static unsigned long lastSend = 0;
  const unsigned long sendInterval = 100; // send every 100ms

  // Only read/send at the desired interval:
  if (millis() - lastSend >= sendInterval) {
    lastSend = millis();

    // Read the fused Euler angles from BNO055:
    sensors_event_t event;
    bno.getEvent(&event);

    // event.orientation.x = roll (ϕ)
    // event.orientation.y = pitch (θ)
    // event.orientation.z = yaw (ψ)  → we’ll treat this as "gamma"
    float gamma = event.orientation.z;

    // Send gamma over Serial1 (ASCII, 3 decimal places), terminated with '\n':
    Serial1.print(gamma, 3);
    Serial1.print('\n');

    // Also print to USB Serial for debugging:
    Serial.print("Sent γ = ");
    Serial.println(gamma, 3);
  }

  // (Optional) If Teensy 4.0 ever sends a reply back on UART, you can read it here:
  if (Serial1.available()) {
    String reply = Serial1.readStringUntil('\n');
    Serial.print("[Reply from 4.0]: ");
    Serial.println(reply);
  }
}
