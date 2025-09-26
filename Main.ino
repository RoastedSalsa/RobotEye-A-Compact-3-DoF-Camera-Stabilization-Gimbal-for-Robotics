#include <SimpleFOC.h>
#include <Wire.h>

// primary AS5600 on I2C0 (pins 18=SDA, 19=SCL)
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);

BLDCMotor      motor1  = BLDCMotor(7);
BLDCDriver6PWM driver1 = BLDCDriver6PWM(2, 3, 6, 9, 8, 7);

Commander command = Commander(Serial);
void onAngleA(char* cmd) { command.scalar(&motor1.target, cmd); }

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  // If you don’t want to block waiting for USB-Serial, comment this out:
  // while(!Serial);

  Wire.begin();
  sensor1.init(&Wire);
  for (int i = 0; i < 10; i++) {
    sensor1.update();
    delay(20);
  }

  motor1.linkSensor(&sensor1);
  driver1.voltage_power_supply = 10;
  driver1.voltage_limit        = 10;
  driver1.init();
  driver1.enable();
  motor1.linkDriver(&driver1);

  motor1.PID_velocity.P = 0.1;
  motor1.PID_velocity.I = 3;
  motor1.PID_velocity.D = 0;
  motor1.PID_velocity.output_ramp = 1000;
  motor1.LPF_velocity.Tf = 0.01;

  motor1.voltage_limit = 10;
  motor1.controller    = MotionControlType::angle;
  motor1.P_angle.P     = 20;
  motor1.P_angle.I     = 0;
  motor1.P_angle.D     = 0;

  motor1.init();
  if (!motor1.initFOC()) {
    Serial.println("ERROR: initFOC failed for motor1!");
    while (1) { delay(10); }
  }

  motor1.target = sensor1.getAngle();
  command.add('A', onAngleA, "set motor1 target [rad]");
  Serial.println("Ready! Send 'A<rad>' to move.");
}

void loop() {
  static float lastF = 0.0;      // holds the last value we received
  String incoming;

  // 1) If there is a new line on Serial1, read and update lastF
  if (Serial1.available()) {
    incoming = Serial1.readStringUntil('\n');
    float parsed = incoming.toFloat();
    lastF = parsed;               // overwrite lastF only when new data arrives

    Serial.print("[Got from 4.1] ");
    Serial.println(incoming);

    // (Optional) reply back
    Serial1.print("B received: ");
    Serial1.print(incoming);
    Serial1.print('\n');
    Serial.println("[Replied to 4.1]");
  }

  // 2) Compute desired angle = lastF + 3.0, then clamp between [2.0, 4.0]
  float desired = lastF*PI/180 + 3.0;
  if (desired < 2.0) desired = 2.0;
  if (desired > 4.0) desired = 4.0;

  // 3) Run FOC & move motor toward the clamped target
  sensor1.update();
  motor1.loopFOC();
  command.run();
  motor1.move(3.4);

  // 4) Print status:
  Serial.print(lastF);
  Serial.print("T:");   Serial.print(motor1.target, 3);
  Serial.print("  A1:"); Serial.print(sensor1.getAngle(), 3);
  Serial.print("  V:");  Serial.println(motor1.shaft_velocity, 3);
  Serial.print("  →desired: "); Serial.println(desired, 3);

  delay(2);
}
