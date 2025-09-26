#include <SimpleFOC.h>
#include <Wire.h>

// Use same motor/sensor/driver definitions as main code
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(2, 3, 6, 9, 8, 7);  // motor1 setup

Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void doVelPID(char* cmd) { command.pid(&motor.PID_velocity, cmd); }
void doAngPID(char* cmd) { command.pid(&motor.P_angle, cmd); }

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  sensor.init(&Wire);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 10;
  driver.voltage_limit = 10;
  driver.init();
  driver.enable();
  motor.linkDriver(&driver);

  // Default PID settings
  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 3.0;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.01;

  motor.P_angle.P = 30;
  motor.P_angle.I = 0;
  motor.P_angle.D = 0.01;

  motor.voltage_limit = 10;
  motor.controller = MotionControlType::angle;

  motor.init();
  motor.initFOC();

  // Start at current angle
  motor.target = sensor.getAngle();

  // Add tuning commands
  command.add('T', doTarget, "target angle");
  command.add('V', doVelPID, "velocity PID");
  command.add('A', doAngPID, "angle PID");

  Serial.println(F("PID tuning ready."));
  Serial.println(F("Commands:"));
  Serial.println(F("  T<target>  → Set target angle [rad]"));
  Serial.println(F("  V<P,I,D>   → Velocity PID"));
  Serial.println(F("  A<P,I,D>   → Angle PID"));
}

void loop() {
  motor.loopFOC();
  motor.move();

  command.run();

  // Optional: print sensor/motor data for graphing
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.print("Target: ");
    Serial.print(motor.target);
    Serial.print("\tPos: ");
    Serial.println(sensor.getAngle());
  }
}
