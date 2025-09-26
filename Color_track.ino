#include <SimpleFOC.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>  // for imu::Quaternion

//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// 1) SimpleFOC dual‐motor + two AS5600 sensors

// Primary AS5600 on I2C0 (pins 18=SDA, 19=SCL)
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);

// Secondary AS5600 on I2C1 (pins 17=SDA1, 16=SCL1)
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);

// Motor 1 (driver pins 2,3,6,9,8,7)
BLDCMotor      motor1  = BLDCMotor(7);
BLDCDriver6PWM driver1 = BLDCDriver6PWM(2, 3, 6, 9, 8, 7);

// Motor 2 (driver pins 4,33,36,37,29,28)
BLDCMotor      motor2  = BLDCMotor(7);
BLDCDriver6PWM driver2 = BLDCDriver6PWM(4, 33, 36, 37, 29, 28);

Commander command = Commander(Serial);
void onAngleA(char* cmd) { command.scalar(&motor1.target, cmd); }
void onAngleB(char* cmd) { command.scalar(&motor2.target, cmd); }

//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// 2) BNO055 on I2C0 (shared with sensor1) → send “gamma” over Serial1 as a float

// BNO055 default I2C address (0x28 or 0x29)
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Store the initial quaternion as a “zero” reference (captured once, after gimbal = home)
imu::Quaternion quatInitial;

// Send‐interval for BNO055 (ms)
static const unsigned long BNO_SEND_INTERVAL = 100;
static unsigned long lastBnoSendTime = 0;

// Variables for incoming yaw/pitch from Pi
float pi_yaw = 0.0;
float pi_pitch = 0.0;

//=============================================================================
// Helper: compute the conjugate of quaternion q = [w, x, y, z]:
//   q* = [w, −x, −y, −z]
imu::Quaternion quatConjugate(const imu::Quaternion &q) {
  return imu::Quaternion(q.w(), -q.x(), -q.y(), -q.z());
}

//=============================================================================
// Helper: quaternion multiplication: result = a * b
imu::Quaternion quatMultiply(const imu::Quaternion &a, const imu::Quaternion &b) {
  float aw = a.w(), ax = a.x(), ay = a.y(), az = a.z();
  float bw = b.w(), bx = b.x(), by = b.y(), bz = b.z();

  float rw = (aw * bw - ax * bx - ay * by - az * bz);
  float rx = (aw * bx + ax * bw + ay * bz - az * by);
  float ry = (aw * by - ax * bz + ay * bw + az * bx);
  float rz = (aw * bz + ax * by - ay * bx + az * bw);

  return imu::Quaternion(rw, rx, ry, rz);
}

//=============================================================================
// Helper: convert a quaternion into Euler angles (heading/yaw, roll, pitch) in degrees.
void quatToEuler(const imu::Quaternion &q, float &heading, float &roll, float &pitch) {
  float qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
  float norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  qw /= norm; qx /= norm; qy /= norm; qz /= norm;

  float sinr_cosp = +2.0f * (qw * qx + qy * qz);
  float cosr_cosp = +1.0f - 2.0f * (qx * qx + qy * qy);
  roll = atan2(sinr_cosp, cosr_cosp) * (180.0 / M_PI);

  float sinp = +2.0f * (qw * qy - qz * qx);
  if (fabs(sinp) >= 1)
    pitch = copysign(90.0, sinp);
  else
    pitch = asin(sinp) * (180.0 / M_PI);

  float siny_cosp = +2.0f * (qw * qz + qx * qy);
  float cosy_cosp = +1.0f - 2.0f * (qy * qy + qz * qz);
  heading = atan2(siny_cosp, cosy_cosp) * (180.0 / M_PI);

  if (heading > 180.0f)  heading -= 360.0f;
  if (heading < -180.0f) heading += 360.0f;
}

//=============================================================================

bool initialized = false;

void setup() {
  //––– 1) USB Serial (for debugging) and Serial1 (for BNO055)
  Serial.begin(115200);
  while (!Serial) {}
  Serial1.begin(115200);
  //––– 2) Serial3 for Pi yaw/pitch input
  Serial3.begin(115200);

  //––– 3) Initialize both I2C buses
  Wire.begin();
  Wire1.begin();

  //––– 4) init sensors, motors, BNO055 (same as original)...
  sensor1.init(&Wire);
  sensor2.init(&Wire1);
  for (int i = 0; i < 10; i++) { sensor1.update(); sensor2.update(); delay(20); }

  motor1.linkSensor(&sensor1);
  driver1.voltage_power_supply = 10;
  driver1.voltage_limit        = 10;
  driver1.init(); driver1.enable();
  motor1.linkDriver(&driver1);

  motor2.linkSensor(&sensor2);
  driver2.voltage_power_supply = 10;
  driver2.voltage_limit        = 10;
  driver2.init(); driver2.enable();
  motor2.linkDriver(&driver2);

  // Motor and FOC parameter setup (unchanged)...
  motor1.PID_velocity.P        = 0.1;
  motor1.PID_velocity.I        = 3;
  motor1.PID_velocity.D        = 0;
  motor1.PID_velocity.output_ramp = 1000;
  motor1.LPF_velocity.Tf       = 0.001;
  motor1.voltage_limit         = 10;
  motor1.controller            = MotionControlType::angle;
  motor1.P_angle.P             = 30;
  motor1.P_angle.I             = 0;
  motor1.P_angle.D             = 0.01;

  motor2.PID_velocity.P        = 0.1;
  motor2.PID_velocity.I        = 3;
  motor2.PID_velocity.D        = 0;
  motor2.PID_velocity.output_ramp = 1000;
  motor2.LPF_velocity.Tf       = 0.001;
  motor2.voltage_limit         = 10;
  motor2.controller            = MotionControlType::angle;
  motor2.P_angle.P             = 30;
  motor2.P_angle.I             = 0;
  motor2.P_angle.D             = 0.05;

  motor1.init(); if (!motor1.initFOC()) while(1) delay(10);
  motor2.init(); if (!motor2.initFOC()) while(1) delay(10);

  motor1.target = sensor1.getAngle();
  motor2.target = sensor2.getAngle();
  command.add('A', onAngleA, "set motor1 target [rad]");
  command.add('B', onAngleB, "set motor2 target [rad]");
  Serial.println("Ready! Send 'A<rad>' or 'B<rad>' to move.");

  if (!bno.begin(OPERATION_MODE_NDOF)) { while (1) delay(10); }
  bno.setExtCrystalUse(true);
  delay(1000);
  Serial.println("BNO055 initialized.");
}

void loop() {
  //––– Read incoming from Pi on Serial3
  if (Serial3.available()) {
    String line = Serial3.readStringUntil('\n');
    int comma = line.indexOf(',');
    if (comma > 0) {
      pi_yaw   = -line.substring(0, comma).toFloat();
      pi_pitch = -line.substring(comma + 1).toFloat();
      if(pi_yaw<3 && pi_yaw >-3){
          pi_yaw=0;
      }
      if(pi_pitch<3  && pi_pitch >-3){
          pi_pitch=0;
      }
    }
  }
  else {
    pi_yaw=0;
    pi_pitch=0;
  }

  static float cum_yaw = 0.0;
  static float cum_pitch = 0.0;
  static float lastM1 = 0.0;      // holds the last value we received
  static float lastM2 = 0.0;      // holds the last value we received
  static float lastM3 = 0.0;      // holds the last value we received
  static float talpha = 0.0;
  static float tbeta  = 0.0;
  static float tgamma  = 0.0;
  static float calpha = 0.0;
  static float cbeta = 0.0;
  static float cgamma = 0.0;
  cum_yaw += pi_yaw*PI/5000;
  cum_yaw = constrain(cum_yaw, -1, 1);
  cum_pitch += pi_pitch*PI/5000;
  cum_pitch = constrain(cum_pitch, -1, 1);
  //––– A) Update both AS5600 sensors
  sensor1.update();
  sensor2.update();

  //––– C) Print feedback for motor1 & motor2, run initial‐position routine
  if (!initialized) {
    Serial.print("Starting initialization");

    // Run FOC & control to drive each motor to its “home” angle
    motor1.loopFOC();
    motor2.loopFOC();
    command.run();

    motor1.move(4.34);   // “home” position for motor1
    motor2.move(1.27);   // “home” position for motor2
    Serial1.print(0.43, 3);
    Serial1.print('\n');
    Serial.print("M1:");
    Serial.print(sensor1.getAngle());
    Serial.print("   M2:");
    Serial.println(sensor2.getAngle());
    unsigned long now = millis();

    // Once both motors are within ±0.1 rad of their targets, mark “initialized = true”
    if ((fabs(sensor1.getAngle() - 4.34) <= 0.01) &&
        (fabs(sensor2.getAngle() - 1.27) <= 0.01)) {
      Serial.println("  → Target initialized");
      initialized = true;

      // ** Capture the initial quaternion as “zero reference” here: **
      quatInitial = bno.getQuat();
      Serial.println("  → Stored initial IMU quaternion as zero reference.");

      // Immediately send out the first zeroed quaternion reading:
      // (We apply relative quaternion: q_rel = conj(quatInitial) * q_current)

      imu::Quaternion qCurrent = bno.getQuat();
      imu::Quaternion qConj    = quatConjugate(quatInitial);
      imu::Quaternion qRelative = quatMultiply(qConj, qCurrent);

      float heading, roll, pitch;
      quatToEuler(qRelative, heading, roll, pitch);

      // talpha = roll; tbeta = pitch; tgamma = heading
      talpha = roll/180*PI;
      tbeta  = pitch/180*PI;
      tgamma = heading/180*PI;

      // Send out gamma over Serial1 (ASCII float, 3 decimals, newline)
      Serial1.print(0.43, 3);
      Serial1.print('\n');
      // Echo to USB Serial for debugging
      
      // Initialize lastBnoSendTime so the next block isn’t triggered immediately
      lastBnoSendTime = now;
    }
  }

  //––– D) Once “initialized == true”, every BNO_SEND_INTERVAL ms, read & send zeroed IMU data
  if (initialized) {
    unsigned long now = millis();
    static unsigned long lastLogTime = 0;

    if (now - lastBnoSendTime >= BNO_SEND_INTERVAL) {
      lastBnoSendTime = now;

      // Read current quaternion
      imu::Quaternion qCurrent = bno.getQuat();

      // Compute relative quaternion: q_rel = conj(quatInitial) * qCurrent
      imu::Quaternion qConj     = quatConjugate(quatInitial);
      imu::Quaternion qRelative = quatMultiply(qConj, qCurrent);

      // Convert to Euler angles (degrees)
      float heading, roll, pitch;
      quatToEuler(qRelative, heading, roll, pitch);

      // Store these “zeroed” roll & pitch; send “zeroed” yaw (= heading)

      lastM3 = -heading/180*PI;
      lastM2 = -roll/180*PI+cum_pitch;   // delta between initial‐zeroed roll & current roll
      lastM1 = -pitch/180*PI+cum_yaw;    // delta between initial‐zeroed pitch & current pitch
      // Send out gamma over Serial1 (ASCII float, 3 decimals, newline)
      //Serial1.print(-lastM3, 3);
      Serial1.print(-lastM3+0.43, 3);
      Serial1.print('\n');
    }

    // Use lastM1 / lastM2 to drive the two gimbal motors
    float desired1 = lastM2 + 1.27;

    // Clamp motor1 target
    if (desired1 < 0.27) desired1 = 0.27;
    if (desired1 > 2.27) desired1 = 2.27;

    float desired2 = atan(-tan((PI - lastM1)) * cos(lastM2)) + 4.34;
    // Clamp motor2 target
    if (desired2 < 3.34) desired2 = 3.34;
    if (desired2 > 5.34) desired2 = 5.34;

    // If receiver on the other end sends back a reply over Serial1, print it (optional)

    // Run FOC & control and drive motors to the “zeroed” setpoints
    motor1.loopFOC();
    motor2.loopFOC();
    command.run();
    motor1.move(desired2);
    motor2.move(desired1);

    if (now - lastLogTime >= 10 && (fabs(pi_yaw) > 0 || fabs(pi_pitch) > 0)) {  // Log every 20ms (50Hz)
  lastLogTime = now;
  Serial.print("T:"); Serial.print(now);
  Serial.print("  InputY:"); Serial.print(pi_yaw, 3);
  Serial.print("  InputP:"); Serial.println(pi_pitch, 3);
  }
  }
}

