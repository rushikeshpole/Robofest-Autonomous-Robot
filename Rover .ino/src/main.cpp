#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <DFRobot_QMC5883.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

TinyGPSPlus gps;

HardwareSerial SerialGPS(1);
#define PIN 48
#define NUMPIXELS 1
#define BRIGHTNESS 64 // Adjust brightness (0-255)
DFRobot_QMC5883 compass(&Wire, 0x0D);

const int GPS_TX_PIN = 17; // Connect to ESP32 RX2
const int GPS_RX_PIN = 18; // Connect to ESP32 TX2

// Define OLED dimensions and address
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDRESS 0x3C

// Create an OLED display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

// ===== PID structures & control variables (insert near other globals) =====
struct PID
{
    float kp = 0.0;
    float ki = 0.0;
    float kd = 0.0;
    float integrator = 0.0;
    float prev_error = 0.0;
    float integrator_min = -1000.0;
    float integrator_max = 1000.0;
};

PID velocityPID; // controls forward velocity (linearVelocityX)
PID yawPID;      // controls heading (yaw from IMU)

float desired_vx = 0.0; // m/s (set via FUNC_MOTION / set_bot_motion)
float desired_wz = 0.0; // rad/s or deg/s depending on your convention (we'll use rad/s*1000 scaling like python side)
unsigned long lastControlMillis = 0;
const unsigned long CONTROL_DT_MS = 25; // control loop period ~ 25ms (matches UPDATE_INTERVAL)

float max_pwm = 255.0; // adjust to your driver
float deadband_pwm = 10.0;

float clampf(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

float pid_update(PID &pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;
    pid.integrator += error * dt;
    pid.integrator = clampf(pid.integrator, pid.integrator_min, pid.integrator_max);
    float derivative = 0.0;
    if (dt > 0.0)
        derivative = (error - pid.prev_error) / dt;
    float output = pid.kp * error + pid.ki * pid.integrator + pid.kd * derivative;
    pid.prev_error = error;
    return output;
}

// ===== 2-DOF Camera Servo Control =====
#define PAN_SERVO_PIN   13
#define TILT_SERVO_PIN  14

#define PAN_MIN_ANGLE   0
#define PAN_MAX_ANGLE   180
#define TILT_MIN_ANGLE  0
#define TILT_MAX_ANGLE  180

const int SERVO_FREQ = 50;           // 50 Hz (SG90)
const int SERVO_RESOLUTION = 16;     // 16-bit PWM
const int SERVO_CHANNEL_PAN  = 4;
const int SERVO_CHANNEL_TILT = 5;

float pan_angle  = 90.0;  // default center
float tilt_angle = 90.0;

// Convert angle → duty cycle for ledcWrite()
uint32_t angleToDuty(float angle) {
    float min_us = 500.0;   // 0.5 ms
    float max_us = 2500.0;  // 2.5 ms
    float us = min_us + (max_us - min_us) * (angle / 180.0f);
    return (uint32_t)((us / 20000.0f) * 65535.0f); // 20 ms period
}

void setupServos() {
    ledcSetup(SERVO_CHANNEL_PAN,  SERVO_FREQ, SERVO_RESOLUTION);
    ledcSetup(SERVO_CHANNEL_TILT, SERVO_FREQ, SERVO_RESOLUTION);
    ledcAttachPin(PAN_SERVO_PIN,  SERVO_CHANNEL_PAN);
    ledcAttachPin(TILT_SERVO_PIN, SERVO_CHANNEL_TILT);

    // Center both servos on startup
    ledcWrite(SERVO_CHANNEL_PAN,  angleToDuty(pan_angle));
    ledcWrite(SERVO_CHANNEL_TILT, angleToDuty(tilt_angle));
}

struct GPSData
{
    double latitude;
    double longitude;
    double altitude;
    double speed;
    int satellites;
    int hour;
    int minute;
    int second;
    int day;
    int month;
    int year;
    bool locationValid;
    bool timeValid;
};

int16_t gx = 0;
int16_t gy = 0;
int16_t gz = 0;
int16_t ax = 0;
int16_t ay = 0;
int16_t az = 0;
int16_t mx = 0;
int16_t my = 0;
int16_t mz = 0;

int16_t roll = 0;
int16_t pitch = 0;
int16_t yaw = 0;

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#define HEAD 0xFF
#define DEVICE_ID 0xFC
#define COMPLEMENT (257 - DEVICE_ID)
#define FUNC_REPORT_SPEED 0x0A
#define FUNC_REPORT_MPU_RAW 0x0B
#define FUNC_REPORT_IMU_ATT 0x0C
#define FUNC_REPORT_ENCODER 0x0D
#define FUNC_MOTION 0x12
#define FUNC_DRIVE 0x16
#define FUNC_REPORT_GPS 0x17
#define FUNC_MANIPULATOR 0x18
#define FUNC_SET_MOTOR_PID 0x13
#define FUNC_SET_YAW_PID 0x14
#define FUNC_UART_SERVO 0x20
#define FUNC_PWM_SERVO_ALL 0x04

#define ENCODER_LEFT1_A 47  // GPIO1  - Interrupt capable, input pullup
#define ENCODER_LEFT1_B 19  // GPIO2  - Interrupt capable, input pullup
#define ENCODER_RIGHT1_A 20 // GPIO3  - Interrupt capable, input pullup
#define ENCODER_RIGHT1_B 21 // GPIO4  - Interrupt capable, input pullup

// Motor control pins - Selected GPIO pins with PWM capability
#define Mtr_Left 1     // GPIO35 - Digital output
#define Mtr_Left_pwm 2 // GPIO36 - PWM capable
#define Mtr_Right 3     // GPIO37 - Digital output
#define Mtr_Right_pwm 4 // GPIO38 - PWM capable

#define WHEEL_DIAMETER 0.075 // Diameter of the wheel in meters
#define WHEEL_BASE 0.26      // Distance between the wheels in meters (adjust for your robot)
#define PULSES_PER_REV 138   // Pulses per revolution of the encoder

MPU6050 mpu(0x68);

#define EARTH_GRAVITY_MS2 9.80665 // m/s2
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

Quaternion q;        // [w, x, y, z]         Quaternion container
VectorInt16 aa;      // [x, y, z]            Accel sensor measurements
VectorInt16 gg;      // [x, y, z]            Gyro sensor measurements
VectorInt16 aaWorld; // [x, y, z]            World-frame accel sensor measurements
VectorInt16 ggWorld; // [x, y, z]            World-frame gyro sensor measurements
VectorFloat gravity; // [x, y, z]            Gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

volatile long encoder_m1 = 0, encoder_m2 = 0, encoder_m3 = 0, encoder_m4 = 0;
float vx = 0, vy = 0, wz = 0;
const float ENCODER_TICKS_PER_REV = 138.0;
const float UPDATE_INTERVAL = 0.025;
volatile long leftPulseCount = 0;
volatile long rightPulseCount = 0;
volatile int leftDirection = 1;  // 1 = forward, -1 = backward
volatile int rightDirection = 1; // 1 = forward, -1 = backward
float linearVelocityX = 0.0;
float angularVelocity = 0.0;
float distanceLeft = 0.0;     // Distance covered by the left wheel
float distanceRight = 0.0;    // Distance covered by the right wheel
float distanceCoveredX = 0.0; // Average distance along the X-axis
unsigned long lastUpdateTime = 0;

void displayGPSData(const GPSData &data)
{
    display.clearDisplay();
    display.setCursor(0, 5);

    if (data.locationValid)
    {
        display.print("Lat: ");
        display.println(data.latitude, 6);
        display.print("Lon: ");
        display.println(data.longitude, 6);
    }
    else
    {
        display.println("Location not valid");
    }

    display.display();
}

void IRAM_ATTR handleLeft1Encoder()
{
    int channelA = digitalRead(ENCODER_LEFT1_A);
    int channelB = digitalRead(ENCODER_LEFT1_B);

    // Determine direction based on quadrature logic
    if (channelA == channelB)
    {
        leftDirection = 1; // Forward
    }
    else
    {
        leftDirection = -1; // Backward
    }

    // Update pulse count based on direction
    leftPulseCount += leftDirection;
}

void IRAM_ATTR handleRight1Encoder()
{
    int channelA = digitalRead(ENCODER_RIGHT1_A);
    int channelB = digitalRead(ENCODER_RIGHT1_B);

    // Determine direction based on quadrature logic
    if (channelA == channelB)
    {
        rightDirection = 1; // Forward
    }
    else
    {
        rightDirection = -1; // Backward
    }

    // Update pulse count based on direction
    rightPulseCount += rightDirection;
}

void setMotion(float vx1, float vy1, float wz1)
{
    // Apply the motion logic here
    // vx, vy, wz values can be used to control motor speeds
    // Serial.print("Setting motion: ")
    if (!digitalRead(46))
    {
        pixels.setPixelColor(0, pixels.Color(255, 255, 255));
        pixels.show();
        delay(10);
        if (vx1 > 0)
        {
            digitalWrite(Mtr_Left, HIGH);
            analogWrite(Mtr_Left_pwm, 80);
        }
    }
    else
    {
        pixels.setPixelColor(0, pixels.Color(255, 0, 0));
        pixels.show();
        delay(10);
        analogWrite(Mtr_Right, 0);
        analogWrite(Mtr_Right_pwm, 0);
    }
    display.clearDisplay();
    display.setCursor(0, 5);
    display.print("vx=");
    display.println(vx1);
    display.print("vy=");
    display.println(vy1);
    display.print("wz=");
    display.println(wz1);
    display.display();
    // Set motion
    // Example: Map the velocities to motor speeds
    // Assuming you have functions to control motors
    // setMotorSpeeds(vx, vy, wz);
}

void processDriveMotor(int16_t drive_mtr, int16_t swerve_mtr, int16_t drive_mtr_pwm,
                       int16_t swerve_mtr_pwm)
{
    // Implement motor control logic here
    // Serial.print("Drive Motor: ");
    // Serial.print(drive_mtr);
    // Serial.print(",Swerve Motor: ");
    // Serial.print(swerve_mtr);
    // Serial.print(",Drive Motor PWM: ");
    // Serial.print(drive_mtr_pwm);
    // Serial.print(",Swerve Motor PWM: ");
    // Serial.print(swerve_mtr_pwm);
    // Serial.print(",Swift Turn: ");
    // Serial.print(swift_turn);
    // Serial.print(",Bogie Motor: ");
    // Serial.println(bogie_mtr);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Drive: ");
    display.println(drive_mtr);
    display.print("Drive PWM: ");
    display.println(drive_mtr_pwm);
    display.print("Swerve : ");
    display.println(swerve_mtr);
    display.print("Swerve PWM: ");
    display.println(swerve_mtr_pwm);

    if (drive_mtr > 0)
    {

        digitalWrite(Mtr_Left, HIGH);
        analogWrite(Mtr_Left_pwm, drive_mtr_pwm);
        display.println("Forward");
    }
    if (drive_mtr < 0)
    {

        digitalWrite(Mtr_Right, LOW);
        analogWrite(Mtr_Right_pwm, drive_mtr_pwm);
        display.println("Backward");
    }
    if (drive_mtr == 0)
    {

        analogWrite(Mtr_Left_pwm, 0);
        display.println("No Change Drive");
    }
    if (swerve_mtr > 0)
    {

        digitalWrite(Mtr_Right, HIGH);
        analogWrite(Mtr_Right_pwm, swerve_mtr_pwm);
        display.println("Rightward");
    }
    if (swerve_mtr < 0)
    {

        digitalWrite(Mtr_Right, LOW);
        analogWrite(Mtr_Right_pwm, swerve_mtr_pwm);
        display.println("Leftward");
    }
    if (swerve_mtr == 0)
    {

        analogWrite(Mtr_Right_pwm, 0);
        display.println("No change Swerve");
    }
    display.display();
    // Add logic to control motors as required
}

// void processManipulatorMotor(int16_t base, int16_t angle, int16_t arm, int16_t claw, int16_t pwm, int16_t forward, int16_t backward)
// {
//     display.clearDisplay();
//     display.setCursor(0, 0);
//     display.print("Base : ");
//     display.println(base);
//     display.print("Angle : ");
//     display.println(angle);
//     display.print("Arm : ");
//     display.println(arm);
//     display.print("Claw : ");
//     display.println(claw);
//     display.print("Pwm : ");
//     display.println(pwm);
//     display.print("Forward : ");
//     display.println(forward);
//     display.print("Backward : ");
//     display.println(backward);

//     if ((base == 1) && (forward == 1))
//     {
//         digitalWrite(Manipulator_base_dir, HIGH);
//         analogWrite(Manipulator_base_pwm, pwm);
//         display.println("Forward Base");
//     }
//     if ((base == 1) && (backward == 1))
//     {
//         digitalWrite(Manipulator_base_dir, LOW);
//         analogWrite(Manipulator_base_pwm, pwm);
//         display.println("Backward Base");
//     }
//     if ((arm == 1) && (forward == 1))
//     {
//         digitalWrite(Manipulator_arm_dir, HIGH);
//         analogWrite(Manipulator_arm_pwm, pwm);
//         display.println("Forward Arm");
//     }
//     if ((arm == 1) && (backward == 1))
//     {
//         digitalWrite(Manipulator_arm_dir, LOW);
//         analogWrite(Manipulator_arm_pwm, pwm);
//         display.println("Backward Arm");
//     }
//     if ((angle == 1) && (forward == 1))
//     {
//         digitalWrite(Manipulator_angle_dir, HIGH);
//         analogWrite(Manipulator_angle_pwm, pwm);
//         display.println("Forward Angle");
//     }
//     if ((angle == 1) && (backward == 1))
//     {
//         digitalWrite(Manipulator_angle_dir, LOW);
//         analogWrite(Manipulator_angle_pwm, pwm);
//         display.println("Backward Angle");
//     }
//     if ((claw == 1) && (forward == 1))
//     {
//         digitalWrite(Manipulator_claw_dir, HIGH);
//         analogWrite(Manipulator_claw_pwm, pwm);
//         display.println("Forward Claw");
//     }
//     if ((claw == 1) && (backward == 1))
//     {
//         digitalWrite(Manipulator_claw_dir, LOW);
//         analogWrite(Manipulator_claw_pwm, pwm);
//         display.println("Backward Claw");
//     }
//     if ((forward == 0) && (backward == 0))
//     {
//         analogWrite(Manipulator_claw_pwm, 0);
//         analogWrite(Manipulator_arm_pwm, 0);
//         analogWrite(Manipulator_base_pwm, 0);
//         analogWrite(Manipulator_angle_pwm, 0);
//     }

//     display.display();
//     // Add logic to control motors as required
// }

void runControl()
{
    unsigned long now = millis();
    static unsigned long last = 0;
    unsigned long dt_ms = now - last;
    if (dt_ms < CONTROL_DT_MS)
        return;
    float dt = dt_ms / 1000.0f;
    last = now;

    // Use measured linearVelocityX and yaw (convert yaw to same units as desired_wz)
    // Note: in your code yaw variable is in ypr*RAD_TO_DEG * 10000 scale earlier, but you also report yaw in IMU packet send.
    // For heading control, it's usually better to use angularVelocity (computed from encoders) or fused yaw rate.
    float measured_vx = linearVelocityX; // m/s
    float measured_wz = angularVelocity; // rad/s (we computed angularVelocity earlier using wheel speeds)

    // Velocity PID output: translate to forward throttle
    float vel_output = pid_update(velocityPID, desired_vx, measured_vx, dt);
    // Heading PID output: translate to differential correction
    float yaw_output = pid_update(yawPID, desired_wz, measured_wz, dt);

    // Combine outputs to produce left & right motor commands
    // Simple mix: forward + correction, where correction subtracts from left and adds to right (or vice versa for sign convention)
    float left_cmd = vel_output - yaw_output;
    float right_cmd = vel_output + yaw_output;

    // Map to PWM range (assuming vel_output/yaw_output are in a range that expects direct mapping to PWM)
    // You may need to tune a scaling factor to map PID output to PWM units (0..255)
    float scale = 1.0; // try 1.0 initially; tune if outputs are too big/small
    int left_pwm = (int)clampf(left_cmd * scale, -max_pwm, max_pwm);
    int right_pwm = (int)clampf(right_cmd * scale, -max_pwm, max_pwm);

    // Convert to your processDriveMotor convention (drive_mtr, swerve_mtr, drive_mtr_pwm, swerve_mtr_pwm)
    // I assume you drive left/right via two channels; here we approximate:
    int drive_dir_left = (left_pwm >= 0) ? 1 : -1;
    int drive_dir_right = (right_pwm >= 0) ? 1 : -1;
    int drive_pwm_left = abs(left_pwm);
    int drive_pwm_right = abs(right_pwm);

    // Here we call processDriveMotor — it expects aggregated values; if you have separate left/right motor APIs,
    // replace these calls with direct motor writes. For now we'll send average drive_mtr and swerve as differential.
    // Convert left/right into a forward (drive_mtr) and a swerve (turn) approximation:
    int forward_dir = ((drive_dir_left + drive_dir_right) >= 0) ? 1 : -1;
    int avg_pwm = (drive_pwm_left + drive_pwm_right) / 2;
    int turn_val = (drive_pwm_right - drive_pwm_left); // >0 means right faster => turn right

    // Use processDriveMotor to apply the commands (this function sets pins and PWM)
    processDriveMotor(forward_dir, 0, avg_pwm, abs(turn_val));

    // Debug
    Serial.printf("CTRL dt=%.3f vx_set=%.3f vx_meas=%.3f vel_out=%.3f yaw_out=%.3f L=%d R=%d\n",
                  dt, desired_vx, measured_vx, vel_output, yaw_output, left_pwm, right_pwm);
}

GPSData parseGPSData()
{
    GPSData data;

    data.locationValid = gps.location.isValid();
    data.timeValid = gps.time.isValid();

    if (data.locationValid)
    {
        data.latitude = gps.location.lat();
        data.longitude = gps.location.lng();
        data.altitude = gps.altitude.meters();
        data.speed = gps.speed.kmph();
        data.satellites = gps.satellites.value();
    }
    else
    {
        data.latitude = 0.0;
        data.longitude = 0.0;
        data.altitude = 0.0;
        data.speed = 0.0;
        data.satellites = 0;
    }

    if (data.timeValid)
    {
        data.hour = gps.time.hour();
        data.minute = gps.time.minute();
        data.second = gps.time.second();
        data.day = gps.date.day();
        data.month = gps.date.month();
        data.year = gps.date.year();
    }
    else
    {
        data.hour = 0;
        data.minute = 0;
        data.second = 0;
        data.day = 0;
        data.month = 0;
        data.year = 0;
    }

    return data;
}

void printGPSData(const GPSData &data)
{
    if (data.locationValid)
    {
        Serial.println("\n=== GPS Data ===");
        Serial.print("Latitude: ");
        Serial.println(data.latitude, 6);
        Serial.print("Longitude: ");
        Serial.println(data.longitude, 6);
        Serial.print("Altitude: ");
        Serial.print(data.altitude);
        Serial.println(" meters");
        Serial.print("Speed: ");
        Serial.print(data.speed);
        Serial.println(" km/h");
        Serial.print("Satellites: ");
        Serial.println(data.satellites);
    }
    else
    {
        Serial.println("Location data not valid");
    }

    if (data.timeValid)
    {
        Serial.print("Time: ");
        Serial.print(data.hour < 10 ? "0" : "");
        Serial.print(data.hour);
        Serial.print(":");
        Serial.print(data.minute < 10 ? "0" : "");
        Serial.print(data.minute);
        Serial.print(":");
        Serial.print(data.second < 10 ? "0" : "");
        Serial.println(data.second);

        Serial.print("Date: ");
        Serial.print(data.day < 10 ? "0" : "");
        Serial.print(data.day);
        Serial.print("/");
        Serial.print(data.month < 10 ? "0" : "");
        Serial.print(data.month);
        Serial.print("/");
        Serial.println(data.year);
    }
    else
    {
        Serial.println("Time data not valid");
    }
}

void processIncomingCommand()
{
    static uint8_t buffer[256];
    static uint8_t buffer_index = 0;
    static bool packet_started = false;

    while (Serial.available())
    {
        uint8_t byte = Serial.read();
        if (byte == HEAD && !packet_started)
        {
            // Start of a new packet
            packet_started = true;
            buffer_index = 0;
            buffer[buffer_index++] = byte;
        }
        else if (packet_started)
        {
            buffer[buffer_index++] = byte;

            // Check if we have enough data for length
            if (buffer_index >= 3)
            {
                uint8_t length = buffer[2];
                if (length > 252)
                { // Length out of valid range
                    // Serial.println("Error: Invalid packet length");
                    packet_started = false;
                    buffer_index = 0;
                    continue;
                }

                if (buffer_index >= length + 3)
                {
                    // Packet complete
                    uint8_t checksum = 0;
                    for (int i = 2; i < buffer_index - 2; i++)
                    {
                        checksum += buffer[i];
                    }

                    // display.clearDisplay();
                    // display.setCursor(0, 0);
                    // Print received data
                    // display.print("Received data: ");
                    // for (int i = 0; i < buffer_index - 1; i++)
                    // {
                    //     display.print(buffer[i]);
                    //     if (i < buffer_index - 1)
                    //         Serial.print(", ");
                    // }
                    // display.println();

                    // Print checksum details
                    uint8_t received_checksum = buffer[buffer_index - 2];
                    // display.print("Calculated checksum: ");
                    // display.print(checksum & 0xFF);
                    // display.print(" ,Received checksum: ");
                    // display.println(received_checksum);
                    // display.display();

                    if ((checksum & 0xFF) != received_checksum)
                    {
                        // Serial.println("Error: Checksum mismatch");
                    }
                    else
                    {
                        // Valid packet
                        uint8_t device_id = buffer[1];
                        uint8_t func_code = buffer[3];
                        uint8_t *data = buffer + 5;

                        if (device_id != DEVICE_ID)
                        {
                            // Serial.println("Error: Unsupported device ID");
                        }
                        else if (func_code == FUNC_MOTION)
                        {
                            int16_t vx = (data[1] << 8) | data[0];
                            int16_t vy = (data[3] << 8) | data[2];
                            int16_t wz = (data[5] << 8) | data[4];

                            float motion_vx = vx / 1000.0;
                            float motion_vy = vy / 1000.0;
                            float motion_wz = wz / 1000.0;
                            desired_vx = motion_vx;
                            desired_wz = motion_wz;
                            Serial.printf("Desired motion set: vx=%.3f wz=%.3f\n", desired_vx, desired_wz);

                            setMotion(motion_vx, motion_vy, motion_wz);
                            // Serial.write("Motion command processed successfully");
                        }
                        else if (func_code == FUNC_DRIVE)
                        {
                            // Extract drive motor control data
                            int16_t drive_mtr = (data[1] << 8) | data[0];
                            int16_t swerve_mtr = (data[3] << 8) | data[2];
                            int16_t drive_mtr_pwm = (data[5] << 8) | data[4];
                            int16_t swerve_mtr_pwm = (data[7] << 8) | data[6];
                            int16_t swift_turn = (data[9] << 8) | data[8];
                            int16_t bogie_mtr = (data[11] << 8) | data[10];

                            // Process motion commands
                            processDriveMotor(drive_mtr / 1000, swerve_mtr / 1000, drive_mtr_pwm,
                                              swerve_mtr_pwm);
                            // Serial.println("Drive motor command processed successfully");
                        }

                        else if (func_code == FUNC_MANIPULATOR)
                        {
                            // Extract drive motor control data
                            int16_t base = (data[1] << 8) | data[0];
                            int16_t angle = (data[3] << 8) | data[2];
                            int16_t arm = (data[5] << 8) | data[4];
                            int16_t claw = (data[7] << 8) | data[6];

                            int16_t pwm = (data[9] << 8) | data[8];
                            int16_t forward = (data[11] << 8) | data[10];
                            int16_t backward = (data[13] << 8) | data[12];

                            // Process motion commands
                            // processManipulatorMotor(base, angle, arm, claw, pwm, forward, backward);
                            // // Serial.println("Drive motor command processed successfully");
                        }
                        // inside the code block where func_code is available:
                        else if (func_code == FUNC_SET_MOTOR_PID)
                        {
                            // data[] is pointer to payload starting at buffer[5] as in your code
                            // Expect: pid_index (byte), kp (int16), ki (int16), kd (int16), state (byte)
                            uint8_t pid_index = data[0];
                            int16_t raw_kp = (data[2] << 8) | data[1];
                            int16_t raw_ki = (data[4] << 8) | data[3];
                            int16_t raw_kd = (data[6] << 8) | data[5];
                            // Convert back to float: python packs kp*1000 etc.
                            float kp = raw_kp / 1000.0f;
                            float ki = raw_ki / 1000.0f;
                            float kd = raw_kd / 1000.0f;
                            // Assign to velocity PID (we'll use PID index 1 -> velocity)
                            velocityPID.kp = kp;
                            velocityPID.ki = ki;
                            velocityPID.kd = kd;
                            // clear integrator to avoid bumps on change
                            velocityPID.integrator = 0.0;
                            velocityPID.prev_error = 0.0;
                            // Optionally report debug
                            Serial.printf("Set MOT PID: kp=%.3f ki=%.3f kd=%.3f idx=%d\n", kp, ki, kd, pid_index);
                        }
                        else if (func_code == FUNC_SET_YAW_PID)
                        {
                            uint8_t pid_index = data[0];
                            int16_t raw_kp = (data[2] << 8) | data[1];
                            int16_t raw_ki = (data[4] << 8) | data[3];
                            int16_t raw_kd = (data[6] << 8) | data[5];
                            float kp = raw_kp / 1000.0f;
                            float ki = raw_ki / 1000.0f;
                            float kd = raw_kd / 1000.0f;
                            yawPID.kp = kp;
                            yawPID.ki = ki;
                            yawPID.kd = kd;
                            yawPID.integrator = 0.0;
                            yawPID.prev_error = 0.0;
                            Serial.printf("Set YAW PID: kp=%.3f ki=%.3f kd=%.3f idx=%d\n", kp, ki, kd, pid_index);
                        }
                        else if (func_code == FUNC_UART_SERVO)
                        {
                            // Payload: [servo_id, angle]
                            uint8_t servo_id = data[0];
                            uint8_t angle = data[1];

                            if (servo_id == 1)
                            { // Pan
                                pan_angle = constrain(angle, PAN_MIN_ANGLE, PAN_MAX_ANGLE);
                                ledcWrite(SERVO_CHANNEL_PAN, angleToDuty(pan_angle));
                                Serial.printf("[Servo] Pan -> %d°\n", (int)pan_angle);
                            }
                            else if (servo_id == 2)
                            { // Tilt
                                tilt_angle = constrain(angle, TILT_MIN_ANGLE, TILT_MAX_ANGLE);
                                ledcWrite(SERVO_CHANNEL_TILT, angleToDuty(tilt_angle));
                                Serial.printf("[Servo] Tilt -> %d°\n", (int)tilt_angle);
                            }
                        }

                        else if (func_code == FUNC_PWM_SERVO_ALL)
                        {
                            // Payload: [id1, angle1, id2, angle2]
                            uint8_t id1 = data[0];
                            uint8_t angle1 = data[1];
                            uint8_t id2 = data[2];
                            uint8_t angle2 = data[3];

                            if (id1 == 1)
                            {
                                pan_angle = constrain(angle1, PAN_MIN_ANGLE, PAN_MAX_ANGLE);
                                ledcWrite(SERVO_CHANNEL_PAN, angleToDuty(pan_angle));
                                Serial.printf("[ServoAll] Pan -> %d°\n", (int)pan_angle);
                            }
                            if (id2 == 2)
                            {
                                tilt_angle = constrain(angle2, TILT_MIN_ANGLE, TILT_MAX_ANGLE);
                                ledcWrite(SERVO_CHANNEL_TILT, angleToDuty(tilt_angle));
                                Serial.printf("[ServoAll] Tilt -> %d°\n", (int)tilt_angle);
                            }
                        }

                        else
                        {
                            // Serial.println("Error: Unsupported function code");
                        }
                    }

                    // Reset buffer
                    packet_started = false;
                    buffer_index = 0;
                }
            }
        }
    }
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    delay(2000);
    // Initialize OLED display
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS))
    {
        Serial.println("SSD1306 allocation failed");
        for (;;)
            ; // Halt if initialization fails
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 3);
    display.print("Team SGGS");
    display.display();
    delay(2000);

    while (!compass.begin())
    {
        Serial.println("Could not find a valid 5883 sensor, check wiring!");
        delay(500);
    }

    SerialGPS.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    if (compass.isHMC())
    {
        Serial.println("Initialize HMC5883");
    }
    else if (compass.isQMC())
    {
        Serial.println("Initialize QMC5883");
    }
    else if (compass.isVCM())
    {
        Serial.println("Initialize VCM5883L");
    }
    delay(1000);

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    Serial.begin(115200);
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    Serial.println(F("Testing MPU6050 connection..."));
    if (mpu.testConnection() == false)
    {
        Serial.println("MPU6050 connection failed");
        while (true)
            ;
    }
    else
    {
        Serial.println("MPU6050 connection successful");
    }

    /* Initializate and configure the DMP*/
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    /* Supply your gyro offsets here, scaled for min sensitivity */
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);

    /* Making sure it worked (returns 0 if so) */
    if (devStatus == 0)
    {
        mpu.CalibrateAccel(6); // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateGyro(6);
        Serial.println("These are the Active offsets: ");
        mpu.PrintActiveOffsets();
        Serial.println(F("Enabling DMP...")); // Turning ON DMP
        mpu.setDMPEnabled(true);

        MPUIntStatus = mpu.getIntStatus();

        /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        DMPReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize(); // Get expected DMP packet size for later comparison
    }

    pinMode(ENCODER_LEFT1_A, INPUT_PULLUP);
    pinMode(ENCODER_LEFT1_B, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT1_A, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT1_B, INPUT_PULLUP);


    pinMode(Mtr_Left, OUTPUT);
    pinMode(Mtr_Left_pwm, OUTPUT);
    pinMode(Mtr_Right, OUTPUT);
    pinMode(Mtr_Right_pwm, OUTPUT);

    // pinMode(Manipulator_angle_dir, OUTPUT);
    // pinMode(Manipulator_angle_pwm, OUTPUT);
    // pinMode(Manipulator_arm_dir, OUTPUT);
    // pinMode(Manipulator_arm_pwm, OUTPUT);
    // pinMode(Manipulator_base_dir, OUTPUT);
    // pinMode(Manipulator_base_pwm, OUTPUT);
    // pinMode(Manipulator_claw_dir, OUTPUT);
    // pinMode(Manipulator_claw_pwm, OUTPUT);

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT1_A), handleLeft1Encoder, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT1_A), handleRight1Encoder, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT1_B), handleLeft1Encoder, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT1_B), handleRight1Encoder, RISING);

    lastUpdateTime = millis();
}

void sendPacket(uint8_t func_code, uint8_t *data, uint8_t data_len)
{
    uint8_t packet[256], packet_index = 0;
    packet[packet_index++] = HEAD;
    packet[packet_index++] = DEVICE_ID - 1;
    packet[packet_index++] = data_len + 2;
    packet[packet_index++] = func_code;

    uint8_t checksum = data_len + 2 + func_code;
    for (int i = 0; i < data_len; i++)
    {
        packet[packet_index++] = data[i];
        checksum += data[i];
    }
    packet[packet_index++] = checksum % 256;
    Serial.write(packet, packet_index);
    Serial.flush();
}

void loop()
{
    float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
    compass.setDeclinationAngle(declinationAngle);
    sVector_t mag = compass.readRaw();
    compass.getHeadingDegrees();
    mx = (mag.XAxis);
    my = (mag.YAxis);
    mz = (mag.ZAxis);

    if (gps.encode(SerialGPS.read()))
    {
        GPSData currentData = parseGPSData();
        displayGPSData(currentData);

        // Create GPS data packet
        uint8_t gps_data[32];
        memcpy(gps_data, &currentData.latitude, 8); // 8 bytes for double
        memcpy(gps_data + 8, &currentData.longitude, 8);
        memcpy(gps_data + 16, &currentData.altitude, 8);
        float speed_kmh = currentData.speed * 1.852; // Convert knots to km/h
        memcpy(gps_data + 24, &speed_kmh, 4);        // 4 bytes for float

        sendPacket(FUNC_REPORT_GPS, gps_data, 28); // Send 28 bytes
    }

    processIncomingCommand();

    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();

    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer))
    {

        mpu.dmpGetQuaternion(&q, FIFOBuffer);

        mpu.dmpGetGravity(&gravity, &q);

        mpu.dmpGetAccel(&aa, FIFOBuffer);
        mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);
        ax = (aaWorld.x * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2) * 1671.84;
        ay = (aaWorld.y * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2) * 1671.84;
        az = (aaWorld.z * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2) * 1671.84;

        mpu.dmpGetGyro(&gg, FIFOBuffer);
        mpu.dmpConvertToWorldFrame(&ggWorld, &gg, &q);
        gx = (ggWorld.x * mpu.get_gyro_resolution() * DEG_TO_RAD) * 3754.9;
        gy = (ggWorld.y * mpu.get_gyro_resolution() * DEG_TO_RAD) * 3754.9;
        gz = (ggWorld.z * mpu.get_gyro_resolution() * DEG_TO_RAD) * 3754.9;

        /* Display Euler angles in degrees */
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        yaw = (ypr[0] * RAD_TO_DEG) * 10000.0;
        pitch = (ypr[1] * RAD_TO_DEG) * 10000.0;
        roll = (ypr[2] * RAD_TO_DEG) * 10000.0;

        // Serial.print("ypr\t");
        // Serial.print(ypr[0] * RAD_TO_DEG);
        // Serial.print("\t");
        // Serial.print(ypr[1] * RAD_TO_DEG);
        // Serial.print("\t");
        // Serial.println(ypr[2] * RAD_TO_DEG);
    }
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastUpdateTime;

    static uint32_t prev_ms = millis();
    if (elapsedTime >= 1000)
    {
        float wheelCircumference = WHEEL_DIAMETER * PI;
        float leftRevolutions = leftPulseCount / (float)PULSES_PER_REV;
        float rightRevolutions = rightPulseCount / (float)PULSES_PER_REV;

        // Calculate RPM (with direction included)
        float leftRPM = leftRevolutions * (60000.0 / elapsedTime);
        float rightRPM = rightRevolutions * (60000.0 / elapsedTime);

        // Calculate angular velocities of the wheels
        float leftAngularVelocity = leftRPM * (2 * PI / 60); // Convert RPM to rad/s
        float rightAngularVelocity = rightRPM * (2 * PI / 60);

        // Calculate linear velocities for each wheel
        float leftLinearVelocity = leftAngularVelocity * (WHEEL_DIAMETER / 2.0);
        float rightLinearVelocity = rightAngularVelocity * (WHEEL_DIAMETER / 2.0);

        // Average the linear velocities to compute overall linear velocity along X
        linearVelocityX = (leftLinearVelocity + rightLinearVelocity) / 2.0;

        // Calculate angular velocity (rotation rate) based on differential velocities
        angularVelocity = (rightLinearVelocity - leftLinearVelocity) / WHEEL_BASE;

        // Calculate distances covered for each wheel
        distanceLeft += leftRevolutions * wheelCircumference * leftDirection;
        distanceRight += rightRevolutions * wheelCircumference * rightDirection;

        // Calculate average distance along X-axis
        distanceCoveredX = (distanceLeft + distanceRight) / 2.0;

        // Speed packet
        uint8_t speed_data[7];
        int16_t vx_mm = linearVelocityX * 1000, vy_mm = 0 * 1000, wz_mm = angularVelocity * 1000;
        memcpy(speed_data, &vx_mm, 2);
        memcpy(speed_data + 2, &vy_mm, 2);
        memcpy(speed_data + 4, &wz_mm, 2);
        speed_data[6] = 100; // Battery voltage
        sendPacket(FUNC_REPORT_SPEED, speed_data, 7);

        uint8_t mpu_data[18];
        memcpy(mpu_data, &gx, 2);
        memcpy(mpu_data + 2, &gy, 2);
        memcpy(mpu_data + 4, &gz, 2);
        memcpy(mpu_data + 6, &ax, 2);
        memcpy(mpu_data + 8, &ay, 2);
        memcpy(mpu_data + 10, &az, 2);
        memcpy(mpu_data + 12, &mx, 2);
        memcpy(mpu_data + 14, &my, 2);
        memcpy(mpu_data + 16, &mz, 2);
        sendPacket(FUNC_REPORT_MPU_RAW, mpu_data, 18);
        // IMU attitude
        uint8_t imu_data[6];
        memcpy(imu_data, &roll, 2);
        memcpy(imu_data + 2, &pitch, 2);
        memcpy(imu_data + 4, &yaw, 2);
        sendPacket(FUNC_REPORT_IMU_ATT, imu_data, 6);
        // Encoder data
        uint8_t encoder_data[16];
        noInterrupts();
        long e1 = 10, e2 = 20, e3 = 30, e4 = 40;
        interrupts();
        memcpy(encoder_data, &e1, 4);
        memcpy(encoder_data + 4, &e2, 4);
        memcpy(encoder_data + 8, &e3, 4);
        memcpy(encoder_data + 12, &e4, 4);
        sendPacket(FUNC_REPORT_ENCODER, encoder_data, 16);

        leftPulseCount = 0;
        rightPulseCount = 0;
        lastUpdateTime = currentTime;
    }
}
