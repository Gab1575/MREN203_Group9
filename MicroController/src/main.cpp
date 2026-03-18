#include <Arduino.h>
#include "encoders.h"
#include "movement.h"
#include "IMU.h"

InternalIMU::IMUData imuData;
InternalIMU imu;
movement move;
encoders enc;

void readSerialCommands();
void sendDataToPi();


// --- ROBOT PHYSICAL CONSTANTS (from my_controllers.yaml) ---
const double WHEEL_RADIUS_MM = 60.0; 
const double TRACK_WIDTH_MM = 220.0; 

// --- ROS 2 TIMING VARIABLES ---
unsigned long last_time = 0;
const int LOOP_DELAY_MS = 20; // 50Hz loop

// --- ROS 2 COMMAND TARGETS (rad/s) ---
float cmd_fl = 0.0, cmd_rl = 0.0, cmd_fr = 0.0, cmd_rr = 0.0;

// --- ODOMETRY POSITIONS (radians) ---
float pos_l = 0.0, pos_r = 0.0;

void setup() {
  Serial.begin(115200);
  if(!Serial) {
    while (1); 
  }
  imu.startup();
  move.startup();
  enc.startup();
}

void loop() {
  readSerialCommands();

  unsigned long current_time = millis();
  if (current_time - last_time >= LOOP_DELAY_MS) {
    double dt = (current_time - last_time) / 1000.0; 
    last_time = current_time;

    // --- READ HARDWARE ---
    imuData = imu.read();
    enc.run();

    // --- CALCULATE POSITION ---
    // Accumulate total radians turned for odometry
    pos_l += (enc.omega_L * dt);
    pos_r += (enc.omega_R * dt);

    // --- KINEMATICS TRANSLATION (Wheel Speeds -> Chassis Speeds) ---
    // 1. Average the left and right sides (skid steer)
    double left_rad_per_sec = (cmd_fl + cmd_rl) / 2.0;
    double right_rad_per_sec = (cmd_fr + cmd_rr) / 2.0;

    // 2. Convert wheel angular velocity (rad/s) to linear velocity (mm/s)
    double v_left = left_rad_per_sec * WHEEL_RADIUS_MM;
    double v_right = right_rad_per_sec * WHEEL_RADIUS_MM;

    // 3. Calculate chassis linear (V) and angular (W) velocities
    double targetV = (v_left + v_right) / 2.0;
    double targetW = (v_right - v_left) / TRACK_WIDTH_MM;

    // --- EXECUTE MOVEMENT ---
    move.move(targetW, targetV, dt);

    // --- BLAST TELEMETRY ---
    sendDataToPi(); 
  }
}

// ==========================================================
// INCOMING COMMAND PARSER
// ==========================================================
void readSerialCommands() {
  static char buffer[64];
  static byte index = 0;

  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      buffer[index] = '\0'; 
      if (buffer[0] == 'C') {
        char* token = strtok(buffer, ",");
        if (token != NULL && token[0] == 'C') {
          token = strtok(NULL, ","); if(token) cmd_fl = atof(token);
          token = strtok(NULL, ","); if(token) cmd_rl = atof(token);
          token = strtok(NULL, ","); if(token) cmd_fr = atof(token);
          token = strtok(NULL, "\r\n"); if(token) cmd_rr = atof(token);
        }
      }
      index = 0; 
    } else if (index < 63) {
      buffer[index++] = c;
    }
  }
}

// ==========================================================
// TELEMETRY TRANSMITTER
// ==========================================================
void sendDataToPi() {
  // Output format MUST be: gyroZ, accelX, accelY, accelZ, pos_L, pos_R\n
  Serial.print(imuData.gyroZ, 4); Serial.print(","); 
  Serial.print(imuData.accelX, 4); Serial.print(",");
  Serial.print(imuData.accelY, 4); Serial.print(",");
  Serial.print(imuData.accelZ, 4); Serial.print(",");
  Serial.print(pos_l, 4); Serial.print(",");
  Serial.println(pos_r, 4); 
}