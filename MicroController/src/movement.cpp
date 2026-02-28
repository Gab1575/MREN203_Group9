#include "movement.h"

void movement::startup() {
  pinMode(EA, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(EB, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I4, OUTPUT);
}

double movement::LeftPID(double setpoint, double current, double dt) {
  double error = setpoint - current;
    integral_left += error * dt;
  if (integral_left > LEFT_MAX_INTEGRAL) integral_left = LEFT_MAX_INTEGRAL; 
  if (integral_left < -LEFT_MAX_INTEGRAL) integral_left = -LEFT_MAX_INTEGRAL;
  
  double derivative = (error - prev_err_left) / dt;
  prev_err_left = error;

  double pid_output = (LEFT_KP * error) + (LEFT_KI * integral_left) + (LEFT_KD * derivative);
  
  double ff_output = calculateFeedforward(setpoint, calibrateFeedforward_L);
  
  return ff_output + pid_output;
}

double movement::RightPID(double setpoint, double current, double dt) {
    double error = setpoint - current;
        
    integral_right += error * dt;
    if (integral_right > RIGHT_MAX_INTEGRAL) integral_right = RIGHT_MAX_INTEGRAL; 
    if (integral_right < -RIGHT_MAX_INTEGRAL) integral_right = -RIGHT_MAX_INTEGRAL;
  
    double derivative = (error - prev_err_right) / dt;
    prev_err_right = error;
    
  double pid_output = (RIGHT_KP * error) + (RIGHT_KI * integral_right) + (RIGHT_KD * derivative);
  
  double ff_output = calculateFeedforward(setpoint, calibrateFeedforward_R);
  
  return ff_output + pid_output;
}

void movement::move(double targetW, double targetV, double dt, double Lencoder, double Rencoder, double actual_W) {
  
  double max_step = MAX_ACCELERATION * dt;

  if (targetV > current_target_v + max_step) {
    current_target_v += max_step; // Accelerate smoothly forward
  } 
  else if (targetV < current_target_v - max_step) {
    current_target_v -= max_step; // Accelerate smoothly backward (or brake)
  } 
  else {
    current_target_v = targetV;   // We reached the target, hold steady
  }

  double left = LeftPID(current_target_v, Lencoder, dt);
  double right = RightPID(current_target_v, -Rencoder, dt);

  if (left > 255) left = 255;
  else if (left < -255) left = -255;

  if (right > 255) right = 255;
  else if (right < -255) right = -255;

  unsigned long current_time = millis();
  
  if (abs(left) > 0 && abs(left)<MIN_PWM){
    double duty_cycle = abs(left) / MIN_PWM;
    double on_time = duty_cycle * MACRO_WINDOW_MS;
    
    if (current_time - macro_timer_L >= MACRO_WINDOW_MS) {
      macro_timer_L = current_time; 
    }

    if (current_time - macro_timer_L < on_time) {
      int active_pwm = (left > 0) ? MIN_PWM : -MIN_PWM;   //forward or backward
      run(active_pwm, 0); 
    } else {
      run(0, 0);
    }
    
  } 
  else {
    macro_timer_L = current_time; 
    run(left, 0);
  }
  
  if (abs(right) > 0 && abs(right)<MIN_PWM){
    double duty_cycle = abs(right) / MIN_PWM;
    double on_time = duty_cycle * MACRO_WINDOW_MS;
    
    if (current_time - macro_timer_R >= MACRO_WINDOW_MS) {
      macro_timer_R = current_time; 
    }

    if (current_time - macro_timer_R < on_time) {
      int active_pwm = (right > 0) ? MIN_PWM : -MIN_PWM;   //forward or backward
      run(active_pwm, 1); 
    } else {
      run(0, 1);
    }
    
  } 
  else {
    macro_timer_R = current_time; 
    run(right, 1);
  }
  
}

void movement::run(int PWM, bool side) {
  if (side == 0) { // Left side
    if (PWM > 0) {
      digitalWrite(I1, HIGH);
      digitalWrite(I2, LOW);
      analogWrite(EA, PWM);
    } else if (PWM < 0) {
      digitalWrite(I1, LOW);
      digitalWrite(I2, HIGH);
      analogWrite(EA, -PWM);
    } else {
      analogWrite(EA, 0); // Stop left motor
    }
  } else { // Right side
    if (PWM > 0) {
      digitalWrite(I3, HIGH);
      digitalWrite(I4, LOW);
      analogWrite(EB, PWM);
    } else if (PWM < 0) {
      digitalWrite(I3, LOW);
      digitalWrite(I4, HIGH);
      analogWrite(EB, -PWM);
    } else {
      analogWrite(EB, 0); // Stop right motor
    }
  }

}

void movement::calibrateFeedforward() {
  Serial.print("Calibrating Feedforward");
  encoders encCalibrate;
  double movingAVG_ARRAY_L[10] = {0};
  double movingAVG_ARRAY_R[10] = {0};
  double movingAVG_L = 0;
  double movingAVG_R = 0;
  for (int speed = MIN_PWM; speed <= 255; speed += 5){
    Serial.print("Calibrating at speed: ");
    Serial.println(speed);
    run(speed, 0); // Left motor forward
    run(speed, 1); // Right motor forward
    delay(500); // Allow time for the motors to reach the target speed
    
    for (int i = 0; i < 10; i++) {
      encCalibrate.run(); // Update encoder readings
      movingAVG_ARRAY_L[i] = encCalibrate.omega_L;
      movingAVG_ARRAY_R[i] = -encCalibrate.omega_R;
      delay(50); // Sample every 10ms within the macro window
    }

    // Calculate moving average for left and right encoders
    movingAVG_L = movingAVG_ARRAY_L[0]+
                  movingAVG_ARRAY_L[1]+
                  movingAVG_ARRAY_L[2]+
                  movingAVG_ARRAY_L[3]+
                  movingAVG_ARRAY_L[4]+
                  movingAVG_ARRAY_L[5]+
                  movingAVG_ARRAY_L[6]+
                  movingAVG_ARRAY_L[7]+
                  movingAVG_ARRAY_L[8]+
                  movingAVG_ARRAY_L[9];
    movingAVG_L /= 10;

    movingAVG_R = movingAVG_ARRAY_R[0]+
                  movingAVG_ARRAY_R[1]+
                  movingAVG_ARRAY_R[2]+
                  movingAVG_ARRAY_R[3]+
                  movingAVG_ARRAY_R[4]+
                  movingAVG_ARRAY_R[5]+
                  movingAVG_ARRAY_R[6]+
                  movingAVG_ARRAY_R[7]+
                  movingAVG_ARRAY_R[8]+
                  movingAVG_ARRAY_R[9];
    movingAVG_R /= 10;

    calibrateFeedforward_L.insert({movingAVG_L, speed});
    calibrateFeedforward_R.insert({movingAVG_R, speed});
  }

  run(0, 0); // Left motor forward
  run(0, 1); // Right motor forward

  Serial.println("\n// ==========================================");
  Serial.println("// COPY AND PASTE CALIBRATION MAPS BELOW");
  Serial.println("// ==========================================\n");

  // Print Left Map
  Serial.println("std::map<double, int> calibrateFeedforward_L = {");
  for (auto const& pair : calibrateFeedforward_L) {
    Serial.print("  {");
    Serial.print(pair.first);
    Serial.print(", ");
    Serial.print(pair.second, 4);
    Serial.println("},");
  }
  Serial.println("};\n");

  // Print Right Map
  Serial.println("std::map<double, int> calibrateFeedforward_R = {");
  for (auto const& pair : calibrateFeedforward_R) {
    Serial.print("  {");
    Serial.print(pair.first);
    Serial.print(", ");
    Serial.print(pair.second, 4); 
    Serial.println("},");
  }
  Serial.println("};\n");
  
  Serial.println("// ==========================================");
  Serial.println("// END OF CALIBRATION MAPS");
  Serial.println("// UPDATE MAPS IN movement.h AND THEN UPLOAD/REBOOT HEARSHEY");
  Serial.println("// ==========================================");

  while(true){
    // Infinite loop to prevent the robot from doing anything else after calibration
  };
}

double movement::calculateFeedforward(double target, const std::map<double, int>& calibrationMap) {
  
  double absTarget = abs(target);
  if (calibrationMap.empty() || absTarget == 0) {
    return 0.0;
  }

  auto upper = calibrationMap.lower_bound(absTarget);   
  if(upper == calibrationMap.end()) {
    if (target < 0) {
      return -255; //return max PWM if target exceeds range
    }
    else return 255; 
  }
  if (upper == calibrationMap.begin()) {
    if (target < 0) {
      return -upper->second; //return min PWM if target is below range
    }
    else return upper->second; 
  }
  if (upper->first == absTarget) {
    if (target < 0) {
      return -upper->second; //exact match
    }
    else return upper->second; 
  }

  auto lower = upper; 
  --lower; // Get the largest key less than target

  // Linear interpolation
  if(target < 0) {
    return -(lower->second + (upper->second - lower->second) * ((absTarget - lower->first) / (upper->first - lower->first)));
  }
  else {
  return lower->second + (upper->second - lower->second) * ((absTarget - lower->first) / (upper->first - lower->first));
  }
}