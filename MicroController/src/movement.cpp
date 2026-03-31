#include "movement.h"

void movement::startup() {
  pinMode(EA, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(EB, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I4, OUTPUT);
}

double movement::AngularPID(double setpoint, double current, double dt) {

  double error = setpoint - current;
  integral_angular += error * dt; 
  if (integral_angular > ANGULAR_MAX_INTEGRAL) integral_angular = ANGULAR_MAX_INTEGRAL;
  if (integral_angular < -ANGULAR_MAX_INTEGRAL) integral_angular = -ANGULAR_MAX_INTEGRAL;
  
  double derivative = (error - prev_err_angular) / dt;
  prev_err_angular = error;
  
  double correction = (ANGULAR_KP * error) + (ANGULAR_KI * integral_angular) + (ANGULAR_KD * derivative);
  
  return correction;
}

double movement::LeftPID(double setpoint, double current, double dt) {
  double errorL = setpoint - current;
    integral_left += errorL * dt;
  if (integral_left > LEFT_MAX_INTEGRAL) integral_left = LEFT_MAX_INTEGRAL; 
  if (integral_left < -LEFT_MAX_INTEGRAL) integral_left = -LEFT_MAX_INTEGRAL;
  
  double derivativeL = (errorL - prev_err_left) / dt;
  prev_err_left = errorL;

  double pid_outputL = (LEFT_KP * errorL) + (LEFT_KI * integral_left) + (LEFT_KD * derivativeL);
  
  double ff_outputL = calculateFeedforward(setpoint, calibrateFeedforward_L);
  
  return ff_outputL + pid_outputL;
}

double movement::RightPID(double setpoint, double current, double dt) {
  
  double errorR = setpoint - current;
        
  integral_right += errorR * dt;
    if (integral_right > RIGHT_MAX_INTEGRAL) integral_right = RIGHT_MAX_INTEGRAL; 
    if (integral_right < -RIGHT_MAX_INTEGRAL) integral_right = -RIGHT_MAX_INTEGRAL;
  
  double derivativeR = (errorR - prev_err_right) / dt;
  
  prev_err_right = errorR;
    
  double pid_outputR = (RIGHT_KP * errorR) + (RIGHT_KI * integral_right) + (RIGHT_KD * derivativeR);
  
  double ff_outputR = calculateFeedforward(setpoint, calibrateFeedforward_R);
  
  return ff_outputR + pid_outputR;
}

void movement::move(double targetW, double targetV, double dt, double Lencoder, double Rencoder, double actual_W) {
  // ==========================================
  // 0. THE HARD STOP
  // If the joystick is completely released, wipe all PID memory and stop dead.
  // ==========================================
  if (targetV == 0.0 && targetW == 0.0) {
    integral_angular = 0;
    integral_left = 0;
    integral_right = 0;
    current_target_v = 0.0;
    
    run(0, 0); // Cut power to Left
    run(0, 1); // Cut power to Right
    return;    // Exit the function immediately!
  }

  // 1. Acceleration Limiter (m/s)
  double max_step = MAX_ACCELERATION * dt;
  if (targetV > current_target_v + max_step) {
    current_target_v += max_step; 
  } else if (targetV < current_target_v - max_step) {
    current_target_v -= max_step; 
  } else {
    current_target_v = targetV; 
  }

  // 2. Angular PID Correction (rad/s)
  double w_correction = AngularPID(targetW, actual_W, dt);
  double corrected_W = targetW + w_correction;

  // 3. Kinematics: Calculate wheel targets in meters per second (m/s)
  double target_vel_L_ms = current_target_v + ((corrected_W * TRACK_WIDTH) / 2.0);
  double target_vel_R_ms = current_target_v - ((corrected_W * TRACK_WIDTH) / 2.0);

  // 4. Conversion: m/s to rad/s (using 0.0625m wheel radius)
  double target_vel_L_rad = target_vel_L_ms / 0.0625;
  double target_vel_R_rad = target_vel_R_ms / 0.0625;

  // 5. PID & Feedforward: Input rad/s, Output PWM
  double left_pwm = LeftPID(target_vel_L_rad, Lencoder, dt);
  double right_pwm = RightPID(target_vel_R_rad, Rencoder, dt);

  // 6. Proportional PWM Clamping (Maintains turn radius if max power is exceeded)
  double abs_L = fabs(left_pwm);
  double abs_R = fabs(right_pwm);
  double max_pwm = (abs_L > abs_R) ? abs_L : abs_R; // Find the highest requested power
  
  if (max_pwm > 255.0) {
    double scale = 255.0 / max_pwm; // Calculate the shrink ratio (e.g., 0.85)
    left_pwm *= scale;              // Shrink both equally to preserve the turn!
    right_pwm *= scale;
  }

  // 7. Execution: Macro-Pulsing & Normal Drive
  unsigned long current_time = millis();
  
  // LEFT MOTOR
  if (abs(left_pwm) > 0 && abs(left_pwm) < MIN_PWM) {
    double duty_cycleL = fabs(left_pwm) / (double)MACRO_PWM; 
    double on_timeL = duty_cycleL * MACRO_WINDOW_MS;
    
    if (current_time - macro_timer_L >= MACRO_WINDOW_MS) macro_timer_L = current_time; 
    
    if (current_time - macro_timer_L <= on_timeL) {
      int active_pwm = (left_pwm > 0) ? MACRO_PWM : -MACRO_PWM;
      run(active_pwm, 0); 
    } else {
      run(0, 0);
    }
  } else {
    macro_timer_L = current_time; 
    run(left_pwm, 0);
  }
  
  // RIGHT MOTOR
  if (abs(right_pwm) > 0 && abs(right_pwm) < MIN_PWM) {
    double duty_cycleR = fabs(right_pwm) / (double)MACRO_PWM; 
    double on_timeR = duty_cycleR * MACRO_WINDOW_MS;
    
    if (current_time - macro_timer_R >= MACRO_WINDOW_MS) macro_timer_R = current_time; 
    
    if (current_time - macro_timer_R <= on_timeR) {
      int active_pwm = (right_pwm > 0) ? MACRO_PWM : -MACRO_PWM;
      run(active_pwm, 1); 
    } else {
      run(0, 1);
    }
  } else {
    macro_timer_R = current_time; 
    run(right_pwm, 1);
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
  Serial.print("Calibrating Feedforward\n");
  encoders encCalibrate;
  
  calibrateFeedforward_L.clear();
  calibrateFeedforward_R.clear();

  // Start all the way from 5, going up to 255
  for (int speed = 5; speed <= 255; speed += 5){
    Serial.print("Calibrating at target power: ");
    Serial.println(speed);

    double encoderSum_L = 0;
    double encoderSum_R = 0;
    int sampleCount = 0;

    unsigned long startTime = millis();
    unsigned long lastSampleTime = millis();

    // Run this specific speed test for 1.5 seconds
    while (millis() - startTime < 1500) {
      unsigned long current_time = millis();

      // ==========================================
      // 1. DRIVE THE MOTORS (Non-blocking)
      // ==========================================
      if (speed < MIN_PWM) {
        // Macro-Pulsing Mode for low speeds
        double duty_cycle = (double)speed / MACRO_PWM; 
        double on_time = duty_cycle * MACRO_WINDOW_MS;
        
        // Left
        if (current_time - macro_timer_L >= MACRO_WINDOW_MS) macro_timer_L = current_time; 
        if (current_time - macro_timer_L <= on_time) run(MACRO_PWM, 0); 
        else run(0, 0);
        
        // Right
        if (current_time - macro_timer_R >= MACRO_WINDOW_MS) macro_timer_R = current_time; 
        if (current_time - macro_timer_R <= on_time) run(MACRO_PWM, 1); 
        else run(0, 1);
      } 
      else {
        // Continuous power for normal speeds
        run(speed, 0);
        run(speed, 1);
      }

      // ==========================================
      // 2. READ THE ENCODERS
      // ==========================================
      encCalibrate.run(); // Keep updating encoder ticks constantly

      // Only start recording averages AFTER the first 500ms to let motors get to speed
      if (current_time - startTime > 500) {
        
        // Take a snapshot every 50ms
        if (current_time - lastSampleTime >= 50) {
          encoderSum_L += encCalibrate.omega_L;
          encoderSum_R += -encCalibrate.omega_R;
          sampleCount++;
          lastSampleTime = current_time;
        }
      }
      
    } // End of 1.5 second test loop

    // Calculate the true physical average for this speed
    double movingAVG_L = (sampleCount > 0) ? (encoderSum_L / sampleCount) : 0.0;
    double movingAVG_R = (sampleCount > 0) ? (encoderSum_R / sampleCount) : 0.0;

    calibrateFeedforward_L.insert({movingAVG_L, speed});
    calibrateFeedforward_R.insert({movingAVG_R, speed});
  }

  // Turn motors off when completely done
  run(0, 0); 
  run(0, 1);

  Serial.println("\n// ==========================================");
  Serial.println("// COPY AND PASTE CALIBRATION MAPS BELOW");
  Serial.println("// ==========================================\n");

  // Print Left Map
  Serial.println("std::map<double, int> calibrateFeedforward_L = {");
  for (auto const& pair : calibrateFeedforward_L) {
    Serial.print("  {");
    Serial.print(pair.first);
    Serial.print(", ");
    Serial.print(pair.second);
    Serial.println("},");
  }
  Serial.println("};\n");

  // Print Right Map
  Serial.println("std::map<double, int> calibrateFeedforward_R = {");
  for (auto const& pair : calibrateFeedforward_R) {
    Serial.print("  {");
    Serial.print(pair.first);
    Serial.print(", ");
    Serial.print(pair.second); 
    Serial.println("},");
  }
  Serial.println("};\n");
  
  Serial.println("// ==========================================");
  Serial.println("// END OF CALIBRATION MAPS");
  Serial.println("// UPDATE MAPS IN movement.h AND THEN UPLOAD/REBOOT HEARSHEY");
  Serial.println("// ==========================================");

  while(true){
    
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