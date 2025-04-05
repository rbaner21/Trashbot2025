#include <Arduino.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>    // ESP32-compatible servo library
#include "driver/ledc.h"   // For LEDC configuration types and functions

/*
 * This sketch controls:
 *  - Two drive motors (BTS7960 drivers) for left/right drive.
 *  - One collector motor (L298N HW-095) for trash collection.
 *  - A controller via Bluepad32.
 *  - A servo (on pin 7) that is now controlled via the right trigger (R2) along with the collector.
 *
 * Controller Mapping:
 *  - A (Xbox)/Cross (PS): Engage system.
 *  - L1 (left bumper): Kill switch – stops motors, stops collector, and after a pause resets servo.
 *  - Left joystick Y: Controls the left drive motor.
 *  - Right joystick Y (inverted): Controls the right drive motor.
 *  - R2 (Right Trigger): When pressed:
 *       - If the collector is off, the servo goes down then (after a delay) the collector turns on.
 *       - If the collector is on, the collector turns off then (after a delay) the servo goes back up.
 *  - D-pad Down: Toggle collector motor direction.
 *
 * LED Indicators:
 *  - Red LED (pin 15): Always on.
 *  - Blue LED (pin 16): Solid if a controller is connected; blinks otherwise.
 *  - Green LED (pin 17): Blinks when the system is engaged.
 *
 * Motor PWM/LEDC (using low-speed timer 1):
 *  - Left drive forward: Channel 1 (pin 4)
 *  - Left drive reverse:  Channel 2 (pin 5)
 *  - Right drive forward: Channel 3 (pin 8)
 *  - Right drive reverse:  Channel 4 (pin 9)
 *  - Collector motor enable: Channel 5 (pin 10)
 *
 * Servo PWM/LEDC (using low-speed timer 0):
 *  - Servo is attached on pin 7 with a PWM frequency of ~50Hz.
 *    It auto-allocates a channel from TIMER_0 (typically channel 0).
 */

// -----------------------
// PIN DEFINITIONS
// -----------------------
const int LEFT_RPWM_PIN  = 4;   // Left drive forward PWM
const int LEFT_LPWM_PIN  = 5;   // Left drive reverse PWM
const int RIGHT_RPWM_PIN = 8;   // Right drive forward PWM
const int RIGHT_LPWM_PIN = 9;   // Right drive reverse PWM

const int COLLECTOR_ENA_PIN = 10;  // PWM for collector motor speed
const int COLLECTOR_IN1_PIN = 11;  // Collector motor direction control
const int COLLECTOR_IN2_PIN = 12;  // Collector motor direction control

const int RED_LED_PIN   = 15;  // Red LED: Always on
const int BLUE_LED_PIN  = 16;  // Blue LED: Controller connection status
const int GREEN_LED_PIN = 17;  // Green LED: Blinks when engaged

const int SERVO_PIN = 7;       // Servo control pin

// New limit switch definition
const int LIMIT_SWITCH_PIN = 18;  // Mechanical kill switch pin

// -----------------------
// Ultrasonic Sensor Definitions
// -----------------------
const int US_TRIG_PIN = 13;  // Ultrasonic sensor trigger
const int US_ECHO_PIN = 14;  // Ultrasonic sensor echo (with voltage divider for ESP32)

// -----------------------
// PWM SETTINGS
// -----------------------
const int PWM_FREQ = 20000;           // 20 kHz for motors
const int PWM_RESOLUTION = 8;         // 8-bit resolution (0–255)
const int MIN_PWM = 70;               // Minimum PWM value (to overcome inertia)
const int MAX_PWM = 255;              // Maximum PWM value
const int JOYSTICK_DEADZONE = 25;     // Joystick deadzone threshold

// Delay before servo resets during kill sequence or between servo and collector actions (in milliseconds)
const int KILL_SERVO_DELAY_MS = 500;

// Slew rate (ramp) configuration for drive motors
const int SPEED_STEP = 20;  // Change in PWM per loop iteration (adjust to your liking)
int currentLeftPWM = 0;     // Global current PWM for left drive (can be positive for forward, negative for reverse)
int currentRightPWM = 0;    // Global current PWM for right drive

// -----------------------
// LEDC CHANNEL ASSIGNMENTS for Motors (using TIMER_1, LOW_SPEED_MODE)
// -----------------------
const int LEFT_RPWM_CHANNEL  = 1;  // Left drive forward
const int LEFT_LPWM_CHANNEL  = 2;  // Left drive reverse
const int RIGHT_RPWM_CHANNEL = 3;  // Right drive forward
const int RIGHT_LPWM_CHANNEL = 4;  // Right drive reverse
const int COLLECTOR_ENA_CHANNEL = 5;  // Collector motor enable

// -----------------------
// BLUEPAD32 SETUP
// -----------------------
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Global flags and variables
bool engaged = false;
bool collectorEngaged = false;
bool collectorDirectionReversed = false;
const int COLLECTOR_SPEED = 200;
unsigned long lastBlueBlink = 0;
bool blueLedState = false;

// Servo global
Servo myServo;
// servoAtUp is true when the servo is at its "origin" position (180°, up).
bool servoAtUp = false;

// -----------------------
// HELPER FUNCTIONS
// -----------------------
bool anyControllerConnected() {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] && myControllers[i]->isConnected())
      return true;
  }
  return false;
}

// Helper function to immediately stop drive motors.
void immediateStopDriveMotors() {
  currentLeftPWM = 0;
  currentRightPWM = 0;
  ledcWrite((ledc_channel_t)LEFT_RPWM_CHANNEL, 0);
  ledcWrite((ledc_channel_t)LEFT_LPWM_CHANNEL, 0);
  ledcWrite((ledc_channel_t)RIGHT_RPWM_CHANNEL, 0);
  ledcWrite((ledc_channel_t)RIGHT_LPWM_CHANNEL, 0);
}

// Modified setDriveMotor with slew-rate limiting for left and right drives
// This function is used for both left and right drive motors.
void setDriveMotor(int rpwmChannel, int lpwmChannel, int speedVal) {
  int targetPWM = 0;
  // Calculate target PWM value based on joystick input.
  if (abs(speedVal) < JOYSTICK_DEADZONE) {
    targetPWM = 0;
  } else {
    int absVal = constrain(abs(speedVal), JOYSTICK_DEADZONE, 512);
    targetPWM = map(absVal, JOYSTICK_DEADZONE, 512, MIN_PWM, MAX_PWM);
    if (speedVal < 0) {
      targetPWM = -targetPWM;  // Negative for reverse
    }
  }
  
  // Apply slew-rate limiting separately for left and right drive.
  if (rpwmChannel == LEFT_RPWM_CHANNEL) {
    // Gradually update currentLeftPWM toward targetPWM.
    if (currentLeftPWM < targetPWM) {
      currentLeftPWM = min(currentLeftPWM + SPEED_STEP, targetPWM);
    } else if (currentLeftPWM > targetPWM) {
      currentLeftPWM = max(currentLeftPWM - SPEED_STEP, targetPWM);
    }
    // Drive the left motor.
    if (currentLeftPWM > 0) {
      ledcWrite((ledc_channel_t)rpwmChannel, currentLeftPWM);
      ledcWrite((ledc_channel_t)lpwmChannel, 0);
    } else if (currentLeftPWM < 0) {
      ledcWrite((ledc_channel_t)rpwmChannel, 0);
      ledcWrite((ledc_channel_t)lpwmChannel, -currentLeftPWM);
    } else {
      ledcWrite((ledc_channel_t)rpwmChannel, 0);
      ledcWrite((ledc_channel_t)lpwmChannel, 0);
    }
  } else if (rpwmChannel == RIGHT_RPWM_CHANNEL) {
    // Gradually update currentRightPWM toward targetPWM.
    if (currentRightPWM < targetPWM) {
      currentRightPWM = min(currentRightPWM + SPEED_STEP, targetPWM);
    } else if (currentRightPWM > targetPWM) {
      currentRightPWM = max(currentRightPWM - SPEED_STEP, targetPWM);
    }
    // Drive the right motor.
    if (currentRightPWM > 0) {
      ledcWrite((ledc_channel_t)rpwmChannel, currentRightPWM);
      ledcWrite((ledc_channel_t)lpwmChannel, 0);
    } else if (currentRightPWM < 0) {
      ledcWrite((ledc_channel_t)rpwmChannel, 0);
      ledcWrite((ledc_channel_t)lpwmChannel, -currentRightPWM);
    } else {
      ledcWrite((ledc_channel_t)rpwmChannel, 0);
      ledcWrite((ledc_channel_t)lpwmChannel, 0);
    }
  }
}

void setCollectorMotor(int speedVal) {
  if (speedVal > 0) {
    if (!collectorDirectionReversed) {
      digitalWrite(COLLECTOR_IN1_PIN, HIGH);
      digitalWrite(COLLECTOR_IN2_PIN, LOW);
    } else {
      digitalWrite(COLLECTOR_IN1_PIN, LOW);
      digitalWrite(COLLECTOR_IN2_PIN, HIGH);
    }
    ledcWrite((ledc_channel_t)COLLECTOR_ENA_CHANNEL, speedVal);
  } else {
    digitalWrite(COLLECTOR_IN1_PIN, LOW);
    digitalWrite(COLLECTOR_IN2_PIN, LOW);
    ledcWrite((ledc_channel_t)COLLECTOR_ENA_CHANNEL, 0);
  }
}

void updateStatusLEDs() {
  digitalWrite(RED_LED_PIN, HIGH);
  bool anyConnected = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i]) { anyConnected = true; break; }
  }
  if (anyConnected)
    digitalWrite(BLUE_LED_PIN, HIGH);
  else {
    if (millis() - lastBlueBlink > 500) {
      blueLedState = !blueLedState;
      lastBlueBlink = millis();
    }
    digitalWrite(BLUE_LED_PIN, blueLedState ? HIGH : LOW);
  }
  if (engaged) {
    static unsigned long lastGreenToggleTime = 0;
    static bool greenLedState = false;
    if (millis() - lastGreenToggleTime > 500) {
      greenLedState = !greenLedState;
      lastGreenToggleTime = millis();
    }
    digitalWrite(GREEN_LED_PIN, greenLedState ? HIGH : LOW);
  } else {
    digitalWrite(GREEN_LED_PIN, LOW);
  }
}

void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (!myControllers[i]) {
      Serial.printf("Controller connected at index %d\n", i);
      myControllers[i] = ctl;
      return;
    }
  }
  Serial.println("Controller connected but no free slot found.");
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("Controller disconnected from index %d\n", i);
      myControllers[i] = nullptr;
      return;
    }
  }
  Serial.println("Controller disconnected but not found.");
}

void processGamepad(ControllerPtr ctl) {
  // --- Kill Command (L1) ---
  if (ctl->l1()) {
    engaged = false;
    collectorEngaged = false;
    immediateStopDriveMotors();  // Immediately stop drive motors.
    setCollectorMotor(0);
    delay(KILL_SERVO_DELAY_MS);
    myServo.write(180);  // Return to origin (up)
    servoAtUp = true;
    Serial.println("L1 pressed: System disengaged, motors immediately stopped, servo reset (after delay).");
    return;
  }
  
  if (!engaged) {
    if (ctl->a()) {
      engaged = true;
      Serial.println("A pressed: System engaged.");
    } else {
      // When not engaged, ensure motors are stopped.
      setDriveMotor(LEFT_RPWM_CHANNEL, LEFT_LPWM_CHANNEL, 0);
      setDriveMotor(RIGHT_RPWM_CHANNEL, RIGHT_LPWM_CHANNEL, 0);
      setCollectorMotor(0);
      return;
    }
  }
  
  int leftSpeed = ctl->axisY();
  int rightSpeed = -ctl->axisRY();
  // Use the new slew-rate limited setDriveMotor for left and right drives.
  setDriveMotor(LEFT_RPWM_CHANNEL, LEFT_LPWM_CHANNEL, leftSpeed);
  setDriveMotor(RIGHT_RPWM_CHANNEL, RIGHT_LPWM_CHANNEL, rightSpeed);
  
  // --- D-pad Collector Direction Toggle (unchanged) ---
  static bool prevDpadDownCollector = false;
  bool currentDpadDownCollector = (ctl->dpad() & 0x02) != 0;
  if (currentDpadDownCollector && !prevDpadDownCollector) {
    collectorDirectionReversed = !collectorDirectionReversed;
    Serial.printf("Collector direction toggled: now %s\n", collectorDirectionReversed ? "Reversed" : "Normal");
  }
  prevDpadDownCollector = currentDpadDownCollector;
  
  // --- Unified Collector & Servo Control using Right Trigger (R2) ---
  static bool prevR2State = false;
  bool r2Pressed = ctl->r2();  // Using Bluepad32's r2() method for right trigger.
  if (r2Pressed && !prevR2State) {
    if (!collectorEngaged) {
      // Lower the servo then engage the collector
      if (servoAtUp) {
        myServo.write(0);
        servoAtUp = false;
        Serial.println("R2 pressed: Servo moved to 0° (down).");
      }
      delay(KILL_SERVO_DELAY_MS);
      collectorEngaged = true;
      setCollectorMotor(COLLECTOR_SPEED);
      Serial.println("R2 pressed: Collector engaged.");
    } else {
      // Disengage the collector then raise the servo
      collectorEngaged = false;
      setCollectorMotor(0);
      Serial.println("R2 pressed: Collector disengaged.");
      delay(KILL_SERVO_DELAY_MS);
      myServo.write(180);
      servoAtUp = true;
      Serial.println("R2 pressed: Servo moved to origin (up).");
    }
  }
  prevR2State = r2Pressed;
  
  setCollectorMotor(collectorEngaged ? COLLECTOR_SPEED : 0);
  
  Serial.printf("Drive: L=%d, R=%d | Collector: %s at %d\n",
                leftSpeed, rightSpeed,
                collectorEngaged ? (collectorDirectionReversed ? "Reversed" : "Normal") : "Off",
                collectorEngaged ? COLLECTOR_SPEED : 0);
}

void processControllers() {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] && myControllers[i]->isConnected() &&
        myControllers[i]->hasData() && myControllers[i]->isGamepad())
      processGamepad(myControllers[i]);
  }
}

// -----------------------
// Ultrasonic Sensor & Vibration Functionality (unchanged)
// -----------------------
void updateUltrasonicSensor() {
  digitalWrite(US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);
  
  unsigned long duration = pulseIn(US_ECHO_PIN, HIGH, 30000);
  float distance = duration * 0.0343 / 2.0;
  
  if (distance > 50 || duration == 0) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
      if (myControllers[i] && myControllers[i]->isConnected()) {
        myControllers[i]->playDualRumble(0, 0, 0, 0);
      }
    }
    return;
  }
  
  if (distance <= 10) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
      if (myControllers[i] && myControllers[i]->isConnected()) {
        myControllers[i]->playDualRumble(0, 255, 255, 50);
      }
    }
    return;
  }
  
  int pulsePeriod = map((int)distance, 10, 50, 200, 1000);
  int halfPeriod = pulsePeriod / 2;
  uint8_t durationParam = (halfPeriod > 255 ? 255 : halfPeriod);
  static unsigned long lastPulseTime = 0;
  static bool pulseOn = false;
  unsigned long now = millis();
  
  if (now - lastPulseTime >= (unsigned long)halfPeriod) {
    pulseOn = !pulseOn;
    lastPulseTime = now;
    if (pulseOn) {
      for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] && myControllers[i]->isConnected()) {
          myControllers[i]->playDualRumble(0, 255, 255, durationParam);
        }
      }
    } else {
      for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] && myControllers[i]->isConnected()) {
          myControllers[i]->playDualRumble(0, 0, 0, 0);
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Robot + Collector with LEDs, Bluepad32 (Separated Timers for Motors & Servo)");
  
  ledc_timer_config_t motorTimer;
  motorTimer.speed_mode = LEDC_LOW_SPEED_MODE;
  motorTimer.timer_num = LEDC_TIMER_1;
  motorTimer.duty_resolution = LEDC_TIMER_8_BIT;
  motorTimer.freq_hz = PWM_FREQ;
  motorTimer.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&motorTimer);
  
  ledc_channel_config_t channelConfig;
  channelConfig.duty = 0;
  channelConfig.speed_mode = LEDC_LOW_SPEED_MODE;
  channelConfig.timer_sel = LEDC_TIMER_1;
  channelConfig.intr_type = LEDC_INTR_DISABLE;
  channelConfig.hpoint = 0;
  
  channelConfig.channel = (ledc_channel_t)LEFT_RPWM_CHANNEL;
  channelConfig.gpio_num = LEFT_RPWM_PIN;
  ledc_channel_config(&channelConfig);
  
  channelConfig.channel = (ledc_channel_t)LEFT_LPWM_CHANNEL;
  channelConfig.gpio_num = LEFT_LPWM_PIN;
  ledc_channel_config(&channelConfig);
  
  channelConfig.channel = (ledc_channel_t)RIGHT_RPWM_CHANNEL;
  channelConfig.gpio_num = RIGHT_RPWM_PIN;
  ledc_channel_config(&channelConfig);
  
  channelConfig.channel = (ledc_channel_t)RIGHT_LPWM_CHANNEL;
  channelConfig.gpio_num = RIGHT_LPWM_PIN;
  ledc_channel_config(&channelConfig);
  
  channelConfig.channel = (ledc_channel_t)COLLECTOR_ENA_CHANNEL;
  channelConfig.gpio_num = COLLECTOR_ENA_PIN;
  ledc_channel_config(&channelConfig);
  
  pinMode(COLLECTOR_IN1_PIN, OUTPUT);
  pinMode(COLLECTOR_IN2_PIN, OUTPUT);
  
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  
  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);
  
  ledc_timer_config_t servoTimer;
  servoTimer.speed_mode = LEDC_LOW_SPEED_MODE;
  servoTimer.timer_num = LEDC_TIMER_0;
  servoTimer.duty_resolution = LEDC_TIMER_8_BIT;
  servoTimer.freq_hz = 50;
  servoTimer.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&servoTimer);
  
  myServo.attach(SERVO_PIN, 544, 2400);
  // Initialize servo at origin (upward position, 180°)
  myServo.write(180);
  servoAtUp = true;
  
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);
  
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* bdAddr = BP32.localBdAddress();
  Serial.printf("BD Addr: %02X:%02X:%02X:%02X:%02X:%02X\n",
                bdAddr[0], bdAddr[1], bdAddr[2],
                bdAddr[3], bdAddr[4], bdAddr[5]);
}

void loop() {
  bool dataUpdated = BP32.update();
  
  if (!anyControllerConnected()) {
    engaged = false;
    setDriveMotor(LEFT_RPWM_CHANNEL, LEFT_LPWM_CHANNEL, 0);
    setDriveMotor(RIGHT_RPWM_CHANNEL, RIGHT_LPWM_CHANNEL, 0);
    setCollectorMotor(0);
  }
  
  bool limitSwitchPressed = (digitalRead(LIMIT_SWITCH_PIN) == LOW);
  
  // Limit switch kill sequence: immediately stop drive motors.
  if (limitSwitchPressed) {
    if (engaged) {
      engaged = false;
      collectorEngaged = false;
      immediateStopDriveMotors();
      setCollectorMotor(0);
      delay(KILL_SERVO_DELAY_MS);
      myServo.write(180);
      servoAtUp = true;
      Serial.println("Limit switch pressed: System disengaged, motors immediately stopped, servo reset (after delay).");
    }
  } else {
    if (dataUpdated) {
      processControllers();
    }
  }
  
  updateStatusLEDs();
  
  if (engaged) {
    updateUltrasonicSensor();
  }
  
  delay(20);
}
