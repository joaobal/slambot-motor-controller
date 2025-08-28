/*
 * ESP32-WROVER Dual Motor Control with DRV8833 and Quadrature Encoders
 * UART Command Control + Dual PID Speed Control (RPM)
 *
 * Command Format over Serial2 (JetsonSerial): "M1,M2\n"
 *   where M1 and M2 are TARGET RPMs (can be negative), e.g. "150,-120"
 * The controller will regulate each wheel's speed to the requested RPM.
 */

#include <Arduino.h>
#include <PID_v1.h>

HardwareSerial JetsonSerial(2);

// ===== Motor Driver Pins (DRV8833)
const int MOTOR_A_IN1 = 32;
const int MOTOR_A_IN2 = 33;
const int MOTOR_B_IN1 = 18;
const int MOTOR_B_IN2 = 19;

// ===== Encoder Pins
const int ENCODER_A_PHASE_A = 25;
const int ENCODER_A_PHASE_B = 26;
const int ENCODER_B_PHASE_A = 27;
const int ENCODER_B_PHASE_B = 14;

// ===== PWM Configuration
const int PWM_FREQUENCY   = 1000; // 1kHz
const int PWM_RESOLUTION  = 8;    // 8-bit (0..255)
const int PWM_CHANNEL_A1  = 0;
const int PWM_CHANNEL_A2  = 1;
const int PWM_CHANNEL_B1  = 2;
const int PWM_CHANNEL_B2  = 3;

// ===== Encoder variables
volatile long encoder_A_count = 0;
volatile long encoder_B_count = 0;
volatile bool encoder_A_direction = true; // true = forward, false = reverse
volatile bool encoder_B_direction = true;
volatile byte encoder_A_last_state = 0;
volatile byte encoder_B_last_state = 0;

// ===== Speed calculation
unsigned long last_time = 0;
long last_encoder_A_count = 0;
long last_encoder_B_count = 0;
float motor_A_rpm = 0;
float motor_B_rpm = 0;

// Timers
unsigned long last_time_led = 0;

// DFRobot spec
const int   PULSES_PER_REVOLUTION      = 960;
const float SPEED_CALCULATION_INTERVAL = 100; // ms

// ===== UART command processing
String inputString = "";
bool stringComplete = false;

// ======= PID control (per-motor speed control in RPM) =======
double rpm_A_input = 0.0;     // |measured RPM| for A
double rpm_A_set   = 0.0;     // |target RPM| for A
double pid_A_output = 0.0;    // PWM magnitude [0..255]

double rpm_B_input = 0.0;     // |measured RPM| for B
double rpm_B_set   = 0.0;     // |target RPM| for B
double pid_B_output = 0.0;    // PWM magnitude [0..255]

// PID gains (start with a PI; set Kd=0, tune Kp then Ki)
double Kp_A = 1.5, Ki_A = 5.0, Kd_A = 0.0;
double Kp_B = 1.0, Ki_B = 4.0, Kd_B = 0.0;

PID pidA(&rpm_A_input, &pid_A_output, &rpm_A_set, Kp_A, Ki_A, Kd_A, DIRECT);
PID pidB(&rpm_B_input, &pid_B_output, &rpm_B_set, Kp_B, Ki_B, Kd_B, DIRECT);

// Setpoints (signed RPM)
int target_A_rpm = 0;
int target_B_rpm = 0;

// Test changes: zig-zag RPM
unsigned long cycle_time = 5000;  // Time to complete one full cycle (5 seconds for up and down)
int direction = 1;                // Direction flag (1: increasing, -1: decreasing)
float max_rpm = 150.0;            // Max RPM in either direction (positive and negative)
unsigned long cycle_start_time = 0; // Track when the cycle started
float target_rpm = 0.0;           // Target RPM value

// Function declarations
void setupPWM();
void setupEncoders();
void motors_control(int pwmA_signed, int pwmB_signed);
void set_motor_speed(int signed_pwm, int channel1, int channel2);
void calculate_speeds();
void IRAM_ATTR encoder_A_ISR();
void IRAM_ATTR encoder_B_ISR();
void processUARTCommand();
void parseMotorCommand(String command);

void setup() {
  Serial.begin(115200);                         // USB debug
  JetsonSerial.begin(115200, SERIAL_8N1, 23, 21); // RX=23, TX=21

  setupPWM();
  setupEncoders();

  // PID setup
  pidA.SetSampleTime((int)SPEED_CALCULATION_INTERVAL);
  pidB.SetSampleTime((int)SPEED_CALCULATION_INTERVAL);
  pidA.SetOutputLimits(0, 255); // PWM magnitude
  pidB.SetOutputLimits(0, 255);
  pidA.SetMode(AUTOMATIC);
  pidB.SetMode(AUTOMATIC);

  // Stop motors initially
  motors_control(0, 0);

  last_time = millis();
  inputString.reserve(200);

  Serial.println("Setup complete. PID speed control ready.");
  Serial.println("Send target RPM as: M1,M2 (e.g., 150,-120)");
}

void loop() {
  // Process UART commands (updates target_A_rpm / target_B_rpm)
  processUARTCommand();

  // // Test changes
  // if (millis() - last_time_led >= 10000) {
  //   target_B_rpm = 150;
  //   last_time_led = millis();
  // } else if (millis() - last_time_led >= 5000) {
  //   target_B_rpm = 0;
  // }

  target_B_rpm = update_rpm_target();

  // Update measured speeds and run PIDs every SPEED_CALCULATION_INTERVAL
  if (millis() - last_time >= SPEED_CALCULATION_INTERVAL) {
    last_time = millis();

    // 1) Update measured RPMs from encoders
    calculate_speeds();

    // 2) Prepare PID inputs/targets in magnitude (direction handled separately)
    rpm_A_input = fabs(motor_A_rpm);
    rpm_B_input = fabs(motor_B_rpm);

    rpm_A_set = fabs((double)target_A_rpm);
    rpm_B_set = fabs((double)target_B_rpm);

    // 3) Compute PID outputs (PWM magnitudes 0..255)
    pidA.Compute();
    pidB.Compute();

    // 4) Apply signed PWM: sign comes from target RPM sign
    int pwmA_signed = (target_A_rpm >= 0) ? (int)pid_A_output : -(int)pid_A_output;
    int pwmB_signed = (target_B_rpm >= 0) ? (int)pid_B_output : -(int)pid_B_output;

    motors_control(pwmA_signed, pwmB_signed);

    // 5) Periodic status
    static int status_counter = 0;
    if (++status_counter >= 2) {
      status_counter = 0;

      // To Jetson
      JetsonSerial.print("A tgt/meas/PWM: ");
      JetsonSerial.print(target_A_rpm); JetsonSerial.print(" / ");
      JetsonSerial.print(motor_A_rpm, 1); JetsonSerial.print(" / ");
      JetsonSerial.print(pwmA_signed);

      JetsonSerial.print(" | B tgt/meas/PWM: ");
      JetsonSerial.print(target_B_rpm); JetsonSerial.print(" / ");
      JetsonSerial.print(motor_B_rpm, 1); JetsonSerial.print(" / ");
      JetsonSerial.println(pwmB_signed);

      // To USB debug
      //Serial.print("A tgt/meas/PWM: ");
      //Serial.print(target_A_rpm); Serial.print(",");
      //Serial.print(motor_A_rpm, 1); Serial.print("\n");
      //Serial.print(pwmA_signed);

      // Serial.print(target_A_rpm); Serial.print(",");
      // Serial.print(motor_A_rpm, 1); Serial.print(",");
      Serial.print(target_B_rpm); Serial.print(",");
      Serial.print(motor_B_rpm, 1); Serial.print("\n");

      // Serial.print(" | B tgt/meas/PWM: ");
      // Serial.print(target_B_rpm); Serial.print(" / ");
      // Serial.print(motor_B_rpm, 1); Serial.print(" / ");
      // Serial.println(pwmB_signed);
    }
  }

  delay(2);
}

int update_rpm_target() {
  unsigned long current_time = millis();

  // Calculate ramp fraction within the current cycle (0 to 1)
  float ramp_fraction = float(current_time - cycle_start_time) / cycle_time;

  // If we have reached the end of the current ramp (either up or down), flip the direction
  if (ramp_fraction >= 1.0) {
    cycle_start_time = current_time;  // Reset cycle start time
    direction *= -1;                  // Change direction: 1 to -1 or -1 to 1
    ramp_fraction = 1.0;              // Ensure ramp_fraction stays at 1.0
  }

  // Update the target RPM
  target_rpm = direction * max_rpm * ramp_fraction;
  return target_rpm;
  // Debugging output to monitor target RPM
  //Serial.print("Target RPM: ");
  //Serial.println(target_rpm);
}

void processUARTCommand() {
  while (JetsonSerial.available()) {
    char c = (char)JetsonSerial.read();
    // echo to USB for debugging
    Serial.write(c);

    if (c == '\n' || c == '\r') {
      if (inputString.length() > 0) stringComplete = true;
    } else {
      inputString += c;
    }
  }

  if (stringComplete) {
    parseMotorCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
}

void parseMotorCommand(String command) {
  command.trim();
  int comma = command.indexOf(',');
  if (comma <= 0 || comma >= command.length() - 1) {
    JetsonSerial.println("Invalid format. Use: M1,M2  (RPM, can be negative)");
    return;
  }

  String s1 = command.substring(0, comma);
  String s2 = command.substring(comma + 1);

  int rpm1 = s1.toInt();
  int rpm2 = s2.toInt();

  // reasonable limits (adjust to your mechanics)
  rpm1 = constrain(rpm1, -1000, 1000);
  rpm2 = constrain(rpm2, -1000, 1000);

  target_A_rpm = rpm1;
  target_B_rpm = rpm2;

  // Optional: clear integrators when changing direction hard (helps tuning)
  // if ((rpm1 == 0) || (signbit(rpm1) != signbit((int)motor_A_rpm))) pidA.SetMode(MANUAL), pidA.SetMode(AUTOMATIC);
  // if ((rpm2 == 0) || (signbit(rpm2) != signbit((int)motor_B_rpm))) pidB.SetMode(MANUAL), pidB.SetMode(AUTOMATIC);

  Serial.print("New targets RPM -> A: ");
  Serial.print(target_A_rpm);
  Serial.print(", B: ");
  Serial.println(target_B_rpm);
}

void setupPWM() {
  ledcSetup(PWM_CHANNEL_A1, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_A2, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B1, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B2, PWM_FREQUENCY, PWM_RESOLUTION);

  ledcAttachPin(MOTOR_A_IN1, PWM_CHANNEL_A1);
  ledcAttachPin(MOTOR_A_IN2, PWM_CHANNEL_A2);
  ledcAttachPin(MOTOR_B_IN1, PWM_CHANNEL_B1);
  ledcAttachPin(MOTOR_B_IN2, PWM_CHANNEL_B2);

  Serial.println("PWM channels configured");
}

void setupEncoders() {
  pinMode(ENCODER_A_PHASE_A, INPUT_PULLUP);
  pinMode(ENCODER_A_PHASE_B, INPUT_PULLUP);
  pinMode(ENCODER_B_PHASE_A, INPUT_PULLUP);
  pinMode(ENCODER_B_PHASE_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PHASE_A), encoder_A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PHASE_A), encoder_B_ISR, CHANGE);

  Serial.println("Encoder interrupts configured");
}

// signed PWM helper: positive=forward (CH1), negative=reverse (CH2)
void set_motor_speed(int signed_pwm, int ch_fwd, int ch_rev) {
  int pwm = constrain(abs(signed_pwm), 0, 255);
  if      (signed_pwm > 0) { ledcWrite(ch_fwd, pwm); ledcWrite(ch_rev, 0); }
  else if (signed_pwm < 0) { ledcWrite(ch_fwd, 0);   ledcWrite(ch_rev, pwm); }
  else                     { ledcWrite(ch_fwd, 0);   ledcWrite(ch_rev, 0); }
}

void motors_control(int pwmA_signed, int pwmB_signed) {
  set_motor_speed(pwmA_signed, PWM_CHANNEL_A1, PWM_CHANNEL_A2);
  set_motor_speed(pwmB_signed, PWM_CHANNEL_B1, PWM_CHANNEL_B2);
}

void calculate_speeds() {
  float dt = SPEED_CALCULATION_INTERVAL / 1000.0f;

  long pulse_diff_A = encoder_A_count - last_encoder_A_count;
  long pulse_diff_B = encoder_B_count - last_encoder_B_count;

  // RPM = (pulses / dt) * 60 / PPR
  motor_A_rpm = (pulse_diff_A / dt) * 60.0f / PULSES_PER_REVOLUTION;
  motor_B_rpm = -(pulse_diff_B / dt) * 60.0f / PULSES_PER_REVOLUTION;

  last_encoder_A_count = encoder_A_count;
  last_encoder_B_count = encoder_B_count;
}

void IRAM_ATTR encoder_A_ISR() {
  byte phase_A = digitalRead(ENCODER_A_PHASE_A);
  byte phase_B = digitalRead(ENCODER_A_PHASE_B);

  if (encoder_A_last_state != phase_A) {
    if (phase_A == HIGH) encoder_A_direction = (phase_B == LOW);
    else                 encoder_A_direction = (phase_B == HIGH);

    if (encoder_A_direction) encoder_A_count++;
    else                     encoder_A_count--;

    encoder_A_last_state = phase_A;
  }
}

void IRAM_ATTR encoder_B_ISR() {
  byte phase_A = digitalRead(ENCODER_B_PHASE_A);
  byte phase_B = digitalRead(ENCODER_B_PHASE_B);

  if (encoder_B_last_state != phase_A) {
    if (phase_A == HIGH) encoder_B_direction = (phase_B == LOW);
    else                 encoder_B_direction = (phase_B == HIGH);

    if (encoder_B_direction) encoder_B_count++;
    else                     encoder_B_count--;

    encoder_B_last_state = phase_A;
  }
}

// Utilities (optional)
void reset_encoders() {
  encoder_A_count = 0; encoder_B_count = 0;
  last_encoder_A_count = 0; last_encoder_B_count = 0;
}
