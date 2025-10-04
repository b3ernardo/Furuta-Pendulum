/*
  ============================================
  UFMG FURUTA-Kit is placed under the MIT License
  Copyright (c) 2024 by GTI (UFMG)
  ============================================
  Hardware:
  * 1 ESP-WROOM-32 WEMOS D1 R32 development board
  * 1 Nidec 24-H
  * 1 Rotary Incremental Encoder
*/

// Include the encoder library for ESP32, which uses built-in hardware counters
#include <ESP32Encoder.h>

// Pin for the onboard ESP32 LED (blue LED on GPIO 2)
#define INTERNAL_LED 2

// PWM configuration for NIDEC motor driver
#define NIDEC_TIMER_BIT 8      // PWM resolution: 8-bit (0–255)
#define NIDEC_BASE_FREQ 20000  // PWM frequency: 20 kHz (to avoid audible noise)

// NIDEC motor control pins
#define BRAKE 18        // Brake pin (Start/Stop control)
#define NIDEC_PWM 19    // PWM output pin
#define DIR 23          // Direction pin (Forward/Reverse)
#define NCHA 13         // Motor encoder channel A
#define NCHB 5          // Motor encoder channel B
#define NIDEC_PWM_CH 1  // PWM channel used for ESP32 LEDC

// Encoder pins for the pendulum rod
#define RCHA 32  // Rod encoder channel A
#define RCHB 33  // Rod encoder channel B

// Encoder objects
ESP32Encoder NIDEC_ENC;  // Encoder for the NIDEC motor
ESP32Encoder ROD_ENC;    // Encoder for the pendulum rod

// Conversion rad to deg
const float RAD2DEG = 180.0f / PI;  // ≈ 57.2958

// Safety limits (in degrees)
const float BASE_LIMIT_DEG = 160.0f;  // Max angle for the NIDEC motor (±)
const float ROD_LIMIT_DEG = 20.0f;    // Max angle for the pendulum (±)

// Encoder counts per revolution (adjust according to your measurement!)
const float CPR_NIDEC = 400.0f;  // Motor encoder (100 PPR ×4 = 400)
const float CPR_ROD = 1440.0f;   // Rod encoder (360 PPR ×4 = 1440)

// Counts per radian (used to convert counts → radians)
const float CNT_PER_RAD_NIDEC = CPR_NIDEC / (2.0f * PI);  // ≈ 63.66
const float CNT_PER_RAD_ROD = CPR_ROD / (2.0f * PI);      // ≈ 229.18

// Control gains - LQR
float K1 = -11.8;
float K2 = 240.0;
float K3 = -7.2;
float K4 = 12.5;
// float K1 = -10.715;
// float K2 = 177.987;
// float K3 = -3.786;
// float K4 = 10.648;

// Variables to store encoder readings, PWM output, and accumulated states
float rod_position = 0;    // Current angular position of the pendulum rod (incremental encoder counts)
float nidec_position = 0;  // Current angular position of the NIDEC motor
float nidec_speed = 0;
float rod_speed = 0;
float pwm = 0;        // Control signal to be sent to the motor driver (PWM duty cycle)
int NIDEC_count = 0;  // Raw encoder count for NIDEC motor in the current sample
int ROD_count = 0;    // Raw encoder count for pendulum rod in the current sample

// Sampling time variables
float Ts = 0.01;        // Sampling time (control loop period) in seconds → 10 ms
float currentT = 0.0;   // Current loop timestamp (milliseconds converted to seconds)
float previousT = 0.0;  // Previous loop timestamp (used to calculate elapsed time)

// MAIN SETUP: Runs once on startup/reset
void setup() {
  Serial.begin(9600);  // Start Serial Monitor for debugging

  // Initialize motor and encoders
  NIDECsetup();
  ENCsetup();

  // Configure internal LED
  pinMode(INTERNAL_LED, OUTPUT);
  digitalWrite(INTERNAL_LED, HIGH);  // Turn on LED (indicates system is starting)

  // Wait until the rod encoder has moved enough counts before starting
  while (abs(ROD_count) < 600) {
    ROD_count = ROD_ENC.getCount();
  }

  // Blink LED for system ready indication
  delay(1000);
  digitalWrite(INTERNAL_LED, LOW);
  delay(1000);
  digitalWrite(INTERNAL_LED, HIGH);
  delay(1000);
  digitalWrite(INTERNAL_LED, LOW);

  // Reset encoder counters
  NIDEC_ENC.clearCount();
  ROD_ENC.clearCount();
}

// MAIN LOOP: Runs repeatedly after setup
void loop() {
  currentT = millis();  // Current time in ms

  // Run control loop every Ts seconds
  if ((currentT - previousT) / 1000.0 >= Ts) {
    previousT = currentT;

    // Safety: stop system if rod or motor goes beyond safe angle limits
    if (abs(nidec_position * RAD2DEG) < BASE_LIMIT_DEG && abs(rod_position * RAD2DEG) < ROD_LIMIT_DEG) {
      digitalWrite(BRAKE, HIGH);  // Enable motor brake

      // Read encoder increments (speed since last loop = position increment per sampling interval)
      nidec_speed = -NIDEC_ENC.getCount() / (Ts * CNT_PER_RAD_NIDEC);
      rod_speed = ROD_ENC.getCount() / (Ts * CNT_PER_RAD_ROD);

      // Reset encoders for next loop
      NIDEC_ENC.clearCount();
      ROD_ENC.clearCount();

      // Integrate speeds to get position and displacement
      nidec_position += nidec_speed * Ts;
      rod_position += rod_speed * Ts;

      // Compute control law
      pwm = -(K1 * nidec_position + K2 * rod_position + K3 * nidec_speed + K4 * rod_speed);

      // Sometimes the motor stops
      if (nidec_speed == 0) {
        pwm += random(8) - 4;
      }

      // Send command to motor
      MOTORcmd(pwm);
    } else {
      // If system goes out of bounds → stop motor and activate LED alarm
      pwm = 0;
      MOTORcmd(pwm);  // Stop motor
      digitalWrite(BRAKE, LOW);
      digitalWrite(INTERNAL_LED, HIGH);  // Activate LED alarm
    }
  }
}

// NIDEC MOTOR SETUP
void NIDECsetup() {
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH);  // Enable brake initially

  pinMode(DIR, OUTPUT);
  ledcSetup(NIDEC_PWM_CH, NIDEC_BASE_FREQ, NIDEC_TIMER_BIT);
  ledcAttachPin(NIDEC_PWM, NIDEC_PWM_CH);
  MOTORcmd(0);  // Start with motor stopped

  // Setup motor encoder
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  NIDEC_ENC.attachFullQuad(NCHA, NCHB);
  NIDEC_ENC.clearCount();
}

// ROD ENCODER SETUP
void ENCsetup() {
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  ROD_ENC.attachFullQuad(RCHA, RCHB);
  ROD_ENC.clearCount();
}

/*
  --------------------------------------------------------------------------------------------------
  | Control signal (sp) | DIR pin | PWM value (ledcWrite) | Motor result                           |
  |---------------------|---------|-----------------------|----------------------------------------|
  | -200                | LOW     | 200                   | Motor spins in direction A, high power |
  | -50                 | LOW     | 50                    | Motor spins in direction A, low power  |
  | 0                   | none    | 0                     | Motor stopped                          |
  | +50                 | HIGH    | 50                    | Motor spins in direction B, low power  |
  | +200                | HIGH    | 200                   | Motor spins in direction B, high power |
  --------------------------------------------------------------------------------------------------
*/

// MOTOR COMMAND FUNCTION: The code makes an adjustment to the PWM because the driver hardware interprets the inverted values
void MOTORcmd(int sp) {
  // Set direction
  if (sp < 0) {
    digitalWrite(DIR, LOW);
    sp = -sp;  // Make speed positive
  } else {
    digitalWrite(DIR, HIGH);
  }

  // Apply PWM signal (inverted due to motor driver configuration)
  ledcWrite(NIDEC_PWM_CH, int(sp > 255 ? 0 : 255 - sp));
}