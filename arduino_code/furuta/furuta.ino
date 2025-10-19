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

#define MAX_SAMPLES 3000  // Maximum number of data samples stored

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
float K1 = -0.5111;
float K2 = 14.1924;
float K3 = -0.3557;
float K4 = 0.9390;

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

// Variables used for data logging and transmission
float data[MAX_SAMPLES][7];  // Matrix storing up to 3000 samples, each with 7 measured or computed variables
float disturbance = 0;       // External disturbance applied to the system (e.g., pulse or step input)
float Tc = 20;               // Data acquisition period (ms) used to control when samples are sent via Serial
float currentTc = 0.0;       // Current timestamp used for timing control
float previousTc = 0.0;      // Previous timestamp used to determine when to send data
int count = 0;               // Sample counter used to track the number of stored measurements

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
  balance();
  // print();
}

// BALANCE CONTROL LOOP: Runs the main control algorithm for the Furuta pendulum
void balance() {
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

      // Integrate speed to get angular position
      nidec_position += nidec_speed * Ts;
      rod_position += rod_speed * Ts;

      // Compute control law (state feedback)
      float u = K1 * nidec_position + K2 * rod_position + K3 * nidec_speed + K4 * rod_speed;

      // Conversion to PWM
      pwm = -constrain(u * 21.25, -200, 200);

      // Apply small random correction if motor stalls
      if (nidec_speed == 0) {
        pwm += random(8) - 4;
      }

      // Apply a step-type disturbance after 21 seconds
      // if (currentT / 1000.0 > 21) {
      //   disturbance = 1 * 21.25;
      // }

      // Apply a pulse-type disturbance at t ∈ (21s, 22s)
      // if (currentT / 1000.0 > 21 && currentT / 1000.0 < 22) {
      //   disturbance = 1 * 21.25;
      // } else {
      //   disturbance = 0;
      // }

      // Send command to motor
      MOTORcmd(pwm + disturbance);
    } else {
      // If system goes out of bounds → stop motor and activate LED alarm
      pwm = 0;
      MOTORcmd(pwm);  // Stop motor
      digitalWrite(BRAKE, LOW);
      digitalWrite(INTERNAL_LED, HIGH);  // Activate LED alarm
    }

    // Data logging after 20 seconds of operation
    // if (currentT / 1000.0 > 20) {
    //   data[count][0] = currentT / 1000.0;  // Time (s)
    //   data[count][1] = nidec_position;     // Base angle (rad)
    //   data[count][2] = rod_position;       // Rod angle (rad)
    //   data[count][3] = nidec_speed;        // Base angular speed (rad/s)
    //   data[count][4] = rod_speed;          // Rod angular speed (rad/s)
    //   data[count][5] = pwm / 21.25;        // Normalized control signal
    //   data[count][6] = disturbance;        // Applied disturbance

    //   if (count < MAX_SAMPLES - 1) {
    //     count++;
    //   }
    // }
  }
}

// DATA PRINT FUNCTION: Sends all collected samples to Serial Monitor after 50s
void print() {
  currentTc = millis();

  // Print data every Tc seconds, only after 50 seconds of operation
  if ((currentTc - previousTc) / 1000.0 >= Tc && currentTc / 1000.0 > 50) {
    previousTc = currentTc;

    // Send data matrix (time, positions, speeds, control, disturbance)
    for (int i = 0; i < MAX_SAMPLES; i++) {
      for (int j = 0; j < 7; j++) {
        Serial.print(data[i][j], 6);  // Print with 6 decimal places

        if (j != 6) {
          Serial.print(" ");
        }
      }
      Serial.println();
    }

    Serial.println("===");  // End of data block marker
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