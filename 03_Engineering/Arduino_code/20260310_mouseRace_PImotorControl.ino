/*
 * PW3 Mouse PI Control
 * Group 1
 * Updated 14/03/2026
 *
 * Notes:
 * - Left side is defined as positive.
 * - Positive error means the mouse is too far right, so it should turn left.
 */

// -------------------- Pins -------------------- //
const int PUCL_PIN = A0;
const int PUCR_PIN = A1;
const int MOTOR_L_PIN = 5;
const int MOTOR_R_PIN = 6;

// -------------------- Motor settings -------------------- //
const int COAST_SPEED = 128;   // Base forward speed
const int TURN_SPEED  = 40;    // Hard turn amount
const int PWM_MIN = 0;
const int PWM_MAX = 255;

// -------------------- Control region -------------------- //
const int PID_RANGE_LEFT  = -50;
const int PID_RANGE_RIGHT =  50;
const int ERROR_DEADBAND  = 5;     // Ignore very small errors

// -------------------- Timing -------------------- //
unsigned long currentTime = 0;
unsigned long prevTime = 0;
const unsigned long DT_MS = 50;    // Controller update every 50 ms

// -------------------- PI controller -------------------- //
int currentError = 0;
int errorIntegral = 0;

const int KP = 4;       // Represents 0.4
const int KI = 5;       // Represents 5.0
const int SCALE = 10;   // Used to scale PI calculation back down

const int INTEGRAL_LIMIT = 200;
int pwmValPI = 0;


// -------------------- Function prototypes -------------------- //
int readFiltered(int pin);
void calcError();
void calcPI();
void drivePIControl();
void turnLeft();
void turnRight();
void setMotorSpeeds(int leftPWM, int rightPWM);


// -------------------- Setup -------------------- //
void setup() {
  pinMode(PUCL_PIN, INPUT);
  pinMode(PUCR_PIN, INPUT);
  pinMode(MOTOR_L_PIN, OUTPUT);
  pinMode(MOTOR_R_PIN, OUTPUT);
}


// -------------------- Main loop -------------------- //
void loop() {
  currentTime = millis();

  if (currentTime - prevTime >= DT_MS) {
    prevTime = currentTime;

    calcError();

    // Outside the PI region, use a hard turn
    if (currentError > PID_RANGE_RIGHT) {
      turnLeft();
      errorIntegral = 0;
    }
    else if (currentError < PID_RANGE_LEFT) {
      turnRight();
      errorIntegral = 0;
    }
    else {
      calcPI();
      drivePIControl();
    }
  }
}


// -------------------- Read and process sensor error -------------------- //
void calcError() {
  int leftSignal  = readFiltered(PUCL_PIN);
  int rightSignal = readFiltered(PUCR_PIN);

  currentError = leftSignal - rightSignal;

  // Small errors are treated as zero to reduce twitching
  if (abs(currentError) < ERROR_DEADBAND) {
    currentError = 0;
  }
}


// -------------------- Simple filtering by averaging a few readings -------------------- //
int readFiltered(int pin) {
  int sum = 0;

  for (int i = 0; i < 4; i++) {
    sum += analogRead(pin);
  }

  return sum / 4;
}


// -------------------- Calculate PI control output -------------------- //
void calcPI() {
  errorIntegral += currentError;

  // Limit the integral term to stop wind-up
  errorIntegral = constrain(errorIntegral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  int piValue = (KP * currentError + KI * errorIntegral) / SCALE;

  // Limit steering correction
  pwmValPI = constrain(piValue, -127, 127);
}


// -------------------- Apply PI correction to motors -------------------- //
void drivePIControl() {
  int leftPWM  = COAST_SPEED + pwmValPI;
  int rightPWM = COAST_SPEED - pwmValPI;

  setMotorSpeeds(leftPWM, rightPWM);
}


// -------------------- Hard left turn -------------------- //
void turnLeft() {
  setMotorSpeeds(COAST_SPEED - TURN_SPEED, COAST_SPEED + TURN_SPEED);
}


// -------------------- Hard right turn -------------------- //
void turnRight() {
  setMotorSpeeds(COAST_SPEED + TURN_SPEED, COAST_SPEED - TURN_SPEED);
}


// -------------------- Safely write PWM values to motors -------------------- //
void setMotorSpeeds(int leftPWM, int rightPWM) {
  leftPWM  = constrain(leftPWM, PWM_MIN, PWM_MAX);
  rightPWM = constrain(rightPWM, PWM_MIN, PWM_MAX);

//------------Calibration Function for Sensors---------------//
int sensorOffsetL = 0;
int sensorOffsetR = 0;

void calibrateSensors() {
  for (int i = 0; i < 100; i++) {
    sensorOffsetL += analogRead(PUCL_PIN);
    sensorOffsetR += analogRead(PUCR_PIN);
  }
  sensorOffsetL /= 100;
  sensorOffsetR /= 100;
}

  analogWrite(MOTOR_L_PIN, leftPWM);
  analogWrite(MOTOR_R_PIN, rightPWM);
}

//----------Serial Debug Output----------//
Serial.print("Error: "); Serial.print(currentError);
Serial.print(" PI: "); Serial.println(pwmValPI);
