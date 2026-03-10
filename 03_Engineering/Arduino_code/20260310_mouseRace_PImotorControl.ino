/*
* Microcontroller code to implement first-order PI control for PW3 Mouse
* Group 1
* Version 2 - Last updated 10/03/2026 
*/

// IMPORTANT: For this system, we have defined the left side of the mouse to be the positive side

// If testing with only one pick-up coil signal, ground the unused pick-up coil pin. In this case, it should then turn in the direction of the connected motor when near the track.
// For example: If only the right motor is connected, the mouse should try and turn right when the PUC is held near the track 



// -- Setup -- //
// Define pins for inputs and outputs
const int PUCL = A0;
const int PUCR = A1; //Post-filtering pick-up coil signals
const int motorL = 5;
const int motorR = 6; // Output ports for PWM

// Motor drive variables
const int coastSpeed = 128;
const int turnSpeed = 40;

// Define PI region for error
const int PIDRangeL = -50;
const int PIDRangeR = 50;

// Timing variables
unsigned long currentTime, prevTime = 0;
const long dT_ms = 50; // Polling rate in ms
const float dT = 0.05; // For integral calculation

// Processing variables
long currentError, errorIntegral, targetError = 0;

//PI control parameters
const float Kp = 0.4;
const float Ki = 5.0;
int pwmValPI = 0;



// --- Code --- //
void setup() {
  // Set pin modes
  pinMode(PUCL, INPUT);
  pinMode(PUCR, INPUT);
  pinMode(motorL, OUTPUT);
  pinMode(motorR, OUTPUT);

  // Begin serial comms
  Serial.begin(115200); //Baud rate for ESP32
  Serial.println("Begun serial communications...");
}


void loop() {
  // Loop to implement basic bang-bang (open-loop) control
  currentTime = millis();
  
  // Every 50ms, poll the PUCs and drive the motors based on this
  if(currentTime - prevTime >= dT_ms){

    calcError(); // Calculate the current error

    // Now decide which motor to drive based on this error signal
    if(currentError > pidRangeRight){ // Too far right, turn left
      turnLeft();
      errorIntegral = 0; // Stop error from running away
      Serial.println("Turning hard left...");
    } 
    
    else if(currentError < pidRangeLeft) {
      turnRight();
      errorIntegral = 0;
      Serial.println("Turning hard right...");
    } 
    
    else {
      calcPI(); // Calculate PI signal based on error
      // Drive motors based on PI value
      Serial.print("Using PI pwm value to drive motors... Value = "); Serial.println(pwmValPI);
      analogWrite(motorL, coastSpeed + pwmValPI);
      analogWrite(motorR, coastSpeed - pwmValPI);
    }

    //Update time and error
    prevTime = currentTime; 
    // Distinguish between different runs
    Serial.println("\n");
  }
}


void calcError(){
  // Determines the current error signal (currently a single calculation, but for PID control this function will be expanded to include feedback)
  currentError = analogRead(PUCL) - analogRead(PUCR); // Left side referred to as the positive side, hence a +ve error means we are too far right, and so will turn left
  Serial.print("Error signal calculated! Error = "); Serial.println(currentError);
}


void calcPI(){
  //Function to calculate the control signal based on error signal
  errorIntegral += currentError * dT;
  PI = (Kp*currentError) + (Ki*errorIntegral);
  Serial.print("PI value calculated! PI = "); Serial.println(PI);

  pwmValPI = constrain(PI, -127, 127); // Constrain PWM value, assuming error signals won't be very large. +ve corresponds to left turn
}


void turnLeft(){
    // Turn the mouse left
  analogWrite(motorL, coastSpeed - turnSpeed);
  analogWrite(motorR, coastSpeed + turnSpeed);
}


void turnRight(){
  // Turn the mouse right
  analogWrite(motorL, coastSpeed + turnSpeed);
  analogWrite(motorR, coastSpeed - turnSpeed);
}