/*
* Microcontroller code to implement basic bang-bang motor control for PW3 Mouse
* Group 1
* Version 1 - Last updated 10/03/2026 
*/

// IMPORTANT: For this system, we have defined the left side of the mouse to be the positive side

// If testing with only one pick-up coil signal, ground the unused pick-up coil pin. In this case, it should then turn in the direction of the connected motor when near the track.
// For example: If only the right motor is connected, the mouse should try and turn right when the PUC is held near the track 

// Define pins for inputs and outputs
const int PUCL = A0;
const int PUCR = A1; //Post-filtering pick-up coil signals
const int motorL = 5;
const int motorR = 6; // Output ports for PWM

// Motor drive variables
int coastSpeed = 128;
int turnSpeed = 40;

// Timing variables
unsigned long currentTime, prevTime, dT = 0;

// Processing variables
float error= 0;
int targetE = 0;

void setup() {
  // Set pin modes
  pinMode(PUCL, INPUT);
  pinMode(PUCR, INPUT);
  pinMode(motorL, OUTPUT);
  pinMode(motorR, INPUT);

  // Begin serial comms
  Serial.begin(115200); //Baud rate for ESP32
  Serial.println("Begun serial communications...");
}


void loop() {
  // Loop to implement basic bang-bang (open-loop) control
  currentTime = millis();
  dT = currentTime - prevTime; Serial.println("Loop time = " + String(dT) + "ms"); // Get the time for one loop (polling rate)
  calcError(); // Calculate the current error

  // Now decide which motor to drive based on this error signal
  if(error >= targetE){ // Too far right, turn left
    analogWrite(motorL, coastSpeed - turnSpeed);
    analogWrite(motorR, coastSpeed + turnSpeed);
    Serial.println("Turning left...");
  } else {
    analogWrite(motorL, coastSpeed + turnSpeed);
    analogWrite(motorR, coastSpeed - turnSpeed);
    Serial.println("Turning right...");
  }

  prevTime = currentTime; //Update previous time
  Serial.println("\n");
}


void calcError(){
  // Determines the current error signal (currently a single calculation, but for PID control this function will be expanded to include feedback)
  error = analogRead(PUCL) - analogRead(PUCR); // Left side referred to as the positive side, hence a +ve error means we are too far right, and so will turn left
  Serial.println("Error signal calculated! Error = " + String(error));
}