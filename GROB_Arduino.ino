/* 
   Code written for GROB (Capstone Project)
   written by Celine Roodal
*/

//include files
#include <Servo.h>
#include "BTS7960.h"


/* Servo constants start */
//define servo pins
const int trapdoor_pwm_pin = 5;
const int lBrush_pwm_pin = 6;
const int rBrush_pwm_pin = 7;
const int flipper_pwm_pin_left = 8;
const int flipper_pwm_pin_right = 11;

//define constants
int brush_stationary = 90;
int brush_counterclockwise = 20; //needs to be between 0 and 90
int brush_clockwise = 160; //needs to be between 90 and 180
int flipper_ground = 90;
int flipper_gnd_offset = 20;
int flipper_range = 90;
int trapdoor_stationary = 119;
int trapdoor_down = 110;

//initialize motors (sets up motor object only)(motors do not turn on)
Servo lBrush;
Servo rBrush;
Servo flipper_left;
Servo flipper_right;
Servo trapdoor;
/* Servo constants end */

/* BTS7960 constants start */
//define BTS7960 pins
const uint8_t R_EN = 28;
const uint8_t L_EN = 29;
const uint8_t L_PWM = 9;
const uint8_t R_PWM = 10;

//speed must be between 0 and 255 with 255 being max speed
int crushSpeed = 200;
int returnSpeed = 200;

int numCompactions = 0;
int compactionsToFull = 3;

//initialize motor controller (creates the object only)
BTS7960 motorController(L_EN, R_EN, L_PWM, R_PWM);

unsigned long previousMillis = 0;  // store the last time
unsigned long crush_interval = 4500;// time to crush (in milliseconds)
unsigned long uncrush_interval = 5500;// time to crush (in milliseconds)
unsigned long elapsedTime = 0;
/* BTS7960 constants end */

/* Ultrasonic constants start */
//define 4X2 array of ultrasonic sensors where each row contains the trig and echo pins
const int num_bin_ultrasonics = 4;
int bin_ultrasonics[num_bin_ultrasonics][2] = {
  {34,35},
  {36,37},
  {38,39},
  {40,41}
  };
 
//define 4x1 array to hold bin ultrasonics distances
float bin_distances[num_bin_ultrasonics];

//define a bin full counter
int bin_full_counter = 0;

//create integers to store distances
int duration_us;
int distance_cm;

const int DISTANCE_THRESHOLD = 0.017; //centimetres conversion

float readUltrasonic(int ultrasonicID); //return ultrasonic reading

/* Ultrasonics constants end */

//uart comms
bool waitForMsg(String expectedMsg); //function declaration

//current sensor ACS724
const int currentPin = A14;
float voltage = 0;
float current = 0;
const float voltageReference = 5.0; // Reference voltage of the Arduino (5V)
const float sensitivity = 0.04; // 50mV per Amp = 0.05V per Amp
bool overcurrent = false;

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200);
  
  //setup pwm pins for motor for left brush and ensure stationary position
  lBrush.attach(lBrush_pwm_pin);
  lBrush.write(brush_stationary);

  //setup pwm pins for motor for right brush and ensure stationary position
  rBrush.attach(rBrush_pwm_pin);
  rBrush.write(brush_stationary);

  //setup pwm pins for motor for flipper and ensure it is at ground position
  flipper_left.attach(flipper_pwm_pin_left);
  flipper_left.write(flipper_ground + flipper_gnd_offset);
  flipper_right.attach(flipper_pwm_pin_right);
  flipper_right.write(flipper_ground - flipper_gnd_offset);

  //setup pwm pins for motor for trapdoor and ensure stationary position
  trapdoor.attach(trapdoor_pwm_pin);
  trapdoor.write(trapdoor_stationary);
    
  //set up ultrasonic sensors
  for (int i = 0; i < num_bin_ultrasonics; i++){
    pinMode(bin_ultrasonics[i][0], OUTPUT); // set trig pin to output mode
    pinMode(bin_ultrasonics[i][1], INPUT);  // set echo pin to input mode
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  //wait for a request to sweep from Husky
  while (!waitForMsg("sweep"));
  
  //request to sweep is received
  //turn on brushes
  lBrush.write(brush_counterclockwise);
  rBrush.write(brush_clockwise);

  //wait for a request to scoop from Husky
  while (!waitForMsg("scoop"));

  //turn off brushes
  lBrush.write(brush_stationary);
  rBrush.write(brush_stationary);


  delay(2000);
  
  //move flipper upwards - use delay to control speed
  for (int angle = 0; angle < flipper_range - flipper_gnd_offset; angle += 1)
  {
    flipper_left.write(flipper_ground + flipper_gnd_offset + angle);
    flipper_right.write(flipper_ground + flipper_gnd_offset - angle);
    delay(50);          
  }
  
  //wait for garbage to be deposited
  delay(5000);

  //move flipper to ground - use delay to control speed
  for (int angle = 0; angle < flipper_range - flipper_gnd_offset; angle += 1)
  {
    flipper_left.write(180-angle);
    flipper_right.write(angle);
    delay(50);          
  }

  //wait for flipper to return to ground
  delay(2000);

  //initialize bin full counter
  bin_full_counter = 0;

  //reset distances
  duration_us = 10000;
  distance_cm = 10000;
  
  //get ultrasonic sensor readings
  for (int i = 0; i < num_bin_ultrasonics; i++){
    bin_distances[i] = readUltrasonic(i);

    //update number of sensors indicating bin full
    if (bin_distances[i] < 25){
      bin_full_counter ++;
    }
  }

  //check if atleast 2 sensors recognizes bin full
  if (bin_full_counter > 1){
    //tell husky to halt operations while compacting
    //Serial.print("compact");

    //crush garbage
    motorController.Crush(crushSpeed);
    //reset variables
    numCompactions++;
    bin_full_counter = 0;

    //crush for a certain amount of time
    elapsedTime = 0;
    previousMillis = millis();
    while (elapsedTime < crush_interval){
      //monitor current
      voltage = analogRead(currentPin) * (voltageReference / 1023.0) - 2.5; // Convert to voltage
      current = voltage / sensitivity; // Calculate current in Amps
      //if current exceeds 19A, stop compacting and print error
      if (current > 19){
        Serial.print("Current exceeded");
        motorController.Stop();
        return;
      }
      //Serial.print("Current:");
      //Serial.println(current);
      //Serial.print("Raw:");
      //Serial.println(analogRead(currentPin));
      elapsedTime = millis()-previousMillis;
    }

    //stop the winch
    motorController.Stop();
    delay(1000);

    //reverse the winch for a certain amount of time
    motorController.Uncrush(returnSpeed);

    elapsedTime = 0;
    previousMillis = millis();
    while (elapsedTime < uncrush_interval){
      //log current
      voltage = analogRead(currentPin) * (voltageReference / 1023.0) - 2.5; // Convert to voltage
      current = voltage / sensitivity; // Calculate current in Amps
      //Serial.print("Current:");
      //Serial.println(current);
      //Serial.print("Raw:");
      //Serial.println(analogRead(currentPin));
      elapsedTime = millis()-previousMillis;
    }

    //stop the winch
    motorController.Stop();
    
    //delay(5000);
    //if ultrasonic1 sees the wall, stop
    //while (readUltrasonic(1) > 6.0);
    
    //check ultrasonics
        //if bin_full_counter still > 2, print error
        //Serial.print("Crushing unsuccessful");
    
    if (numCompactions > compactionsToFull){
      //tell husky to go to drop-off site
      Serial.print("returnhome");

      //wait for "ready to deliver" message from husky
      while (!waitForMsg("deliver"));

      delay(1000);
      
      //open trapdoor
      trapdoor.write(trapdoor_down);
      //wait
      delay(5000);
      //close trapdoor
      trapdoor.write(trapdoor_stationary);

      //reset number of compactions
      numCompactions = 0;

      //tell Husky garbage delivered
      Serial.println("delivered");
    }
  }  
}

//Function to wait for expected message
bool waitForMsg(String expectedMsg){
  String receivedMsg = "";

  if (Serial.available()>0){ // Check if data is available
    receivedMsg = Serial.readString();  // Read the incoming data
    if (receivedMsg == expectedMsg){
      return true;
    }
    else{
      return false;
    }
  }
  return false;
}

// Function to read an ultrasonic sensor and return the distance in cm
float readUltrasonic(int ultrasonicID) {
  // Get the trigger and echo pins for the specified ultrasonic sensor
  int trigPin = bin_ultrasonics[ultrasonicID][0];
  int echoPin = bin_ultrasonics[ultrasonicID][1];

  // Send a 10-microsecond pulse to the trigger pin
  digitalWrite(trigPin, LOW);  // Ensure the trigger pin is low
  delayMicroseconds(2);        // Wait for a brief moment
  digitalWrite(trigPin, HIGH); // Send a pulse
  delayMicroseconds(10);       // Keep the pulse for 10 microseconds
  digitalWrite(trigPin, LOW);  // Stop the pulse

  // Measure the duration of the pulse from the echo pin
  long duration_us = pulseIn(echoPin, HIGH);

  // Calculate the distance in centimeters (speed of sound: 0.0343 cm/µs)
  float distance_cm = DISTANCE_THRESHOLD * duration_us;

  return distance_cm;  // Return the calculated distance
}
