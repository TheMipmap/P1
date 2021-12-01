//Include the needed libraries
#include <Wire.h>
#include <Zumo32U4.h>

//This is the code from group B377 for our P1 project.

Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;
Zumo32U4LineSensors lineSensors;
Zumo32U4IMU imu;

//
//
//Global variables should be written below this comment
//
//

//Define the number of line sensors
#define NUM_SENSORS 3

//Struct that know whether or not the individuel line sensors detects white or black
struct LineSensorsWhite { // True if White, False if Black
bool L; 
bool C; 
bool R;
};

//Instance of the struct LineSensorsWhite
LineSensorsWhite sensorsState = {0,0,0};


//variable to store the data from the sensors
int lineSensorValues[NUM_SENSORS]; 

//Boolean that determines whether or not to turn on the emitters
bool useEmitters = true;

// White threshold; white returns values lower than this.
int threshold[NUM_SENSORS];




void setup() {
  // Code that runs once before the loop() function

  //Start Serial communication between the computer and the zumo
  Serial.begin(9600);

  // Initialize the 3 lineSensors.
  lineSensors.initThreeSensors();

  //Calibrate the threshold between white and black
  calibrateThreshold();
  
  //Read the sensors and reset the encoders, to make sure the robot is ready to follow the line and measuere the distance
  readSensors(sensorsState);
  resetEncoders();

  //Call the followLine function
  followLine();

  //Call the calculateDistance function, that 
  calculateDistance(avgCounts());
}

void loop() {
  //Code that loops over and over again until the robot stops.
  
}

//
//
//Functions should be declared below this comment
//
//

// A function that calibrates the threshold between black and white.
void calibrateThreshold() {
    //local variabel til sorte og hvide værdier
    int black[NUM_SENSORS];
    int white[NUM_SENSORS];
    int belt[NUM_SENSORS];
    
    //print "Placer over 'sort' og tryk på button A"
    lcd.clear();
    lcd.print("Place");
    lcd.gotoXY(0,1);
    lcd.print("black");
    
    // Vent på button A og aflæs sensorer
      buttonA.waitForPress();
      buttonA.waitForRelease();
      readSensors(sensorsState);
      
    // Gemme de sorte værdier
     for (int i = 0; i < NUM_SENSORS; i++) {
      black[i] = lineSensorValues[i];
     }
     
    //print "placer over hvid og tryk på button A"
    lcd.clear();
    lcd.print("Place");
    lcd.gotoXY(0,1);
    lcd.print("white");

    // vent på button a og aflæs sensorer
    buttonA.waitForPress();
    buttonA.waitForRelease();
    readSensors(sensorsState);

    
    // gem de hvide værdier
    for (int i = 0; i < NUM_SENSORS; i++) {
      white[i] = lineSensorValues[i];
     }

    //lcd skal sige "placer ved start"
    lcd.clear();
    lcd.print("Place");
    lcd.gotoXY(0,1);
    lcd.print("start");

    //Vent på knap før resten af koden forsætter
    buttonA.waitForPress();
    buttonA.waitForRelease();
    
    //læg hvid og sort sammen parvis, og divider med 2, brug derefter dette som threshold
    for (int i = 0; i < NUM_SENSORS; i++) {
      threshold[i] = (black[i] + white[i])/2;
    }   
    Serial.println(String(threshold[0]) + ", " + String(threshold[1]) + ", " + String(threshold[2]));
}


// Function that follows a line
void followLine() {
  
  //while center sensor is NOT white, follow the line.
  while(!sensorsState.C) {

     // A boolean that determines if the lineSensorValue of the outerright sensor is bigger than the threshold for that sensor
     bool lineValuesBigger = lineSensorValues[2] > threshold[2] ? 1 : 0;

     //If the lineSensorValue of the outerright sensor is bigger than the threshold for that sensor, we will divide it by the threshold and get a number between
     double fastMotor = 150 * (lineValuesBigger ? double(lineSensorValues[2] / threshold[2]) : double(threshold[2] / lineSensorValues[2]));
     double slowMotor = 60 / (lineValuesBigger ? double(lineSensorValues[2] / threshold[2]) : double(threshold[2] / lineSensorValues[2]));

       
     //If the line sensor value is above threshold turn right
        if (lineSensorValues[2] > threshold[2] * 1.1) {
          motors.setSpeeds(fastMotor, slowMotor);
        } // else if line sensor value is lower, turn left
        else if (lineSensorValues[2] < threshold[2] * 0.9) {
          motors.setSpeeds(slowMotor, fastMotor);
        } // else move straight ahead
        else {
          motors.setSpeeds(fastMotor, fastMotor);
        }
        readSensors(sensorsState);
  }
  //Stop the motors, as we've now reached the white line
  stopMotors();
}


// Function that gets the average counts from the two encoders
double avgCounts() {

  //store the encoder values and find the avg value
  int countsLeft = encoders.getCountsLeft();
  int countsRight = encoders.getCountsRight();

  //Calculate the avg encoder counts
  double avgCounts = (countsLeft + countsRight) / 2;
  
  return avgCounts;
}

//A function that resets the encoders
void resetEncoders() {
  int countsLeft = encoders.getCountsAndResetLeft();
  int countsRight = encoders.getCountsAndResetRight();
}


//A function that calculates the distance travelled since the encoders were last reset and prints it to the LCD.
void calculateDistance(double counts) {

  //Calulate distance
  double distance = (counts / 909.7) * PI * 3.9 * 0.9395705277;

  //Print the distance on the LCD
  lcd.clear();
  lcd.print("Cm: ");
  lcd.gotoXY(0,1);
  lcd.print(distance);
}

//A function that reads the line sensors and updates the sensorsState array
void readSensors(LineSensorsWhite &state){

 
    // Next line reads the sensor values and store them in the array lineSensorValues
    lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF); 

    // In the following lines, we use the values of the sensors to update the struct
    sensorsState.L = (lineSensorValues[0] < threshold[0]) ? 1 : 0;
    sensorsState.C = (lineSensorValues[1] < threshold[1]) ? 1 : 0;
    sensorsState.R = (lineSensorValues[2] < threshold[2]) ? 1 : 0;

}

//A function a that stops the motors on the Zumo32U4
void stopMotors() {
  motors.setSpeeds(0,0);
}
