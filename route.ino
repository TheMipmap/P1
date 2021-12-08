

//Include the needed libraries
#include <Wire.h>
#include <Zumo32U4.h>
#include <BasicLinearAlgebra.h>

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
LineSensorsWhite sensorsState = {0, 0, 0};


//Variable to store the data from the sensors
int lineSensorValues[NUM_SENSORS];

//Boolean that determines whether or not to turn on the emitters
bool useEmitters = true;

//White threshold; white returns values lower than this.
int threshold[NUM_SENSORS];

/* turnAngle is a 32-bit unsigned integer representing the amount
  the robot has turned since the last time turnSensorReset was
  called.  This is computed solely using the Z axis of the gyro, so
  it could be inaccurate if the robot is rotated about the X or Y
  axes.
  Our convention is that a value of 0x20000000 represents a 45
  degree counter-clockwise rotation.  This means that a uint32_t
  can represent any angle between 0 degrees and 360 degrees.  If
  you cast it to a signed 32-bit integer by writing
  (int32_t)turnAngle, that integer can represent any angle between
  -180 degrees and 180 degrees. */
uint32_t turnAngle = 0;

// turnRate is the current angular rate of the gyro, in units of
// 0.07 degrees per second.
int16_t turnRate;

// This is the average reading obtained from the gyro's Z axis
// during calibration.
int16_t gyroOffset;

// This variable helps us keep track of how much time has passed
// between readings of the gyro.
uint16_t gyroLastUpdate = 0;

//Global variables that stores the encoder counts
double countsL;
double countsR;

//Orientation of the zumo
int theta = 0;
double radTheta;

double distance = 0;

//Short the name of BasicLinearAlgebra
using namespace BLA;


//Matrixes and vectors for positioning
//Matrix for local change to global
BLA::Matrix<2, 2> rotation;

//Matrix for global change to local
BLA::Matrix<2, 2> rotation_inv;

//Vector for zumo's movements
BLA::Matrix<2> v;

//Vector for zumo's global position
BLA::Matrix<2> sumV;

void setup() {
  // Code that runs once before the loop() function

  //Start Serial communication between the computer and the zumo
  Serial.begin(9600);

  //Resets the vector
  sumV.Fill(0);

  // Initialize the 3 lineSensors.
  lineSensors.initThreeSensors();

  //Calibrate the threshold between white and black
  calibrateThreshold();

  //Calibrate the turn sensor and reset it
  turnSensorSetup();
  delay(50);
  turnSensorReset();
  setUp();

  //Read the sensors and reset the encoders, to make sure the robot is ready to follow the line and measuere the distance
  readSensors(sensorsState);
  resetEncoders();

}

void loop() {
  //Code that loops over and over again until the robot stops.
  drivePattern ();
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
  lcd.gotoXY(0, 1);
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
  lcd.gotoXY(0, 1);
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
  lcd.gotoXY(0, 1);
  lcd.print("start");

  //Vent på knap før resten af koden forsætter
  buttonA.waitForPress();
  buttonA.waitForRelease();

  //læg hvid og sort sammen parvis, og divider med 2, brug derefter dette som threshold
  for (int i = 0; i < NUM_SENSORS; i++) {
    threshold[i] = (black[i] + white[i]) / 2;
  }
  Serial.println(String(threshold[0]) + ", " + String(threshold[1]) + ", " + String(threshold[2]));
}

void setUp() {

  //Call the followLine function
  followLine(2);
  moveForwardDistance(100, 100, 3);
  delay(100);
  //Turn left
  turn(100, 90, 'L');
  delay(100);
  //Follow outerline again
  followLine(2);

  moveForwardDistance(100, 100, 2);
  //Call the calculateDistance function, that
  calculateDistance(avgCounts());
}

// Function that follows a line
void followLine(int sensorNumber) {

  //
  //Read linesensors to check if middle sensor is white
  readSensors(sensorsState);

  //while center sensor is NOT white, follow the line.
  while (!sensorsState.C) {

    // A boolean that determines if the lineSensorValue of the outerright sensor is bigger than the threshold for that sensor
    bool lineValuesBigger = lineSensorValues[sensorNumber] > threshold[sensorNumber] ? 1 : 0;

    //If the lineSensorValue of the outerright sensor is bigger than the threshold for that sensor, we will divide it by the threshold and get a number between
    double fastMotor = 150 * (lineValuesBigger ? double(lineSensorValues[sensorNumber] / threshold[sensorNumber]) : double(threshold[sensorNumber] / lineSensorValues[sensorNumber]));
    double slowMotor = 60 / (lineValuesBigger ? double(lineSensorValues[sensorNumber] / threshold[sensorNumber]) : double(threshold[sensorNumber] / lineSensorValues[sensorNumber]));


    //If the line sensor value is above threshold turn right
    if (lineSensorValues[sensorNumber] > threshold[sensorNumber] * 1.1) {

      //If the it if uses the left sensor, the robot should turn left when this happens
      if (sensorNumber == 0) {
        moveForward(slowMotor, fastMotor);
      } else {
        //else do the normal turn right
        moveForward(fastMotor, slowMotor);
      }
    } // else if line sensor value is lower, turn left
    else if (lineSensorValues[sensorNumber] < threshold[sensorNumber] * 0.9) {

      if (sensorNumber == 0) {
        // If it uses left sensor it should turn right here
        moveForward(fastMotor, slowMotor);
      } else {
        //Else do the normal left turn
        moveForward(slowMotor, fastMotor);
      }

    } // else move straight ahead
    else {
      moveForward(fastMotor, fastMotor);
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
  lcd.gotoXY(0, 1);
  lcd.print(distance);
}

//A function that reads the line sensors and updates the sensorsState array
void readSensors(LineSensorsWhite &state) {


  // Next line reads the sensor values and store them in the array lineSensorValues
  lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);

  // In the following lines, we use the values of the sensors to update the struct
  sensorsState.L = (lineSensorValues[0] < threshold[0]) ? 1 : 0;
  sensorsState.C = (lineSensorValues[1] < threshold[1]) ? 1 : 0;
  sensorsState.R = (lineSensorValues[2] < threshold[2]) ? 1 : 0;

}

//A function a that stops the motors on the Zumo32U4
void stopMotors() {
  motors.setSpeeds(0, 0);
}

//A function that moves straight according to the gyro.
void moveStraightForwardUntilLine(int fart) {

  readSensors(sensorsState);
  turnSensorReset();

  //Set a variable for the angle
  int angle = (((uint32_t)turnAngle >> 16) * 360) >> 16;

  //Move straight until the center sensor is white.
  while (!sensorsState.C) {

    //If gyro sensors angle is 0 degrees, run the same speed on both motors
    if (angle == 0) {
      moveForward(fart, fart);
    }
    //If the gyro is < 0 degrees turn slightly right
    else if (angle < 180) {
      moveForward(fart + 50, fart);
    }
    //else if gyro is > 0 degrees turn slightly left
    else if (angle > 180) {
      moveForward(fart, fart + 50);
    }

    //Update sensorsState to see if the center sensor is white.
    readSensors(sensorsState);
    turnSensorUpdate();
    angle = (((uint32_t)turnAngle >> 16) * 360) >> 16;
    lcd.clear();
    lcd.print(angle);
  }
  moveForwardDistance(100, 100, 3);
  motors.setSpeeds(0, 0);
  turnSensorReset();
}




//
//
//Functions involving the gyro and linesensors
//
//

//A function that calibrates the turnsensor according to
void turnSensorSetup()
{
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  lcd.clear();
  lcd.print(F("Gyro cal"));

  // Turn on the yellow LED in case the LCD is not available.
  ledYellow(1);

  // Delay to give the user time to remove their finger.
  delay(500);

  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++)
  {
    // Wait for new data to be available, then read it.
    while (!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / 1024;

  // Display the angle (in degrees from -180 to 180) until the
  // user presses A.
  lcd.clear();
  turnSensorReset();
  while (!buttonA.getSingleDebouncedRelease())
  {
    turnSensorUpdate();
    lcd.gotoXY(0, 0);
    lcd.print((((uint32_t)turnAngle >> 16) * 360) >> 16);
    lcd.print(F("   "));
  }
  lcd.clear();
}

// This should be called to set the starting point for measuring
// a turn.  After calling this, turnAngle will be 0.
void turnSensorReset()
{
  gyroLastUpdate = micros();
  turnAngle = 0;
}

// Read the gyro and update the angle.  This should be called as
// frequently as possible while using the gyro to do turns.
void turnSensorUpdate()
{
  // Read the measurements from the gyro.
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;

  // Figure out how much time has passed since the last update (dt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  // Multiply dt by turnRate in order to get an estimation of how
  // much the robot has turned since the last update.
  // (angular change = angular velocity * time)
  int32_t d = (int32_t)turnRate * dt;

  // The units of d are gyro digits times microseconds.  We need
  // to convert those to the units of turnAngle, where 2^29 units
  // represents 45 degrees.  The conversion from gyro digits to
  // degrees per second (dps) is determined by the sensitivity of
  // the gyro: 0.07 degrees per second per digit.
  //
  // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
  // = 14680064/17578125 unit/(digit*us)
  turnAngle += (int64_t)d * 14680064 / 17578125;
}



//
//
//Function for zumo positioning
//
//

//A function that updates that initializes the calculation of the movement since last time track() was called
void trackUpdate() {
  double countsL = encoders.getCountsAndResetLeft(); // Henter den resettede encoder-data (Skulle gerne være 0)
  double countsR = encoders.getCountsAndResetRight();
  double movement = (((countsL + countsR) / 2) / 900) * PI * 3.9;
  distance += movement;
  track(movement);
}

//A function that moves forward a given distance while updating position
void moveForwardDistance(int spL, int spR, int centimeters) {
  double distance = 0;
  while (distance < centimeters) {
    motors.setSpeeds(spL, spR);
    double countsL = encoders.getCountsAndResetLeft(); // Henter den resettede encoder-data (Skulle gerne være 0)
    double countsR = encoders.getCountsAndResetRight();
    double movementUpdate = ((countsL + countsR) / 900) * PI * 3.9;
    distance += movementUpdate;
    track(movementUpdate);
  }
  motors.setSpeeds(0, 0);
  trackUpdate();
}

//A function that moves foward while updating position
void moveForward(int spL, int spR) {
  motors.setSpeeds(spL, spR);
  trackUpdate();
}

//A function that turns a given angle in a given direction
void turn(int speed, int grader, char direction) {
  if (direction == 'l' || direction == 'L') {
    turnSensorReset();
    Serial.println(((((uint32_t)turnAngle >> 16) * 360) >> 16));
    while (((((uint32_t)turnAngle >> 16) * 360) >> 16) != grader) {
      Serial.println(((((uint32_t)turnAngle >> 16) * 360) >> 16));
      motors.setSpeeds(-speed, speed);
      turnSensorUpdate();
    }
    motors.setSpeeds(0, 0);
  } else if (direction == 'r' || direction == 'R') {
    turnSensorReset();
    while (((((uint32_t)turnAngle >> 16) * 360) >> 16) != 360 - grader) {
      motors.setSpeeds(speed, -speed);
      turnSensorUpdate();
    }
    motors.setSpeeds(0, 0);
    trackUpdate();
  }
}

//A function that tracks movement in the global reference frame
void track(double distance) {

  //read the orientatation and encoders
  radTheta = theta * PI / 180;
  v(0) = distance;
  //construct the rotation matrix
  BLA::Matrix<2, 2> rotation = {cos(radTheta), sin(radTheta), -sin(radTheta), cos(radTheta)};

  //calculate the inverse matrix, in order to go from the local basis to the global
  BLA::Matrix<2, 2> rotation_inv = {cos(radTheta), -sin(radTheta), sin(radTheta), cos(radTheta)};
  //do the calculations
  // add the result onto the zum vector
  sumV += rotation_inv * v;
}

//A function that returns to the original position
void returnHome() {
  //calculate the angle from the sumvector by tan(slope y / slope x)
  double angleSumV = 57.2957795 * atan2(sumV(1), sumV(0));
  lcd.clear();
  lcd.print(sumV(0));
  lcd.gotoXY(0, 1);
  lcd.print(sumV(1));
  buttonA.waitForPress();
  delay(1000);
  lcd.clear();
  lcd.print(angleSumV);
  turn(100, 180 - theta + angleSumV, 'l');
  BLA::Matrix<2, 2> rotation = {cos(radTheta), sin(radTheta), -sin(radTheta), cos(radTheta)};
  BLA::Matrix<2> homeDistance = rotation * sumV;
  lcd.clear();
  lcd.print(homeDistance(0));

  moveForwardDistance(100, 100, homeDistance(0));

}

//
//
// Functions for driving pattern
//
//

int count = 0;
double zumoL = 8.6; //lenght of zumo in cm (8.6)

// Drive pattern for Zumo to be in loop
void drivePattern () {

  if (count % 2 == 0) {
    turn(150, 90, 'R');
    delay(500);
    followLine(0);
    delay(300);
    alignAndCorrect();
    delay(500);
    turn(150, 180, 'R');
    delay(500);
    count++;
    distF(2, count * zumoL);
    delay(500);
    turn(150, 90, 'L');
    delay(500);
    moveStraightForwardUntilLine(100);
    delay(500);
    alignAndCorrect();
    delay(500);

  } else {
    turn(150, 90, 'L');
    delay(500);
    followLine(2);
    delay(500);
    alignAndCorrect();
    delay(500);
    turn(150, 180, 'L');
    delay(500);
    count++;
    distF(0, count * zumoL);
    delay(500);
    turn(150, 90, 'R');
    delay(500);
    moveStraightForwardUntilLine(100);
    delay(500);
    alignAndCorrect();

  }
}

void alignAndCorrect() {

  //Find out which lineSensors that turned white
  readSensors(sensorsState);
  int sensorIsWhite;

  if (sensorsState.L) sensorIsWhite = 0;
  if (sensorsState.C) sensorIsWhite = 2;
  if (sensorsState.R) sensorIsWhite = 1;


  switch (sensorIsWhite) {

    case 0:
      while (!sensorsState.R) {
        readSensors(sensorsState);
        moveForward(-70, 100);
      }
      stopMotors();
      break;

    case 1:
      while (!sensorsState.L) {
        readSensors(sensorsState);
        moveForward(100, -70);
      }
      stopMotors();
      break;

    default:
      lcd.clear();
      lcd.print("Error");
      stopMotors();
      readSensors(sensorsState);
  }
}


// Function that follows a line for a distance: count * 86mm
void followLineDistance(int sensorNumber, int milimeters) {
  readSensors(sensorsState);
  double distance = 0;

  //while center sensor is NOT white, follow the line a certain distance.
  while (!sensorsState.C && distance < milimeters ) {

    // A boolean that determines if the lineSensorValue of the outerright sensor is bigger than the threshold for that sensor
    bool lineValuesBigger = lineSensorValues[sensorNumber] > threshold[sensorNumber] ? 1 : 0;

    //If the lineSensorValue of the outerright sensor is bigger than the threshold for that sensor, we will divide it by the threshold and get a number between
    double fastMotor = 150 * (lineValuesBigger ? double(lineSensorValues[sensorNumber] / threshold[sensorNumber]) : double(threshold[sensorNumber] / lineSensorValues[sensorNumber]));
    double slowMotor = 60 / (lineValuesBigger ? double(lineSensorValues[sensorNumber] / threshold[sensorNumber]) : double(threshold[sensorNumber] / lineSensorValues[sensorNumber]));


    //If the line sensor value is above threshold turn right
    if (lineSensorValues[sensorNumber] > threshold[sensorNumber] * 1.1) {
      moveForward(fastMotor, slowMotor);

    } // else if line sensor value is lower, turn left
    else if (lineSensorValues[sensorNumber] < threshold[sensorNumber] * 0.9) {
      moveForward(slowMotor, fastMotor);

    } // else move straight ahead
    else {
      moveForward(fastMotor, fastMotor);
      lcd.clear();
      lcd.print(milimeters);
      lcd.gotoXY(0, 1);
      lcd.print(distance);
      delay(50);
    }
    double countsL = encoders.getCountsAndResetLeft();
    double countsR = encoders.getCountsAndResetRight();
    double movementUpdate = ((countsL + countsR) / 2 / 900) * PI * 3.9;
    distance += movementUpdate;


    readSensors(sensorsState);

    //Stop the motors, as we've now reached the white line
    stopMotors();
    trackUpdate();
  }
}

void distF (int sensorNumber, double centimeter) {
  //
  //Read linesensors to check if middle sensor is white
  readSensors(sensorsState);
  distance = 0;

  //while center sensor is NOT white, follow the line.
  while (distance < centimeter) {

    // A boolean that determines if the lineSensorValue of the outerright sensor is bigger than the threshold for that sensor
    bool lineValuesBigger = lineSensorValues[sensorNumber] > threshold[sensorNumber] ? 1 : 0;

    //If the lineSensorValue of the outerright sensor is bigger than the threshold for that sensor, we will divide it by the threshold and get a number between
    double fastMotor = 120 * (lineValuesBigger ? double(lineSensorValues[sensorNumber] / threshold[sensorNumber]) : double(threshold[sensorNumber] / lineSensorValues[sensorNumber]));
    double slowMotor = 60 / (lineValuesBigger ? double(lineSensorValues[sensorNumber] / threshold[sensorNumber]) : double(threshold[sensorNumber] / lineSensorValues[sensorNumber]));


    //If the line sensor value is above threshold turn right
    if (lineSensorValues[sensorNumber] > threshold[sensorNumber] * 1.1) {

      //If the it if uses the left sensor, the robot should turn left when this happens
      if (sensorNumber == 0) {
        moveForward(slowMotor, fastMotor);
      } else {
        //else do the normal turn right
        moveForward(fastMotor, slowMotor);
      }
    } // else if line sensor value is lower, turn left
    else if (lineSensorValues[sensorNumber] < threshold[sensorNumber] * 0.9) {

      if (sensorNumber == 0) {
        // If it uses left sensor it should turn right here
        moveForward(fastMotor, slowMotor);
      } else {
        //Else do the normal left turn
        moveForward(slowMotor, fastMotor);
      }

    } // else move straight ahead
    else {
      moveForward(fastMotor, fastMotor);
    }
    readSensors(sensorsState);
    showDistance(distance, centimeter);
    if (sensorsState.C && count % 2 == 0)buttonA.waitForPress();
      
    else if (sensorsState.C){
      turn(150,90,'l');
      followLine(2);
      buttonA.waitForPress();
    }
    
    
  }
  //Stop the motors, as we've now reached the white line
  stopMotors();

}

void showDistance(double distance, double milimeter) {
  lcd.clear();
  lcd.print(milimeter);
  lcd.gotoXY(0, 1);
  lcd.print(distance);
}
