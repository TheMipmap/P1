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
Zumo32U4ProximitySensors proxSensors;


//
//
//Global variables should be written below this comment
//
//

//variable for the collected wads
int wadsCollected = 0;


//Obstacle located on the left or the right side of the robot:  0 = left ----- 1 = right
bool obstacleRL = 0;

//Iterations of the drive pattern it uses while seaching for wads
int iteration = 0;

//lenght of zumo in cm (8.6)
double zumoL = 8;

//Define the number of brightnesslevels
#define numberOfBrightnessLevels 10

//Define the number of line sensors
#define NUM_SENSORS 3

//Array for defining brightnesslevels
int brightnessLevels[numberOfBrightnessLevels];

//Variable for determining what the proximity is detecting. 0 is nothing, 1 is obstacle, 2 is wad
int proxStatus = 0;

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

//Global variable that stores the encoder counts (These are used when measuring a distance
int totalCountsL = 0;
int totalCountsR = 0;

//Orientation of the zumo
int theta = 0;
double radTheta;

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

//Vector calibrating sumvector on corner
BLA::Matrix<2> resetIterationX;
BLA::Matrix<2> resetIterationY;

void setup() { //------------------------------------------------------------------------------------------------------------------------------------------------------------
  // Code that runs once before the loop() function

  //Start Serial communication between the computer and the zumo
  Serial.begin(9600);

  //Initialize the 3 proximity sensors
  proxSensors.initThreeSensors();

  //For-loop that fill the birgtnesslevels-array
  for (int i = 0; i < numberOfBrightnessLevels; i++) {
    brightnessLevels[i] = i;
    Serial.println("Brightnesslevel(" + String(i) + ") = " + String(brightnessLevels[i]));
  }

  //Manuelly configure the brightnesslevels of the proximity sensors
  proxSensors.setBrightnessLevels(brightnessLevels, numberOfBrightnessLevels);

  //Resets the sum vector
  sumV.Fill(0);

  // Initialize the 3 lineSensors.
  lineSensors.initThreeSensors();

  //Calibrate the threshold between white and black
  calibrateThreshold();

  //Calibrate the turn sensor and reset it
  turnSensorSetup();
  delay(50);
  turnSensorReset();
  delay(200);

  //Reset the encoders to make sure the robot is ready to follow the line and measuere the distance
  resetTotalCounts();
}

int stage = 1;

void loop() { //---------------------------------------------------------------------------------------------------------------------------------------------------
  //Code that loops over and over again until the robot stops.
  if (stage == 1) {
    drivePattern();
  }
  if (stage == 2) {
    //if the Zumo is done
    lcd.clear();
    lcd.print("Done!");
    buttonA.waitForPress();
  }
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


// Function that follows a line
void followLine(int sensorNumber) {


  //Read linesensors to check if middle sensor is white
  readSensors(sensorsState);

  //while center sensor is NOT white, follow the line.
  while (!sensorsState.C) {

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
  }
  //Stop the motors, as we've now reached the white line
  stopMotors();
}


// Function that follows a line and checks for objects
void followLineWithProx(int sensorNumber) {


  //Read linesensors to check if middle sensor is white
  readSensors(sensorsState);

  //while center sensor is NOT white, follow the line.
  while (!sensorsState.C) {

    //Check proximity
    proxRead();

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
  }
  //Stop the motors, as we've now reached the white line
  stopMotors();
}

//A function that follow a line until it has travelled a given distance or hit a line with the middle sensor
bool followLineDistance(double centimeters, int sensorNumber) {

  resetTotalCounts();

  //Variable for the distance
  double distance = calculateDistance(avgCounts());

  //Read the sensor states
  readSensors(sensorsState);

  //Move straight until the center sensor is white or the distance is reached
  while (distance < centimeters && !sensorsState.C) {

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

    //Update distance to see if distance is reached and read sensors to check if a line is reached
    distance = calculateDistance(avgCounts());
    readSensors(sensorsState);

  }
  stopMotors();
  trackUpdate();
  return sensorsState.C ? 1 : 0;
}

// Function that gets the average counts from the two encoders
double avgCounts() {

  //Calculate the avg number of counts
  double avgCounts = (totalCountsL + totalCountsR) / 2;
  return avgCounts;
}

//A function that resets the global totalCount variables
void resetTotalCounts() {
  totalCountsL = 0;
  totalCountsR = 0;
}


//A function that calculates the distance travelled since the encoders were last reset and prints it to the LCD.
double calculateDistance(double counts) {

  //Calulate distance
  double distance = (counts / 909.7) * PI * 3.9 * 0.9395705277;

  //Print the distance on the LCD, uncomment to print
  /*lcd.clear();
    lcd.print("Cm: ");
    lcd.gotoXY(0,1);
    lcd.print(distance);*/

  return distance;
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

  //stop motors to get a precise gyro value
  stopMotors();
  delay(100);

  //Read the sensors to make sure the center sensor is black
  readSensors(sensorsState);

  //Reset angle to 0
  turnSensorReset();

  //Set a variable for the angle
  int angle = (((uint32_t)turnAngle >> 16) * 360) >> 16;

  //Move straight until the center sensor is white.
  while (!sensorsState.C) {

    //Call prox read, to check if there are obstacles
    proxRead();

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
  stopMotors();
}

//A function that moves a given distance straight according to the gyroa and then stops
void moveStraightDistance(int fart, double centimeters) {

  //stop motors to get a precise gyro value
  stopMotors();
  delay(100);

  //Reset total counts variables
  resetTotalCounts();

  //Variable for new distance
  double distance = 0;

  //Reset angle to 0
  turnSensorReset();

  //Set a variable for the angle
  int angle = (((uint32_t)turnAngle >> 16) * 360) >> 16;


  //Move straight until the center sensor is white.
  while (distance < centimeters) {

    //print LCD
    lcd.clear();
    lcd.print(distance);
    lcd.gotoXY(0, 1);
    lcd.print(centimeters);


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

    //Update distance to see if distance is reached and check to see if the zumo is still going straight
    distance = calculateDistance(avgCounts());
    turnSensorUpdate();
    angle = (((uint32_t)turnAngle >> 16) * 360) >> 16;
  }
  stopMotors();
  trackUpdate();
}


//Function for aligning with a detected line, and place the robot perpendicular to the line.
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
      lcd.print("Straight");
      stopMotors();
  }
}

//
//
//Functions involving the gyro and linesensors -----------------------------------------------------------------------------------------------------------------
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
//Function for zumo positioning -----------------------------------------------------------------------------------------------------------------------------------------------------
//
//

//Functions that get the counts from an encoder and resets it. While updating the global variable totalCounts
double getCountsL() {
  double countsL = encoders.getCountsAndResetLeft();
  totalCountsL += countsL;
  return countsL;
}

double getCountsR() {
  double countsR = encoders.getCountsAndResetRight();
  totalCountsR += countsR;
  return countsR;
}

//A function that updates that initializes the calculation of the movement since last time track() was called
void trackUpdate() {
  double countsL = getCountsL(); // Henter den resettede encoder-data (Skulle gerne være 0)
  double countsR = getCountsR();
  double avg = (countsL + countsR) / 2; //Divider med 2 :| <----------------------------------------------------------------------------------------------
  double movement = calculateDistance(avg);
  track(movement);
}


//A function that moves foward while updating position
void moveForward(int spL, int spR) {
  motors.setSpeeds(spL, spR);
  trackUpdate();
}

//A function that turns a given angle in a given direction
void turn(int speed, int grader, char direction) {

  //Reset encoders and save prior counts into totalCounts
  getCountsR();
  getCountsL();

  //make sure the zumo is standing still to get a presice measurrement
  stopMotors();
  delay(150);

  if (direction == 'l' || direction == 'L') {
    turnSensorReset();
    theta += grader;
    Serial.println(((((uint32_t)turnAngle >> 16) * 360) >> 16));
    while (((((uint32_t)turnAngle >> 16) * 360) >> 16) != grader) {
      Serial.println(((((uint32_t)turnAngle >> 16) * 360) >> 16));
      motors.setSpeeds(-speed, speed);
      turnSensorUpdate();
    }
    motors.setSpeeds(0, 0);
  } else if (direction == 'r' || direction == 'R') {
    turnSensorReset();
    theta -= grader;
    while (((((uint32_t)turnAngle >> 16) * 360) >> 16) != 360 - grader) {
      motors.setSpeeds(speed, -speed);
      turnSensorUpdate();
    }
    motors.setSpeeds(0, 0);
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
  }
  if (theta >= 360) {
      theta = theta - 360;
  }
  if (theta < 0) {
      theta = 360 - theta;
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
  delay(300);

  //calculate the angle needed to turn to point at starting position
  lcd.clear();
  lcd.print(theta);
  lcd.gotoXY(0, 1);
  lcd.print(angleSumV);
  delay(200);
  double angleTurn = 180 - theta + angleSumV;
  delay(200);
  if (angleTurn >= 0)turn(150, angleTurn, 'l');
  else turn(150, abs(angleTurn), 'r');

  // calculate the distance home
  BLA::Matrix<2, 2> rotation = {cos(radTheta), sin(radTheta), -sin(radTheta), cos(radTheta)};
  double homeDistance = sqrt(pow(sumV(0), 2) + pow(sumV(1), 2));
  showLCD(theta, homeDistance);

  //Return home
  delay(3000);
  moveStraightDistance(100, homeDistance);
  turn(125, 180 - angleSumV, 'L');
  lcd.clear();
  lcd.print("Unload");
  wadsCollected = 0;

  //return to where it left off
  delay(3000);
  turn(125, angleSumV, 'l');
  moveStraightDistance(100, homeDistance);
  if (iteration % 2 == 1) turn(125, 90 + angleSumV, 'r');
  else  turn(125, 90 - angleSumV, 'l');
  turnSensorReset();
}

//
//
//Functions for proximity sensors-------------------------------------------------------------------------------------------------------------------------------------
//
//

void proxRead() {
  proxSensors.read();
  int proximityLeft = proxSensors.countsFrontWithLeftLeds();
  int proximityRight = proxSensors.countsFrontWithRightLeds();
  Serial.println("proxLeft: " + String(proximityLeft) + " // " + "proxRight: " + String(proximityRight));
  if (proximityRight >= 9 || proximityLeft >= 9) { //If something triggers this number, there is either an obstacle or wad in front of the robot
    stopMotors();
    delay(100);
    moveStraightDistance(100, 8); //move forward to check the number again,
    delay(100);
    proxSensors.read();
    proximityLeft = proxSensors.countsFrontWithLeftLeds();
    proximityRight = proxSensors.countsFrontWithRightLeds();
    if (proximityRight <= 8 || proximityLeft <= 8) {
      if (proximityLeft < proximityRight) {
        turn(150, 35, 'r');
        proxSensors.read();
        proximityLeft = proxSensors.countsFrontWithLeftLeds();
        proximityRight = proxSensors.countsFrontWithRightLeds();
        if (proximityRight > 8) {
          proxStatus = 1; //Define the status of the proximity readings as detecting an obstacle
          obstacleRL = 1;
        } else {
          proxStatus = 2; //Define the status of the proximity readings as detecting a wad.
        }

        turn(150, 35, 'l');
        turnSensorReset();
      } else if (proximityLeft > proximityRight) {
        turn(150, 35, 'l');
        proxSensors.read();
        proximityLeft = proxSensors.countsFrontWithLeftLeds();
        proximityRight = proxSensors.countsFrontWithRightLeds();
        if (proximityLeft > 8) {
          proxStatus = 1; //Define the status of the proximity readings as detecting an obstacle
          obstacleRL = 0;
        } else {
          proxStatus = 2; //Define the status of the proximity readings as detecting a wad.
        }
        turn(150, 35, 'r');
        turnSensorReset();
      } else if (proximityLeft == proximityRight) {
        proxStatus = 2;
      }
    }
    else {
      proxStatus = 1; //Define the status of the proximity readings as detecting an obstacle.
    }
  }

  //Actions that follow now are all after identification

  if (proxStatus == 1 && iteration % 2 == 0) {
    stopMotors();
    avoidObstacleRight();
  } else if (proxStatus == 1 && iteration % 2 == 1) {
    stopMotors();
    avoidObstacleLeft();
  }
  if (proxStatus == 2) {
    lcd.clear();
    lcd.print("Wad spotted");
    wadPickUp();
  }
  proxStatus = 0;
}


void wadPickUp() {

  for (int i = 0; i < 5; i++) {
    //Indicate wad is found
    ledGreen(true);
    delay(500);
    ledGreen(false);
    delay(500);
  }
  wadsCollected += 1;
  if (wadsCollected >= 3) {
    delay(3000);
    returnHome();
  }
}


void avoidObstacleRight() {

  turn(150, 90, 'r');

  //--- første omdrejning omkring objekt---

  proxSensors.read();
  int proximityRight = proxSensors.countsLeftWithLeftLeds();
  resetTotalCounts();

  //Function in case the robot is too far on the corner, and needs to reverse back to the corner to have a neutral starting point
  if (proximityRight <= 2 && obstacleRL == 0) {
    while (proximityRight <= 2) {
      proxSensors.read();
      proximityRight = proxSensors.countsLeftWithLeftLeds();
      moveForward(-100, -100);
    }
    stopMotors();
  }

  while (proximityRight >= 5) {

    moveForward(100, 100);
    proxSensors.read();
    proximityRight = proxSensors.countsLeftWithLeftLeds();

  }

  stopMotors();
  delay(1000);
  moveStraightDistance(100, 15);
  delay(1000);
  double totalDistance = calculateDistance(avgCounts());
  lcd.clear();
  lcd.print("Cm: " + String(totalDistance));
  delay(500);
  turn(150, 90, 'l');
  proxSensors.read();
  proximityRight = proxSensors.countsLeftWithLeftLeds();
  while (proximityRight <= 5) {
    lcd.clear();
    lcd.print("before obs");
    lcd.gotoXY(0, 1);
    lcd.print(proximityRight);
    proxSensors.read();
    proximityRight = proxSensors.countsLeftWithLeftLeds();
    moveForward(100, 100);

  }

  stopMotors();



  //---anden omdrengning omkring objekt---


  proxSensors.read();
  proximityRight = proxSensors.countsLeftWithLeftLeds();
  while (proximityRight >= 5) {
    lcd.clear();
    lcd.print("beside obs");
    lcd.gotoXY(0, 1);
    lcd.print(proximityRight);
    proxSensors.read();
    proximityRight = proxSensors.countsLeftWithLeftLeds();
    moveForward(100, 100);
  }
  stopMotors();


  moveStraightDistance(100, 15);
  delay(1000);
  turn(150, 90, 'l');
  moveStraightDistance(100, totalDistance);
  turn(150, 90, 'r');
  turnSensorReset();



}

void avoidObstacleLeft() {


  turn(150, 90, 'l');

  //--- første omdrejning omkring objekt---

  proxSensors.read();
  int proximityRight = proxSensors.countsRightWithRightLeds();
  resetTotalCounts();

  //Function in case the robot is too far on the corner, and needs to reverse back to the corner to have a neutral starting point
  if (proximityRight <= 2 && obstacleRL == 1) {
    while (proximityRight <= 2) {
      proxSensors.read();
      proximityRight = proxSensors.countsRightWithRightLeds();
      moveForward(-100, -100);
    }
    stopMotors();
  }

  while (proximityRight >= 5) {

    moveForward(100, 100);
    proxSensors.read();
    proximityRight = proxSensors.countsRightWithRightLeds();

  }

  stopMotors();
  delay(1000);
  moveStraightDistance(100, 15);
  delay(1000);
  double totalDistance = calculateDistance(avgCounts());
  lcd.clear();
  lcd.print("Cm: " + String(totalDistance));
  delay(500);
  turn(150, 90, 'r');
  proxSensors.read();
  proximityRight = proxSensors.countsRightWithRightLeds();
  while (proximityRight <= 5) {
    lcd.clear();
    lcd.print("before obs");
    lcd.gotoXY(0, 1);
    lcd.print(proximityRight);
    proxSensors.read();
    proximityRight = proxSensors.countsRightWithRightLeds();
    moveForward(100, 100);

  }

  stopMotors();



  //---anden omdrengning omkring objekt---


  proxSensors.read();
  proximityRight = proxSensors.countsRightWithRightLeds();
  while (proximityRight >= 5) {
    lcd.clear();
    lcd.print("beside obs");
    lcd.gotoXY(0, 1);
    lcd.print(proximityRight);
    proxSensors.read();
    proximityRight = proxSensors.countsRightWithRightLeds();
    moveForward(100, 100);
  }
  stopMotors();


  moveStraightDistance(100, 15);
  delay(1000);
  turn(150, 90, 'r');
  moveStraightDistance(100, totalDistance);
  turn(150, 90, 'l');
  turnSensorReset();


}


//Print the two parameters to the lcd
void showLCD(double distance, double milimeter) {
  lcd.clear();
  lcd.print(milimeter);
  lcd.gotoXY(0, 1);
  lcd.print(distance);
}



// Drive pattern for Zumo to search the field
void drivePattern () {

  //Boolean for when the line is hit
  bool hitLine;

  //setup for the course
  if (iteration == 0) {
    //follow the first line
    followLine(2);
    resetIterationX = sumV;
    delay(100);

    //follow the second line
    turn(150, 90, 'L');
    delay(100);
    followLine(2);
    resetIterationY = sumV;
    delay(100);
    turn(150, 90, 'l');
    delay(100);
    iteration++;
  }

  if (iteration % 2 == 1) {
    hitLine = followLineDistance(iteration * zumoL, 2);
    if (hitLine) {
      delay(100);
      turn(125, 90, 'L');
      delay(100);
      followLineWithProx(2);
      delay(100);
      turn(125, 90, 'L');
      stage = 2;
    } else {
      delay(100);
      turn(125, 90, 'L');
      delay(100);
      moveStraightForwardUntilLine(100);
      delay(100);
      turn(125, 90, 'L');
      delay(100);
      followLine(2);
      delay(100);
      alignAndCorrect();
      turn(125, 180, 'L');
      sumV = resetIterationX;
      iteration++;
    }
  }

  else {
    hitLine = followLineDistance(iteration * zumoL, 0);
    if (hitLine) {
      delay(100);
      turn(125, 90, 'R');
      followLineWithProx(0);
      delay(100);
      turn(125, 180, 'R');
      followLine(2);
      turn(125, 90, 'L');
      stage = 2;
    } else {
      delay(100);
      turn(125, 90, 'R');
      delay(100);
      moveStraightForwardUntilLine(100);
      delay(100);
      turn(125, 90, 'R');
      delay(100);
      followLine(0);
      delay(100);
      alignAndCorrect();
      turn(125, 180, 'R');
      sumV = resetIterationY;
      iteration++;
    }
  }
}
