#include<Wire.h>
#include<Zumo32U4.h> //Include the needed libraries
#define NUM_SENSORS 5

Zumo32U4Motors motors;
Zumo32U4Encoders encoders;    //initialize the different Zumo-functionalities
Zumo32U4ButtonA buttonA;
Zumo32U4LineSensors lineSensors;
Zumo32U4LCD lcd;


//
//
// The purpose of this program is to test different line following programs
// and determine the most accurate at measuring distance while following a line
//
//


//variable to store the data from the sensors
int lineSensorValues[NUM_SENSORS]; 

bool useEmitters = true;

// White threshold; white returns values lower than this.
int threshold[NUM_SENSORS] = {300,200,160,200,230}; //{270,160,120,160,200};

struct LineSensorsWhite { // True if White, False if Black
bool L; 
bool LC; 
bool C; 
bool RC; 
bool R;
};

LineSensorsWhite sensorsState = {0,0,0,0,0};



void setup() {
  Serial.begin(9600);  // Start the serial-communication between the robot and the computer.
  lineSensors.initFiveSensors(); // Initialize the 5 lineSensors.
  calibrateThreshold();
  buttonA.waitForPress();
  buttonA.waitForRelease();
  readSensors(sensorsState);
}

int stage = 1;


void loop() {

  if (stage == 1) {
     resetEncoders();
     followLine();
     calculateDistance(avgCounts());
     delay(10000);
  }
  if (stage == 2) {
     resetEncoders();
     followLine2();
     calculateDistance(encoders.getCountsRight());
     delay(10000);
  }
}


//
//
// ---------- Example 1# ----------
//
//

void followLine() {
  
  //while center sensor is NOT white, follow line
  while(!sensorsState.C) {
     bool lineValuesBigger = lineSensorValues[4] > threshold[4] ? 1 : 0;
  
     double fastMotor = 150 * (lineValuesBigger ? double(lineSensorValues[4] / threshold[4]) : double(threshold[4] / lineSensorValues[4]));
     double slowMotor = 60 / (lineValuesBigger ? double(lineSensorValues[4] / threshold[4]) : double(threshold[4] / lineSensorValues[4]));
      //150
      //60
       
     //If the line sensor value is above threshold turn right
        if (lineSensorValues[4] > threshold[4] * 1.1) {
          motors.setSpeeds(fastMotor, slowMotor);
        } // else if line sensor value is lower, turn left
        else if (lineSensorValues[4] < threshold[4] * 0.9) {
          motors.setSpeeds(slowMotor, fastMotor);
        } // else move straight ahead
        else {
          motors.setSpeeds(fastMotor, fastMotor);
        }
        readSensors(sensorsState);
  }
  stopMotors();
}

// Function that gets the average counts from the two encoders
double avgCounts() {

  //store the encoder values and find the avg value
  int countsLeft = encoders.getCountsLeft();
  int countsRight = encoders.getCountsRight();
  
  double avgCounts = (countsLeft + countsRight) / 2;
  
  return avgCounts;
}

//
//
// ---------Example 2# ------------
//
//

void followLine2() {
  
  //while center sensor is NOT white, follow line
  while(!sensorsState.C) {
     
     int rightMotor = 150;
     double leftMotor = 150;

     if (lineSensorValues[4] > threshold[4]) {
        motors.setSpeeds(int(leftMotor * 1.2), rightMotor);
     } else if (lineSensorValues[4] < threshold[4]) {
        motors.setSpeeds(int(leftMotor * 0.8), rightMotor);
     } else {
      motors.setSpeeds(int(leftMotor), rightMotor);
     }
     
     readSensors(sensorsState);
  }
  stopMotors();
}

void resetEncoders() {
  int countsLeft = encoders.getCountsAndResetLeft();
  int countsRight = encoders.getCountsAndResetRight();
}




void calculateDistance(double counts) {
  stopMotors();

  //Constants that make the distance more precise
  // if followLine() it is = 0.9555176
  // if followline2() it is = 0.95715216
  //Calulate distance
  double distance = (counts / 909.7) * PI * 3.9;
  lcd.clear();
  lcd.print("Cm: ");
  lcd.gotoXY(0,1);
  lcd.print(distance);
}

void calibrateThreshold() {
    //local variabel til sorte og hvide værdier
    int black[NUM_SENSORS] = {0,0,0,0,0};
    int white[NUM_SENSORS] = {0,0,0,0,0};
    int belt[NUM_SENSORS] = {0,0,0,0,0};
    
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
     for (int i = 0; i < 5; i++) {
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
    for (int i = 0; i < 5; i++) {
      white[i] = lineSensorValues[i];
     }

    //lcd skal sige "placer ved start"
    lcd.clear();
    lcd.print("Place");
    lcd.gotoXY(0,1);
    lcd.print("start");

    
    //læg hvid og sort sammen parvis, og divider med 2, brug derefter dette som threshold
    for (int i = 0; i < 5; i++) {
      threshold[i] = (black[i] + white[i])/2;
    }   
    Serial.println(String(threshold[0]) + ", " + String(threshold[1]) + ", " + String(threshold[2]) +", " + String(threshold[3]) + ", " + String(threshold[4]));

}

void readSensors(LineSensorsWhite &state){
    // Next line reads the sensor values and store them in the array lineSensorValues 

    lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF); 

    // In the following lines use the values of the sensors to update the struct
      
    sensorsState.L = (lineSensorValues[0] < threshold[0]) ? 1 : 0;
    sensorsState.LC = (lineSensorValues[1] < threshold[1]) ? 1 : 0;
    sensorsState.C = (lineSensorValues[2] < threshold[2]) ? 1 : 0;
    sensorsState.RC = (lineSensorValues[3] < threshold[3]) ? 1 : 0;
    sensorsState.R = (lineSensorValues[4] < threshold[4]) ? 1 : 0;

}
void stopMotors() {
  motors.setSpeeds(0,0);
}
