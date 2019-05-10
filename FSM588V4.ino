#include "Adafruit_VL53L0X.h"
#include <Stepper.h>
#include <Servo.h>
#include <TimerThree.h>
#include <QTRSensors.h>


/*
   README

   1. When programming the arduino with the USB cable, Disconnect the Vin pin from the 12V breadboard, and Prop the robot on the empty battery box
   2. When powering Arduino with Vin, Plug wire first into arduino and then 12V breadboard. When Removing, first remove from 12V BB, then from Arduino.
   3. Battery Connection on the Breadboard tends to fallout. check this before every run.
   4. Robot trips over crack when going CounterCW on the course. CW works
   5. Update Edit History
*/

/*
   To Do List:
   1. add delay for open servo function.
   2. tune forward circle PD controller KP value
   3. track raw values of the line sensor, if more than 3 or 4 is high at one time, then a perpendicular line is found. 5 lines for a full circle
   4. Solder permenent board for 12V strip and 5V strip
   5. test new encoder wiring for interference with timer interrupt
   6. Test Run on Course.
*/

/*
   Edit History:

   1. 23:21 April 13: LineLeft() complete. Motor Control now uses time interrupt. Rewired line sensor pins.
   2. April 21: secured line sensors with hot glue. switched wheel interrupt pins A:21->18, B:20->19, C:19->2, D:18->3.
      pass "desiredSpeed" as a parameter.
   3. April 22: added fake timer with while loop. added gripper stuff

   /

  //uses Arduino mega
  // PMW 2-13
  // interrupt pins 2, 3, 18, 19, 20, 21
  // communication pins 0, 1, 14 - 21

  // Timer3 Library disables pwm in 2,3, and 5. can still use timer3.pwm();
  // 20, 21 reserved for I2C communication
  // Servo Library removes PWM from 44,45,46

  // time interrupt info
  // TimerThree Library uses timer3 which controls pins 2,3, and 5
  // https://www.robotshop.com/community/forum/t/arduino-101-timers-and-interrupts/13072
  // https://oscarliang.com/arduino-timer-and-interrupt-tutorial/

  // use volatile only for shared variables


  /*
   Drive motor variables
*/
//  Encoder Stuff
unsigned long previousDistance;
volatile unsigned long distanceTraveled;
const int encA = 18; // interrupt
const int encB = 19; // interrupt
const int encC = 2;  // interrupt
const int encD = 3;  // interrupt

// Hbridge Stuff
const int inA1 = 23;  //digital
const int inA2 = 25;  //digital
const int inB1 = 27;  //digital
const int inB2 = 29;  //digital

const int inC1 = 22;  //digital
const int inC2 = 24;  //digital
const int inD1 = 26;  //digital
const int inD2 = 28;  //digital
const int PWMA = 4;  // PWM
const int PWMB = 6;  // PWM
const int PWMC = 7;  // PWM
const int PWMD = 8;  // PWM

// Motor PI controller
float inputSpeed;
int inputDir;
volatile float desiredSpeedA;  //volatile
volatile float desiredSpeedB;  //volatile
volatile float desiredSpeedC;  //volatile
volatile float desiredSpeedD;  //volatile
float signedDesiredSpeed;

float kPA = 4;
float kIA = 4;
float kPB = 4;
float kIB = 4;
float kPC = 4;
float kIC = 4;
float kPD = 4;
float kID = 4;

volatile int countA = 0;  //volatile
float wheelSpeedA;
int outputSpeedA;
float voltHA; //hbridge
double voltAA;//arduino
float errorA;
float err_intA;
float oldIntA = 0;

volatile int countB = 0;  //volatile
float wheelSpeedB;
int outputSpeedB;
float voltHB; //hbridge
double voltAB;//arduino
float errorB;
float err_intB;
float oldIntB = 0;

volatile int countC = 0;  //volatile
float wheelSpeedC;
int outputSpeedC;
float voltHC; //hbridge
double voltAC;//arduino
float errorC;
float err_intC;
float oldIntC = 0;

volatile int countD = 0;  //volatile
float wheelSpeedD;
int outputSpeedD;
float voltHD; //hbridge
double voltAD;//arduino
float errorD;
float err_intD;
float oldIntD = 0;


/*
   Line follower variables
*/
QTRSensors frontQTR;
QTRSensors sideQTR;
const uint8_t SensorCount = 8;
uint16_t sensorValuesFr[SensorCount];
uint16_t sensorValuesSd[SensorCount];
uint16_t sensorCalibrationMins[] = {92, 92, 92, 92, 92, 92, 92, 92};
uint16_t sensorCalibrationMaxs[] = {2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500};
int frontPosition;
float frontLineError;
float frontLastLineError;
float forwardModifier;

int sidePosition;
float sideLineError;
float sideLastLineError;
float sideModifier;

float kPLineFr = 8;
float kDLineFr = 0.01;
float kPLineR = 1.3;
float kDLineR = 0.02;

float kPLineRS = 4;
float kDLineRS = 0.02;

int linePositionCount = 0;

/*
   Field ball detector sensors
*/
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

const int frontRProx = 11;
const int frontLProx = 12;
const int sideProx = 13;
const int sideProxHigh = 52;
/*
   Gripper variables
*/
const int topLimitSwitch = 5;
Stepper bridgeStepper = Stepper(200, 48, 49, 50, 51);
Servo leftServo;
Servo rightServo;
const int leftOpenPosition = 10;
const int rightOpenPosition = 155;

const int leftClosePosition = 160;
const int rightClosePosition = 20;
int stepperCount;

/*
   Zbot Communication
*/
int zBotPos = 0;

/*
   FSM Variables
*/
// Assign "code" values to each state
const int START = 101;
const int LINESCAN = 102;
const int PIVOT90CW = 103;
const int OUTSCAN = 104;
const int INSCAN = 105;
const int PIVOT180CW = 106;
const int PIVOT180IN = 107;
const int FOUNDBALL = 108;
const int RETURNLINE = 109;
const int DEPOSITECUE = 201;
const int DEPOSITELACROSS = 202;
const int DEPOSITETENNIS = 203;
const int RETURNZBOT = 204;
const int ASKZTENNIS = 301;
const int PREDEPOSITETENNIS = 503;
const int PIVOT90CCW = 495;
const int CROSS = 234;
const int CROSSPIVOTCCW =453;
const int LINESCANAGAIN = 666;

// Initialize state variable to START
int state = START;
int previousState;

// Control timers
unsigned long raceTimeLimit = (90) * 1000; // (Enter Time) in seconds.

// FSM input variables:
boolean ballFound = 0;
boolean startLineFound = 0;
int circleLaps = 0;
int lineNumber = 0;

// FSM output variables:


// fake timing
unsigned long previousTime;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial2.begin(9600); //16,17 Sorter
  Serial3.begin(9600); // 14, 15 tx, rx bluetooth
  delay(500);

  // Proximity Sensor
  pinMode(frontRProx, INPUT);
  pinMode(frontLProx, INPUT);
  pinMode(sideProx, INPUT);
  pinMode(sideProxHigh, INPUT);

  // Motor Pins
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(inB1, OUTPUT);
  pinMode(inB2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(inC1, OUTPUT);
  pinMode(inC2, OUTPUT);
  pinMode(PWMC, OUTPUT);

  pinMode(inD1, OUTPUT);
  pinMode(inD2, OUTPUT);
  pinMode(PWMD, OUTPUT);

  // Line Sensor setup
  frontQTR.setTypeRC();
  frontQTR.setSensorPins((const uint8_t[]) {
    33, 35, 37, 39, 41, 43, 45, 47
  }, SensorCount);
  frontQTR.setEmitterPin(31);
  frontQTR.calibrate();

  sideQTR.setTypeRC();
  sideQTR.setSensorPins((const uint8_t[]) {
    32, 34, 36, 38, 40, 42, 44, 46
  }, SensorCount);
  sideQTR.setEmitterPin(30);
  sideQTR.calibrate();
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    frontQTR.calibrationOn.minimum[i] = sensorCalibrationMins[i];
    frontQTR.calibrationOn.maximum[i] = sensorCalibrationMaxs[i];
    sideQTR.calibrationOn.minimum[i] = sensorCalibrationMins[i];
    sideQTR.calibrationOn.maximum[i] = sensorCalibrationMaxs[i];
  }

  // Gripper Servo Setup:
  pinMode(topLimitSwitch, INPUT_PULLUP);
  leftServo.attach(9);
  rightServo.attach(10);
  leftServo.write(leftOpenPosition);
  rightServo.write(rightOpenPosition);
  delay(1000);
  leftServo.detach();
  rightServo.detach();

  // Time of Flight Sensor
  lox.begin();
  previousTime = millis();
  while ((millis() - previousTime) < 100) {};

  // Encoder external interrupt
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);
  pinMode(encC, INPUT_PULLUP);
  pinMode(encD, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encA), tickA, RISING);
  attachInterrupt(digitalPinToInterrupt(encB), tickB, RISING);
  attachInterrupt(digitalPinToInterrupt(encC), tickC, RISING);
  attachInterrupt(digitalPinToInterrupt(encD), tickD, RISING);

  // PI Controller Timer interrupt
  Timer3.initialize(100000);  // Set Time interrupt 0.1 seconds
  Timer3.attachInterrupt(ControlMotor);

  while (digitalRead(frontLProx)) {};

  previousTime = millis();
  while ((millis() - previousTime) < 2000) {};
  previousTime = millis();
}



void loop() {
  // put your main code here, to run repeatedly:

  switch (state) {
    case START:
      // move sideways to start, side sensor in the back.
      BlindForward(0.8);
      // if line found, stop then -> LINESCAN
      if ((millis() - previousTime) > 3000) {
        sidePosition = sideQTR.readLineBlack(sensorValuesSd);
        if ((sidePosition > 4000) && (sidePosition  < 6000)) {              //NEW CHANGE OFF CENTERED
          Stop();
          previousDistance = distanceTraveled;
          state = PIVOT90CW;
        }
      }
      break;
    case PIVOT90CW:
      //pivot 90 to outside, stop then ->OUTSCAN

      previousDistance = distanceTraveled;
      while (distanceTraveled - previousDistance < 120) {
        BlindBackward(1);
      }
      Stop();

      previousDistance = distanceTraveled;
      while (distanceTraveled - previousDistance < 450) {
        PivotCW(1);
      }
      Stop();


      previousDistance = distanceTraveled;
      state = LINESCAN;
      //CHANGE HARD CODE.
      //      sidePosition = sideQTR.readLineBlack(sensorValuesSd);
      //      while((sidePosition > 2000) && (sidePosition < 5000)) {
      //        sidePosition = sideQTR.readLineBlack(sensorValuesSd);
      //        BlindBackward(1);
      //      }
      //
      //      PivotCW(1.2);
      //      frontPosition = frontQTR.readLineBlack(sensorValuesFr);
      //      if ((frontPosition < 4000) && (frontPosition > 2000)) {
      //        Stop();
      //        previousDistance = distanceTraveled;
      //        state = LINESCAN;
      //      }

      break;

    case LINESCAN:
      //linefollow forward CW while moving Zbot
      LineForward(0.8);
      // detected perpendicular line, and enough distance has passed, send Zbot
      sidePosition = sideQTR.readLineBlack(sensorValuesSd);
      if ((sidePosition < 4000) && (sidePosition > 3000) && ((distanceTraveled - previousDistance) > 400)) {
        linePositionCount ++;
        if (linePositionCount == 2) {
          Serial3.print("ZB+POST");
        }
        //ASKZBOT THEN DEPOSITE
        if (linePositionCount == 6) {
          Stop();  //stop
          linePositionCount = 0;
          previousDistance = distanceTraveled;
          state = ASKZTENNIS;
        }

        previousDistance = distanceTraveled;
      }

      //if ballfound -> FOUNDBALLLINE
      if (!digitalRead(frontRProx) || !digitalRead(frontLProx)) {
        PickUp();
      }

      if (ReadTOF() < 70) {
        PickUp();
      }


      //else if timeup -> deposite
      break;


    case OUTSCAN:
      //line follow sideways while moving Zbot
      LineRight(0.8);
      frontPosition = frontQTR.readLineBlack(sensorValuesFr);
      if ((frontPosition < 6000) && (frontPosition > 2000) && ((distanceTraveled - previousDistance) > 1440)) {
        linePositionCount ++;
        //zBotPos ++;
        //ZBot(zBotPos);
        previousDistance = distanceTraveled;
      }
      if (linePositionCount == 5) {
        Stop();
        linePositionCount = 0;
        //zBotPos = 0;
        previousDistance = distanceTraveled;
        state = PIVOT180IN;
      }
      //if ballfound -> FOUNDBALL
      if (ReadTOF() < 80) {
        Stop();
        previousDistance = distanceTraveled;
        previousState = OUTSCAN;
        state = FOUNDBALL;
      }
      break;

    case INSCAN:

      break;

    case FOUNDBALL:
      // forward to pickup ball
      BlindForward(0.6);
      if (ReadTOF() < 15) {
        PickUp();
        state = RETURNLINE;
      }
      break;

    case RETURNLINE:
      BlindBackward(0.6);
      sidePosition = sideQTR.readLineBlack(sensorValuesSd);
      if ((sidePosition < 6000) && (sidePosition > 2000)) {
        state = previousState;
      }
      break;

    case ASKZTENNIS:
      Serial3.flush();
      Serial3.print("ZB+POS?");
      previousTime = millis();
      while ((millis() - previousTime) < 1000) {};
      if (Serial3.available()) {
        if (Serial3.readString() == "OK+POS:T") {
          state = PREDEPOSITETENNIS;
        }
      }
      break;

    case DEPOSITECUE:
      break;

    case DEPOSITELACROSS:
      break;

    case PREDEPOSITETENNIS:
      sidePosition = sideQTR.readLineBlack(sensorValuesSd);
      if (sidePosition > 4000) {
        PivotCW(.7);
      }
      else if (sidePosition < 3000) {
        PivotCCW(.7);
      }

      if (sidePosition < 4000 && (sidePosition > 3000)) {
        Stop();
        state = DEPOSITETENNIS;
      }
      break;
    case DEPOSITETENNIS:

      LineRightAnother(1.2);
      if (!digitalRead(sideProx) || !digitalRead(sideProxHigh)) {
        Stop();
        previousTime = millis();
        while ((millis() - previousTime) < 2000) {}
        Serial2.write('A');
        previousTime = millis();
        while ((millis() - previousTime) < 20000) {};

        state = RETURNZBOT;
      }
      //deposite 3 balls.
      break;

    case RETURNZBOT:
      BlindLeft(0.8);
      frontPosition = frontQTR.readLineBlack(sensorValuesFr);
      if ((frontPosition < 5000) && (frontPosition > 2000)) {
        previousDistance = distanceTraveled;
        Serial3.print("ZB+POSC");
        state = PIVOT90CCW;
      }
      break;

    case PIVOT90CCW:
      previousDistance = distanceTraveled;
      while (distanceTraveled - previousDistance < 450) {
        PivotCCW(1);
      }
      Stop();
      state = CROSS;
      break;
      
    case CROSS:
      BlindForward(0.8);
      // detected perpendicular line, and enough distance has passed, send Zbot
      sidePosition = sideQTR.readLineBlack(sensorValuesSd);
      if ((sidePosition < 4000) && (sidePosition > 3000) && ((distanceTraveled - previousDistance) > 400)) {
        state = CROSSPIVOTCCW;
      }

      //if ballfound -> FOUNDBALLLINE
      if (!digitalRead(frontRProx) || !digitalRead(frontLProx)) {
        PickUp();
      }

      if (ReadTOF() < 70) {
        PickUp();
      }
      
      break;
      
    case CROSSPIVOTCCW:
      previousDistance = distanceTraveled;
      while (distanceTraveled - previousDistance < 120) {
        BlindBackward(1);
      }
      Stop();

      previousDistance = distanceTraveled;
      while (distanceTraveled - previousDistance < 450) {
        PivotCCW(1);
      }
      Stop();


      previousDistance = distanceTraveled;
      state = LINESCANAGAIN;
      break;

      
    case LINESCANAGAIN:
      //linefollow forward CW while moving Zbot
      LineForward(0.8);
      // detected perpendicular line, and enough distance has passed, send Zbot
      sidePosition = sideQTR.readLineBlack(sensorValuesSd);
      if ((sidePosition < 4000) && (sidePosition > 3000) && ((distanceTraveled - previousDistance) > 1000)) {
        linePositionCount ++;
        //ASKZBOT THEN DEPOSITE
        if (linePositionCount == 1) {
          Stop();  //stop
          linePositionCount = 0;
          previousDistance = distanceTraveled;
          state = PREDEPOSITETENNIS;
        }

        previousDistance = distanceTraveled;
      }

      //if ballfound -> FOUNDBALLLINE
      if (!digitalRead(frontRProx) || !digitalRead(frontLProx)) {
        PickUp();
      }

      if (ReadTOF() < 70) {
        PickUp();
      }

      //else if timeup -> deposite
      break;
  }
}

int ReadTOF() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  if ((measure.RangeStatus != 4) && (measure.RangeMilliMeter != 8191) && (measure.RangeMilliMeter != -1)) {
    return measure.RangeMilliMeter;
  } else {
    return 10000;
  }
}

//Servo for Gripper
void PickUp() {
  Stop();
  Serial2.write('i');
  GripperOpen();
  GripperClose();
  PullBridge();
  ReleaseBridge();
  GripperOpen();
  previousTime = millis();
  while ((millis() - previousTime) < 3000) {};
  Serial2.write('d');
  previousTime = millis();
  while ((millis() - previousTime) < 3000) {};
}

void GripperOpen() {
  leftServo.attach(9);
  rightServo.attach(10);
  leftServo.write(leftOpenPosition);
  rightServo.write(rightOpenPosition);
  previousTime = millis();
  while ((millis() - previousTime) < 1000) {};
  leftServo.detach();
  rightServo.detach();

  leftServo.attach(9);
  rightServo.attach(10);
  leftServo.write(leftOpenPosition);
  rightServo.write(rightOpenPosition);
  previousTime = millis();
  while ((millis() - previousTime) < 1000) {};
  leftServo.detach();
  rightServo.detach();
}

void GripperClose() {
  leftServo.attach(9);
  rightServo.attach(10);
  leftServo.write(leftClosePosition);
  rightServo.write(rightClosePosition);
  previousTime = millis();
  while ((millis() - previousTime) < 2000) {};
}

void PullBridge() {
  stepperCount = 0;
  while (stepperCount < 600) {
    previousTime = millis();
    while ((millis() - previousTime) < 10) {};
    bridgeStepper.step(1);
    stepperCount ++;
  }
}

void ReleaseBridge() {
  stepperCount = 0;
  while (stepperCount < 500) {
    previousTime = millis();
    while ((millis() - previousTime) < 10) {};
    bridgeStepper.step(-1);
    stepperCount ++;
  }
  digitalWrite(48, LOW);
  digitalWrite(49, LOW);
  digitalWrite(50, LOW);
  digitalWrite(51, LOW);
}

//Motor PI controller time interrupts
void ControlMotor() {
  wheelSpeedA = countA / (48.0); // count to RPS (480 tick/revolution)
  wheelSpeedB = countB / (48.0); // count to RPS (times 0.1 second)
  wheelSpeedC = countC / (48.0); // count to RPS (=48)
  wheelSpeedD = countD / (48.0); // count to RPS
  countA = 0;
  countB = 0;
  countC = 0;
  countD = 0;
  /*
    Serial.print(wheelSpeedA);
    Serial.print("\t");
    Serial.print(wheelSpeedB);
    Serial.print("\t\t");
    Serial.print(wheelSpeedC);
    Serial.print("\t\t\t");
    Serial.println(wheelSpeedD);
  */
  if (desiredSpeedA != 0) {
    errorA = desiredSpeedA - wheelSpeedA;
    err_intA = oldIntA + errorA * 0.1 * kIA;
    voltHA = kPA * errorA + err_intA; //voltage from Hbridge into motor
    voltAA = 0.4642638704 * exp(0.2144535092 * voltHA); // convert to arduino voltage from hbridge voltage
    outputSpeedA = voltAA * 51;    // convert arduino voltage to PWM.
    outputSpeedA = constrain(outputSpeedA, 0, 255); // constrain the output between pwm limits
    oldIntA = err_intA;

    errorB = desiredSpeedB - wheelSpeedB;
    err_intB = oldIntB + errorB * 0.1 * kIB;
    voltHB = kPB * errorB + err_intB; //voltage from Hbridge into motor
    voltAB = 0.4642638704 * exp(0.2144535092 * voltHB); // convert to arduino voltage from hbridge voltage
    outputSpeedB = voltAB * 51;    // convert arduino voltage to PWM.
    outputSpeedB = constrain(outputSpeedB, 0, 255); // constrain the output between pwm limits
    oldIntB = err_intB;

    errorC = desiredSpeedC - wheelSpeedC;
    err_intC = oldIntC + errorC * 0.1 * kIC;
    voltHC = kPC * errorC + err_intC; //voltage from Hbridge into motor
    voltAC = 0.4642638704 * exp(0.2144535092 * voltHC); // convert to arduino voltage from hbridge voltage
    outputSpeedC = voltAC * 51;    // convert arduino voltage to PWM.
    outputSpeedC = constrain(outputSpeedC, 0, 255); // constrain the output between pwm limits
    oldIntC = err_intC;

    errorD = desiredSpeedD - wheelSpeedD;
    err_intD = oldIntD + errorD * 0.1 * kID;
    voltHD = kPD * errorD + err_intD; //voltage from Hbridge into motor
    voltAD = 0.4642638704 * exp(0.2144535092 * voltHD); // convert to arduino voltage from hbridge voltage
    outputSpeedD = voltAD * 51;    // convert arduino voltage to PWM.
    outputSpeedD = constrain(outputSpeedD, 0, 255); // constrain the output between pwm limits
    oldIntD = err_intD;
  } else {
    outputSpeedA = 0;
    outputSpeedB = 0;
    outputSpeedC = 0;
    outputSpeedD = 0;
  }

  analogWrite(PWMA, outputSpeedA);
  analogWrite(PWMB, outputSpeedB);
  analogWrite(PWMC, outputSpeedC);
  analogWrite(PWMD, outputSpeedD);
}

//Motor encoder interrupts
void tickA() {
  countA++;
  distanceTraveled++;
}

void tickB() {
  countB++;
}

void tickC() {
  countC++;
}

void tickD() {
  countD++;
}

//LineFollower speed modifier
// could combine these once I learn the sensor reading.
void LineForward(float desiredSpeed) {
  frontPosition = frontQTR.readLineBlack(sensorValuesFr);
  frontLineError = (frontPosition - 3500) / 20000.0; //(20000) is calibration by guessing, tune the PD values before setup, not this
  forwardModifier = kPLineFr * frontLineError + kDLineFr * (frontLineError - frontLastLineError);
  frontLastLineError = frontLineError;
  forwardModifier = constrain(forwardModifier, -1, 1);
  /*
    Serial.print(frontPosition);
    Serial.print("\t");
    Serial.println(forwardModifier);
  */
  desiredSpeedA = desiredSpeed + forwardModifier - 0.1;
  desiredSpeedD = desiredSpeedA;
  desiredSpeedB = desiredSpeed - forwardModifier + 0.1;
  desiredSpeedC = desiredSpeedB;
  digitalWrite(inA1, LOW);
  digitalWrite(inA2, HIGH);
  digitalWrite(inB1, LOW);
  digitalWrite(inB2, HIGH);
  digitalWrite(inC1, HIGH);
  digitalWrite(inC2, LOW);
  digitalWrite(inD1, HIGH);
  digitalWrite(inD2, LOW);
}


void BlindRight(float desiredSpeed) {
  desiredSpeedA = desiredSpeed;
  desiredSpeedB = desiredSpeed;
  desiredSpeedC = desiredSpeed;
  desiredSpeedD = desiredSpeed;
  digitalWrite(inA1, HIGH);
  digitalWrite(inA2, LOW);
  digitalWrite(inB1, LOW);
  digitalWrite(inB2, HIGH);
  digitalWrite(inC1, LOW);
  digitalWrite(inC2, HIGH);
  digitalWrite(inD1, HIGH);
  digitalWrite(inD2, LOW);
}
//Motor Output
void BlindForward(float desiredSpeed) {
  desiredSpeedA = desiredSpeed;
  desiredSpeedB = desiredSpeed;
  desiredSpeedC = desiredSpeed;
  desiredSpeedD = desiredSpeed;
  digitalWrite(inA1, LOW);
  digitalWrite(inA2, HIGH);
  digitalWrite(inB1, LOW);
  digitalWrite(inB2, HIGH);
  digitalWrite(inC1, HIGH);
  digitalWrite(inC2, LOW);
  digitalWrite(inD1, HIGH);
  digitalWrite(inD2, LOW);
}

void BlindBackward(float desiredSpeed) {
  desiredSpeedA = desiredSpeed;
  desiredSpeedB = desiredSpeed;
  desiredSpeedC = desiredSpeed;
  desiredSpeedD = desiredSpeed;
  digitalWrite(inA1, HIGH);
  digitalWrite(inA2, LOW);
  digitalWrite(inB1, HIGH);
  digitalWrite(inB2, LOW);
  digitalWrite(inC1, LOW);
  digitalWrite(inC2, HIGH);
  digitalWrite(inD1, LOW);
  digitalWrite(inD2, HIGH);
}


void LineRight(float desiredSpeed) {
  sidePosition = sideQTR.readLineBlack(sensorValuesSd);
  sideLineError = (sidePosition - 3500) / 20000.0; //(20000) is calibration by guessing, tune the PD values before setup, not this
  sideModifier = kPLineR * sideLineError + kDLineR * (sideLineError - sideLastLineError);
  sideLastLineError = sideLineError;
  desiredSpeedA = desiredSpeed - sideModifier;
  desiredSpeedB = desiredSpeedA;
  desiredSpeedD = desiredSpeed + sideModifier;
  desiredSpeedC = desiredSpeedD;
  digitalWrite(inA1, HIGH);
  digitalWrite(inA2, LOW);
  digitalWrite(inB1, LOW);
  digitalWrite(inB2, HIGH);
  digitalWrite(inC1, LOW);
  digitalWrite(inC2, HIGH);
  digitalWrite(inD1, HIGH);
  digitalWrite(inD2, LOW);
}

void LineRightAnother(float desiredSpeed) {
  sidePosition = sideQTR.readLineBlack(sensorValuesSd);
  sideLineError = (sidePosition - 3500) / 50000.0; //(20000) is calibration by guessing, tune the PD values before setup, not this
  sideModifier = kPLineRS * sideLineError + kDLineRS * (sideLineError - sideLastLineError);
  sideLastLineError = sideLineError;
  desiredSpeedA = desiredSpeed - sideModifier;
  desiredSpeedB = desiredSpeedA;
  desiredSpeedD = desiredSpeed + sideModifier;
  desiredSpeedC = desiredSpeedD;
  digitalWrite(inA1, HIGH);
  digitalWrite(inA2, LOW);
  digitalWrite(inB1, LOW);
  digitalWrite(inB2, HIGH);
  digitalWrite(inC1, LOW);
  digitalWrite(inC2, HIGH);
  digitalWrite(inD1, HIGH);
  digitalWrite(inD2, LOW);
}

void BlindLeft(float desiredSpeed) {
  desiredSpeedA = desiredSpeed;
  desiredSpeedB = desiredSpeed;
  desiredSpeedD = desiredSpeed;
  desiredSpeedC = desiredSpeed;
  digitalWrite(inA1, LOW);
  digitalWrite(inA2, HIGH);
  digitalWrite(inB1, HIGH);
  digitalWrite(inB2, LOW);
  digitalWrite(inC1, HIGH);
  digitalWrite(inC2, LOW);
  digitalWrite(inD1, LOW);
  digitalWrite(inD2, HIGH);
}

void LineRightStraight(float desiredSpeed) {

  sidePosition = sideQTR.readLineBlack(sensorValuesSd);
  if (sidePosition > 4000) {
    sideModifier = -.7;
  }
  else if (sidePosition < 3000) {
    sideModifier = -.1;
  }
  else {
    sideModifier = -.4;
  }

  desiredSpeedA = desiredSpeed - sideModifier;
  desiredSpeedB = desiredSpeedA;
  desiredSpeedD = desiredSpeed + sideModifier;
  desiredSpeedC = desiredSpeedD;
  digitalWrite(inA1, HIGH);
  digitalWrite(inA2, LOW);
  digitalWrite(inB1, LOW);
  digitalWrite(inB2, HIGH);
  digitalWrite(inC1, LOW);
  digitalWrite(inC2, HIGH);
  digitalWrite(inD1, HIGH);
  digitalWrite(inD2, LOW);
}

void LineForwardStraight(float desiredSpeed) {

  frontPosition = frontQTR.readLineBlack(sensorValuesSd);
  if (frontPosition > 4000 && frontPosition < 7000) {
    forwardModifier = .3;
  }
  else if (frontPosition < 3000 && frontPosition > 0) {
    forwardModifier = -.3;
  }
  else {
    sideModifier = 0;
  }

  desiredSpeedA = desiredSpeed + forwardModifier;
  desiredSpeedD = desiredSpeedA;
  desiredSpeedB = desiredSpeed - forwardModifier;
  desiredSpeedC = desiredSpeedB;
  digitalWrite(inA1, LOW);
  digitalWrite(inA2, HIGH);
  digitalWrite(inB1, LOW);
  digitalWrite(inB2, HIGH);
  digitalWrite(inC1, HIGH);
  digitalWrite(inC2, LOW);
  digitalWrite(inD1, HIGH);
  digitalWrite(inD2, LOW);
}

void PivotCCW(float desiredSpeed) {
  desiredSpeedA = desiredSpeed;
  desiredSpeedB = desiredSpeed;
  desiredSpeedC = desiredSpeed;
  desiredSpeedD = desiredSpeed;
  digitalWrite(inA1, HIGH);
  digitalWrite(inA2, LOW);
  digitalWrite(inB1, LOW);
  digitalWrite(inB2, HIGH);
  digitalWrite(inC1, HIGH);
  digitalWrite(inC2, LOW);
  digitalWrite(inD1, LOW);
  digitalWrite(inD2, HIGH);
}

void PivotCW(float desiredSpeed) {
  desiredSpeedA = desiredSpeed;
  desiredSpeedB = desiredSpeed;
  desiredSpeedC = desiredSpeed;
  desiredSpeedD = desiredSpeed;
  digitalWrite(inA1, LOW);
  digitalWrite(inA2, HIGH);
  digitalWrite(inB1, HIGH);
  digitalWrite(inB2, LOW);
  digitalWrite(inC1, LOW);
  digitalWrite(inC2, HIGH);
  digitalWrite(inD1, HIGH);
  digitalWrite(inD2, LOW);
}

void Stop() {
  desiredSpeedA = 0;
  desiredSpeedB = 0;
  desiredSpeedC = 0;
  desiredSpeedD = 0;
  digitalWrite(inA1, LOW);
  digitalWrite(inA2, LOW);
  digitalWrite(inB1, LOW);
  digitalWrite(inB2, LOW);
  digitalWrite(inC1, LOW);
  digitalWrite(inC2, LOW);
  digitalWrite(inD1, LOW);
  digitalWrite(inD2, LOW);
  previousTime = millis();
  while ((millis() - previousTime) < 500) {};
}
