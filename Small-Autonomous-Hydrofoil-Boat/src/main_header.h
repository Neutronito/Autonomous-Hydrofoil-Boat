#include <Wire.h>
#include "MPU6050_6Axis_MotionApps612.h"
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>
#include <RunningMedian.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// Declare the PID Structure
#define dataArraySize 2000
// The dataArraySize is the size for the logging variables
struct pidStruct {
  float kp, ki, kd, integral, ki_clamping_bound, pParabolaGain, pParabolaThreshold, prevError, prevTime, setpoint, sensorValue, pidOutput;
  bool runPidCycle;
  unsigned long currentLogIndex;
  unsigned long logTime[dataArraySize];
  float logSensor[dataArraySize];
  float logSetpoint[dataArraySize];
  float logOutput[dataArraySize];
  char label[12];  //Maximum of 12 characters for the label. This avoids memory issues.
};

void calculate_PID_output(pidStruct* pidInput);
void calculate_PID_output_rollover_error(pidStruct* pidInput);
void updateUltrasonicSensor();
float getHeightUSS(float currentPitch, float currentRoll);
bool reachedGpsLoc(float curLat, float curLong, float desLat, float desLong);
double calculateSmallestHeadingChange(double currentHeading, double desiredHeading);
void updateServo(int servoChannel, int servoPulse);
void printReceiverReadings();
void printIMUReadings();
pidStruct* getPidData(int i);