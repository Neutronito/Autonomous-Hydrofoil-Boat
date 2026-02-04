#include <Arduino.h>
#include "main_header.h"

HardwareSerial ServoTx(2);
MPU6050 mpu;

// Setup PID
// At 50Hz, we can sample for 40 seconds - not strictly true as the PID loops do not always run every cycle
pidStruct rollPID = { 800.0, 60.0, 30.0, 0, 50.0, 14000, 0.0175, 0, 0, 0, 0, 0, false, 0, { 0 }, { 0 }, { 0 }, { 0 }, { 0 } };
pidStruct pitchPID = { 700.0, 100.0, 200.0, 0, 100.0, 0, -1, 0, 0, 0, 0, 0, false, 0, { 0 }, { 0 }, { 0 }, { 0 }, { 0 } };
pidStruct altitudePID = { 0.0005, 0.0005, 0.000015, 0, 0.0349, 0.00004, 20.0, 0, 0, 0, 0, 0, false, 0, { 0 }, { 0 }, { 0 }, { 0 }, { 0 } };
pidStruct yawPID = { 720.0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, false, 0, { 0 }, { 0 }, { 0 }, { 0 }, { 0 } };


// Define time logging code
// IMPORTANT NOTE, uncomment below to enable accurate loop timestamping
// #define ENABLE_TIME_LOGGING 


#ifdef ENABLE_TIME_LOGGING
unsigned long timeLogIndex = 0;
unsigned long timeLoggingArray[4000][2] = {0}; // The first index holds the timesamps, the second index holds the code value
#endif

// Code definition - WARNING, these are currently broken due to sensor readings now happening at the start of the loop. 
// 10  Start of loop
// 20  Read receiver
// 25  Actually read IMU on this cycle
// 30  Read IMU
// 40  Calculated roll pid	
// 50  Read ultrasonic sensor
// 60  Calculated altitude pid
// 70  Calculated pitch pid
// 75  Calculated yaw pid
// 80  Calculated remaining outputs
// 90  Updated servos 
// 1000 Loop duration
// 1010 Actual full loop duration

// Define the minimalist time logging code
// IMPORTANT NOTE, uncomment below to enable accurate minimalist cycle time logging
//#define SENSOR_TIME_LOGGING
#ifdef SENSOR_TIME_LOGGING
unsigned long timeLogIndex = 0;
unsigned long currentCode = 0;
unsigned long timeLoggingArray[4000][2] = {0}; // The first index holds the timesamps, the second index holds the code
#endif

// Sensor Logging
// The code is made up of 3 decimal digits
// Digit 1 indicates start time, digit 2 indicates finish reading time
// Digit 1    Read IMU          1
// Digit 2    Read Ultrasonic   10
// Digit 3    Read GPS          100



// Define the GPS Coordinate Logging
// IMPORTANT NOTE, uncomment below to enable GPS Coordinate logging
//#define GPS_COORD_LOGGING
#ifdef GPS_COORD_LOGGING
unsigned long gpsLogIndex = 0;
double gpsLoggingArray[1000][10] = {0}; // The first index holds the timesamps, the second index holds the lat, the third the long, the fourth current GPS heading, the fifth desired gps heading and the sixth current IMU heading.
#endif


#define PIDQUANTITY 4

/*
Aileron Movements and Limits

Aileron   Pitch Down    Pitch Up    Neutral Pitch
Port	    1735	        1300	      1478
Starboard	1170	        1620	      1420
*/

// Since the neutral pitch of starboard aileron is smaller, it will be baseline
// the 0 point of the port one will be the same, just with the offset added
// Neutral was 1387, we added 50
long portNeutralOffset = 70;
unsigned long neutralPitch = 1430;


/*
Pin   In or Out   Description               Type
1     IN          Roll                      PWM        
2     IN          Aileron                   PWM         
3     IN          Throttle                  PWM         
10    IN          Rudder                    PWM         
11    IN          AUX1                      PWM        
12    IN          AUX2                      PWM         
16    IN          Ultrasonic RX             UART
17    OUT         ServoRX Pololu            UART
43    OUT         Clock Pin for IMU         SCL
44    IN          Data Pin for IMU          SDA
18    IN          GPS TX - RX on ESP side   UART
21    OUT         GPS RX - TX on ESP side   UART
*/

// Pin Definition
#define rollPin 1
#define aileronPin 2
#define throttlePin 3
#define rudderPin 10
#define auxPin1 12
#define auxPin2 11


// Define some ultrasonic sensor stuff
#define USS1Rx 16
SoftwareSerial USS1_SERIAL(USS1Rx, -1);
#define USS_PACKET_HEAD 0xFF
#define MAX_DISTANCE_MM 270
#define MIN_DISTANCE_MM 0

// This stores the useable data for the utrasonic sensor
unsigned int currentUltrasoundDistanceMM = 0;
bool newUltrasoundDistanceAvailable = false;
float ussHeight = 0;

// Ultrasonic Sensor Setup
uint8_t ussBuffer[4];
uint8_t ussIndex = 0;
unsigned long ussChecksumFailureCount = 0;
unsigned long ussSuccessfulPackets = 0;
unsigned long ussOutOfBounds = 0;

// This is the cruising height when we are flying. It is in mm. 
// Values here are the defaults
float cruisingHeight = 170.0;
float maximumPitchAngle = 0.139626;
// This is used for pitch correction. It is in mm
#define UssCoGoffset 540

// This is the median filter for the ultrasonic sensor
// It has a default size of 1 (no filter) SIZE MUST BE ODD 
int medianFilterSize = 1;
RunningMedian UssMedianFilter = RunningMedian(medianFilterSize); 

// Yaw setpoint, yaw flag and maximum turning angle
bool firstYawRun = true;
// Below must be in radians
// Currently set to 30 deg
float yawMapMax =  0.523599;
// This defines the controller pwm deadzone, two tailed so +- deadzone
#define YAW_CONTROL_DEADZONE 20

// Define the rudder 0 point
unsigned long rudderZeroPoint = 1520;
// Define the rudder cap, which is the maximum amount the rudder is allowed to turn (two tailed, so +-)
unsigned long rudderCap = 200;

// Servo Pin Definition
#define RUDDERSERVO 0
#define PORTSERVO 1
#define STARSERVO 2
#define ESCOUT 3

//Frequency of main loop
#define loopFrequency 50
// Loop period is calculated using the follow formula: 1,000,000 / loop frequency
// It is in Microseconds, hence the 1 million. IT MUST BE A LONG
#define loopPeriod 20000


// Variable Definition
int readState = 0;
unsigned long rollPulse = 1500;
unsigned long pitchPulse = 1500;
unsigned long throttlePulse = 1500;
unsigned long yawPulse = 1500;
unsigned long aux1Pulse = 1000;
unsigned long aux2Pulse = 1500;
unsigned long portPulse = 1500;
unsigned long starPulse = 1500;
float currentHeight = 0;


// IMU Setup
#define IMU_SDA 44
#define IMU_SCL 43
/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;   // Set true if DMP init was successful
uint8_t MPUIntStatus;    // Holds actual interrupt status byte from MPU
uint8_t devStatus;       // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64];  // FIFO storage buffer
/*---Orientation/Motion Variables---*/
Quaternion q;         // [w, x, y, z]         Quaternion container
VectorFloat gravity;  // [x, y, z]            Gravity vector
float ypr[3];         // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector


// GPS Setup
static const int GpsRxPin = 18, GpsTxPin = 21;
static const uint32_t GPSBaudRate = 9600;

// GPS yaw threshold is in degrees
#define GPS_YAW_THRESHOLD 20

float imuYaw = 0;
float gpsYaw = 0;

float headingStableAngleThreshold = 0.349066; // This is in radians, and is 20*
#define HEADING_STABLE_TIME_THRESHOLD 3000
unsigned long headingStableTime = millis();
bool gpsFirstRun = true;

TinyGPSPlus gps; // Setup TinyGps object
HardwareSerial gpsSerial(1); // GPS is on hardware serial 1

// Some global state tracking variables
bool areGPSCoordsValid = false;
bool coordsUpdated = false;
double curLatitude;
double curLongitude;
uint32_t lastProcessedChars;

// This is for GPS waypoint driving
bool autonomousGpsEnabled = false;
float gpsWaypoints[4][2] = {-31.982158, 115.822128, -31.982061, 115.822422, -31.982429, 115.822628, -31.982511, 115.822244};
int waypointQuantity = 4;
int currentWaypoint = 0;

// Webserver Setup ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Webserver Setup ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Webserver Setup ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Webserver Setup ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Wi-Fi AP credentials
const char* ssid = "HydrofoilESP";
const char* password = "Hello123";

// Create web server
WebServer server(80);

// Generate the HTML page dynamically
String generateHTML() {
  String html = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
    <title>ESP32 - PID Control</title>
    <script>
      function validateForm() {
        let inputs = document.querySelectorAll("input[type=number]");
        for (let i = 0; i < inputs.length; i++) {
          if (isNaN(parseFloat(inputs[i].value))) {
            alert("All PID values must be valid float numbers.");
            return false;
          }
        }
        return true;
      }
      function downloadArray() {
        window.open("/download", "_blank");
      }
      function resetLogIndex() {
        window.open("/reset", "_blank");
      }
    </script>
  </head>
  <body>
    <h2>Set PID Values</h2>
    <form action="/submit" method="GET" onsubmit="return validateForm()">
  )rawliteral";

  for (int i = 0; i < PIDQUANTITY; i++) {
    pidStruct* outPID = getPidData(i);
    html += "<h3>" + String(outPID->label) + "</h3>\n";
    html += "P: <input type=\"number\" step=\"any\" name=\"kp" + String(i + 1) + "\" value=\"" + String(outPID->kp, 8) + "\" required><br>\n";
    html += "I: <input type=\"number\" step=\"any\" name=\"ki" + String(i + 1) + "\" value=\"" + String(outPID->ki, 8) + "\" required><br>\n";
    html += "D: <input type=\"number\" step=\"any\" name=\"kd" + String(i + 1) + "\" value=\"" + String(outPID->kd, 8) + "\" required><br>\n";
    html += "I clamp: <input type=\"number\" step=\"any\" name=\"ki_clamping_bound" + String(i + 1) + "\" value=\"" + String(outPID->ki_clamping_bound, 8) + "\" required><br>\n";
    html += "P Term Parabola Gain: <input type=\"number\" step=\"any\" name=\"pParabolaGain" + String(i + 1) + "\" value=\"" + String(outPID->pParabolaGain, 8) + "\" required><br>\n";
    html += "P Term Parabola Threshold: <input type=\"number\" step=\"any\" name=\"pParabolaThreshold" + String(i + 1) + "\" value=\"" + String(outPID->pParabolaThreshold, 8) + "\" required><br><br>\n";
  }

  html += "Cruising Height (mm): <input type=\"number\" step=\"any\" name=\"cruisingHeight\" value=\"" + String(cruisingHeight, 5) + "\" required><br>\n";
  html += "Maximum Pitch Angle (degree): <input type=\"number\" step=\"any\" name=\"maximumPitchAngle\" value=\"" + String(maximumPitchAngle * 180.0 / 3.141592, 5) + "\" required><br><br>\n";

  html += "Maximum Yaw Angle (degree): <input type=\"number\" step=\"any\" name=\"yawMapMax\" value=\"" + String(yawMapMax * 180.0 / 3.141592, 5) + "\" required><br>\n";
  html += "Median Filter Size (int, must be odd): <input type=\"number\" step=\"any\" name=\"medianFilterSize\" value=\"" + String(medianFilterSize) + "\" required><br><br>\n";

  html += "Neutral Pitch: <input type=\"number\" step=\"any\" name=\"neutralPitch\" value=\"" + String(neutralPitch) + "\" required><br>\n";
  html += "Port Neutral Offset: <input type=\"number\" step=\"any\" name=\"portNeutralOffset\" value=\"" + String(portNeutralOffset) + "\" required><br><br>\n";

  html += "Rudder Centre (Zero) Point: <input type=\"number\" step=\"any\" name=\"rudderZeroPoint\" value=\"" + String(rudderZeroPoint) + "\" required><br>\n";
  html += "Rudder Cap (Maximum PWM value): <input type=\"number\" step=\"any\" name=\"rudderCap\" value=\"" + String(rudderCap) + "\" required><br><br>\n";

  html += "<h3>GPS Waypoints<h3>";

  for (int i = 0; i < waypointQuantity; i++) {
    html += "Latitude: <input type=\"number\" step=\"any\" name=\"lat" + String(i + 1) + "\" value=\"" + String(gpsWaypoints[i][0], 5) + "\" required><br>\n";
    html += "Longitude: <input type=\"number\" step=\"any\" name=\"long" + String(i + 1) + "\" value=\"" + String(gpsWaypoints[i][1], 5) + "\" required><br>\n";
  }

  html += "GPS Waypoint Driving Enabled (bool): <input type=\"number\" step=\"any\" name=\"autonomousGpsEnabled\" value=\"" + String(autonomousGpsEnabled) + "\" required><br><br>\n";

  html += R"rawliteral(
    <input type="submit" value="Submit Data Values">
  </form>
  <br>
  )rawliteral";

  html += "<h3>" + String("Current lines of data logged is: ") + String(rollPID.currentLogIndex) + "</h3>\n";

  html += R"rawliteral(
  <button onclick="downloadArray()">Download Array Data</button>
  <button onclick="resetLogIndex()">Rest the Log Line Count Index</button>
  </body>
  </html>
  )rawliteral";

  html += "<h3>Sensor Outputs</h3>";
  html += "<p> Height above water: " + String(currentUltrasoundDistanceMM);
  html += "<br>Ultrasonic Checksum Failures: " + String(ussChecksumFailureCount);
  html += "<br>Ultrasonic Out of Bounds Errors: " + String(ussOutOfBounds);
  html += "<br>Ultrasonic Successful Packets: " + String(ussSuccessfulPackets);
  
  html += "<br><br>Roll: " + String(ypr[2] * 180 / 3.14159);
  html += "<br>Pitch: " + String(ypr[1] * 180 / 3.14159);
  html += "<br>Yaw: " + String(ypr[0] * 180 / 3.14159);
  
  html += "<br><br>GPS Processed Chars: " + String(gps.charsProcessed());
  html += "<br>GPS Fix Status: " + String(areGPSCoordsValid);
  html += "<br>Latitude: " + String(curLatitude, 8);
  html += "<br>Longitude: " + String(curLongitude, 8);
  html += "</p>";

  return html;
}

// Check if a string is a valid float
bool isValidFloat(const String& s) {
  char* endPtr;
  s.toFloat();  // Ensures internal conversion
  strtof(s.c_str(), &endPtr);
  return endPtr != s.c_str() && *endPtr == '\0';
}

// Handle root page
void handleRoot() {
  server.send(200, "text/html", generateHTML());
}

// Handle form submission
void handleForm() {
  String args[41] = {
    server.arg("kp1"),
    server.arg("ki1"),
    server.arg("kd1"),
    server.arg("ki_clamping_bound1"),
    server.arg("pParabolaGain1"),
    server.arg("pParabolaThreshold1"),
    server.arg("kp2"),
    server.arg("ki2"),
    server.arg("kd2"),
    server.arg("ki_clamping_bound2"),
    server.arg("pParabolaGain2"),
    server.arg("pParabolaThreshold2"),
    server.arg("kp3"),
    server.arg("ki3"),
    server.arg("kd3"),
    server.arg("ki_clamping_bound3"),
    server.arg("pParabolaGain3"),
    server.arg("pParabolaThreshold3"),
    server.arg("kp4"),
    server.arg("ki4"),
    server.arg("kd4"),
    server.arg("ki_clamping_bound4"),
    server.arg("pParabolaGain4"),
    server.arg("pParabolaThreshold4"),
    server.arg("cruisingHeight"),
    server.arg("maximumPitchAngle"),
    server.arg("yawMapMax"),
    server.arg("medianFilterSize"),
    server.arg("neutralPitch"),
    server.arg("portNeutralOffset"),
    server.arg("rudderZeroPoint"),
    server.arg("rudderCap"),
    server.arg("lat1"),
    server.arg("long1"),
    server.arg("lat2"),
    server.arg("long2"),
    server.arg("lat3"),
    server.arg("long3"),
    server.arg("lat4"),
    server.arg("long4"),
    server.arg("autonomousGpsEnabled"),
  };

  float argsf[41];
  // Note, the later values are not handled in this loop because they are longs and unsigned longs, not floats. 
  for (int i = 0; i < 27; i++) {
    if (!isValidFloat(args[i])) {
      server.send(400, "text/html", "<html><body><h2>Error: Invalid float input.</h2><a href='/'>Try again</a></body></html>");
      Serial.println("Invalid float input.");
      Serial.println(i);
      Serial.println(args[i]);
      return;
    }
    argsf[i] = args[i].toFloat();
  }

  // But actually the very last values are also floats, so do this here
  for (int i = 31; i < 39; i++) {
    if (!isValidFloat(args[i])) {
      server.send(400, "text/html", "<html><body><h2>Error: Invalid float input.</h2><a href='/'>Try again</a></body></html>");
      Serial.println("Invalid float input.");
      Serial.println(i);
      Serial.println(args[i]);
      return;
    }
    argsf[i] = args[i].toFloat();
  }

  rollPID.kp = argsf[0];
  rollPID.ki = argsf[1];
  rollPID.kd = argsf[2];
  rollPID.ki_clamping_bound = argsf[3];
  rollPID.pParabolaGain = argsf[4];
  rollPID.pParabolaThreshold = argsf[5];

  pitchPID.kp = argsf[6];
  pitchPID.ki = argsf[7];
  pitchPID.kd = argsf[8];
  pitchPID.ki_clamping_bound = argsf[9];
  pitchPID.pParabolaGain = argsf[10];
  pitchPID.pParabolaThreshold = argsf[11];

  altitudePID.kp = argsf[12];
  altitudePID.ki = argsf[13];
  altitudePID.kd = argsf[14];
  altitudePID.ki_clamping_bound = argsf[15];
  altitudePID.pParabolaGain = argsf[16];
  altitudePID.pParabolaThreshold = argsf[17];

  yawPID.kp = argsf[18];
  yawPID.ki = argsf[19];
  yawPID.kd = argsf[20];
  yawPID.ki_clamping_bound = argsf[21];
  yawPID.pParabolaGain = argsf[22];
  yawPID.pParabolaThreshold = argsf[23];

  cruisingHeight = argsf[24];
  maximumPitchAngle = argsf[25] * 3.141592 / 180.0;

  yawMapMax = argsf[26] * 3.141592 / 180.0;
  medianFilterSize = strtoul(args[27].c_str(), NULL, 10);  // base 10

  //Now recreate the median filter with the correct size
  UssMedianFilter = RunningMedian(medianFilterSize);

  neutralPitch = strtoul(args[28].c_str(), NULL, 10);  // base 10
  portNeutralOffset = strtol(args[29].c_str(), NULL, 10);  // base 10

  rudderZeroPoint = strtol(args[30].c_str(), NULL, 10);  // base 10
  rudderCap = strtol(args[31].c_str(), NULL, 10);  // base 10

  for (int i = 0; i < waypointQuantity; i++) {
    gpsWaypoints[i][0] = argsf[32 + (i * 2)];
    gpsWaypoints[i][1] = argsf[33 + (i * 2)];
  }

  long gpsEnabled = strtol(args[40].c_str(), NULL, 10);  // base 10
  autonomousGpsEnabled = (gpsEnabled > 0);

  // Store values
  /*for (int i = 0; i < 4; i++) {
    for(int j = 0; j<3; j++){
      pidParams[i][j] = args[3*i + j].toFloat();
      //kp[i] = args[i * 3].toFloat();
      //ki[i] = args[i * 3 + 1].toFloat();
      //kd[i] = args[i * 3 + 2].toFloat();
    }
  }*/

  Serial.println("Updated PID Values:");
  for (int i = 0; i < PIDQUANTITY; i++) {
    pidStruct* outPID = getPidData(i);
    Serial.printf("%s\tKp: %.3f, Ki: %.3f, Kd: %.3f\n", outPID->label, outPID->kp, outPID->ki, outPID->kd);
  }

  // Show confirmation
  String response = "<html><body><h2>PID Values Updated</h2>";
  // for (int i = 0; i < PIDQUANTITY; i++) {
  //   pidStruct* outPID = getPidData(i);
  //   response += "<b>" + String(outPID->label) + "</b><br>";
  //   response += "Kp: " + String(outPID->kp, 8) + "<br>";
  //   response += "Ki: " + String(outPID->ki, 8) + "<br>";
  //   response += "Kd: " + String(outPID->kd, 8) + "<br>";
  //   response += "I Clamp: " + String(outPID->ki_clamping_bound, 8) + "<br>";
  //   response += "Proportional Term Parabolic Gain: " + String(outPID->pParabolaGain, 8) + "<br>";
  //   response += "Proportional Term Parabolic Threshold: " + String(outPID->pParabolaThreshold, 8) + "<br><br>";
  // }
  // response += "Cruising Height: " + String(cruisingHeight) + "<br>";
  // response += "Maximum Pitch Angle: " + String(maximumPitchAngle * 180 / 3.141592) + "<br><br>";

  // response += "Maximum Yaw Angle: " + String(yawMapMax * 180 / 3.141592) + "<br>";
  // response += "Median Filter Size: " + String(medianFilterSize) + "<br><br>";

  // response += "Neutral Pitch: " + String(neutralPitch) + "<br>";
  // response += "Port Neutral Offset: " + String(portNeutralOffset) + "<br><br>";

  // response += "Rudder Centre (Zero) Point: " + String(rudderZeroPoint) + "<br><br>";

  response += "<a href='/'>Go Back</a></body></html>";
  server.send(200, "text/html", response);
}

void handleReset() {
  rollPID.currentLogIndex = 0;
  pitchPID.currentLogIndex = 0;
  altitudePID.currentLogIndex = 0;
  yawPID.currentLogIndex = 0;

  rollPID.integral = 0;
  pitchPID.integral = 0;
  altitudePID.integral = 0;
  yawPID.integral = 0;

  ussChecksumFailureCount = 0;
  ussOutOfBounds = 0;
  ussSuccessfulPackets = 0;

  currentWaypoint = 0;
}

// Handle logging data download
void handleDownload() {
  // We no longer send over the server. Now we just send over serial, so all the server.sendContents are replaced with Serial.print
  // Use the following command in a platformio terminal:
  // pio device monitor --port COM5 --baud 115200 | tee output.csv
  // server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  // server.send(200, "text/csv", ""); // Initiate stream

  //Let the browser know all is good so the browser doesn't try to spam requests
  server.send(200, "text/plain", "Serial download started. Check your PlatformIO terminal.");

  // Log the timing data if it's enabled
  #ifdef ENABLE_TIME_LOGGING
    Serial.print("Currently logging the loop timestamps \n");
    // We don't know if we rolled over or not so I will log it all
    for (int i = 0; i < 4000; i++) {
      String line = String(timeLoggingArray[i][0]) + ", " +
      String(timeLoggingArray[i][1]) +
      "\n";
      Serial.print(line);
    }
    Serial.print("\n");
    Serial.print("Finished logging loop timestamps \n");
    Serial.print("\n");
  #endif

  // Log the sensor timing data if enabled
  #ifdef SENSOR_TIME_LOGGING
    Serial.print("Currently logging the sensor loop timestamps \n");
    // We don't know if we rolled over or not so I will log it all
    for (int i = 0; i < 4000; i++) {
      String line = String(timeLoggingArray[i][0]) + ", " +
      String(timeLoggingArray[i][1]) +
      "\n";

      Serial.print(line);
    }
    Serial.print("\n");
    Serial.print("Finished logging the sensor loop timestamps \n");
    Serial.print("\n");
  #endif

  // Log the GPS data if enabled
  #ifdef GPS_COORD_LOGGING
    Serial.print("Timestamp, latitude, longitude, des latitude, des longitude, current GPS heading, desired GPS heading, current IMU heading, desired IMU heading, current waypoint \n");
    // We don't know if we rolled over or not so I will log it all
    for (int i = 0; i < gpsLogIndex; i++) {
      String line = String(gpsLoggingArray[i][0]);
      for (int j = 1; j < 10; j++) {
        line += ", " + String(gpsLoggingArray[i][j], 5);
      }
      line += "\n";
      Serial.print(line);
    }
    Serial.print("\n");
    Serial.print("Finished logging the GPS Coordinates  \n");
    Serial.print("\n");
  #endif


  // Create the extra parameter line, which is the top line
  Serial.print("Cruising height (mm), ");
  Serial.print(String(cruisingHeight, 5));

  Serial.print(", Maximum Pitch Angle (degrees), ");
  Serial.print(String(maximumPitchAngle * 180.0 / 3.141592, 5));

  Serial.print(", Maximum Yaw Angle (degrees), ");
  Serial.print(String(yawMapMax * 180.0 / 3.141592, 5));

  Serial.print(", Median Filter Size (int), ");
  Serial.print(String(medianFilterSize));

  Serial.print(", Neutral Pitch (UL PWM value), ");
  Serial.print(String(neutralPitch));

  Serial.print(", Port Neutral Offset (UL PWM value), ");
  Serial.print(String(portNeutralOffset));

  Serial.print(", Yaw Controller Deadzone (UL PWM value), ");
  Serial.print(String(YAW_CONTROL_DEADZONE));

  Serial.print(", Rudder Zero Point (UL PWM value), ");
  Serial.print(String(rudderZeroPoint));

  Serial.print(", USS Front to back CoG Offset (mm), ");
  Serial.print(String(UssCoGoffset));
  Serial.print("\n");

  delay(10); // Delay to try and mitigate corrupted data

  for (int i = 0; i < PIDQUANTITY; i++) {
    pidStruct* outPid = getPidData(i);

    // Create the header
    Serial.print("Currently logging ");
    Serial.print(outPid->label);
    // Add in the P I and D values
    Serial.print(", P: ");
    Serial.print(String(outPid->kp, 8));
    Serial.print(", I: ");
    Serial.print(String(outPid->ki, 8));
    Serial.print(", D: ");
    Serial.print(String(outPid->kd, 8));
    Serial.print(", I Clamp: ");
    Serial.print(String(outPid->ki_clamping_bound, 8));
    Serial.print(", Proportional Term Parabolic Gain: ");
    Serial.print(String(outPid->pParabolaGain, 8));
    Serial.print(", Proportional Term Parabolic Threshold: ");
    Serial.print(String(outPid->pParabolaThreshold, 8));
    Serial.print("\n");
    Serial.print("Timestamp, Sensor Value, Setpoint, PID output \n");

    delay(10); // Delay to try and mitigate corrupted data

    for (int i = 0; i < outPid->currentLogIndex; i++) {
      String line = String(outPid->logTime[i]) + ", " +
      String(outPid->logSensor[i], 5) + ", " +
      String(outPid->logSetpoint[i], 5) + ", " +
      String(outPid->logOutput[i], 5) +
      "\n";
      Serial.print(line);
      delay(10); // Delay to try and mitigate corrupted data
    }

    // Add a break in between logs
    Serial.print("\n");
    Serial.print("\n");
    delay(10); // Delay to try and mitigate corrupted data
  }


}


void initWiFi() {
  // Start AP mode
  Serial.println("Starting Access Point...");
  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  // Define routes
  server.on("/", handleRoot);
  server.on("/submit", handleForm);
  server.on("/download", handleDownload);
  server.on("/reset", handleReset);

  // Start server
  server.begin();
  Serial.println("Web server started");
}

// Setup Sequence ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Setup Sequence ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Setup Sequence ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Setup Sequence ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void setup() {

  // Init the labels of the PID structures
  strncpy(rollPID.label, "Roll", sizeof(rollPID.label));
  strncpy(pitchPID.label, "Pitch", sizeof(pitchPID.label));
  strncpy(altitudePID.label, "Altitude", sizeof(altitudePID.label));
  strncpy(yawPID.label, "Yaw", sizeof(yawPID.label));

  // Init comms
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GpsRxPin, GpsTxPin); //NEO-6M GPS comms
  ServoTx.begin(115200, SERIAL_8N1, -1, 17); //Pololu Maestro comms
  USS1_SERIAL.begin(9600); //Ultrasonic sensor comms

  // Init Pins
  pinMode(aileronPin, INPUT);
  pinMode(rollPin, INPUT);
  pinMode(throttlePin, INPUT);
  pinMode(rudderPin, INPUT);
  pinMode(auxPin1, INPUT);
  pinMode(auxPin2, INPUT);

  //Init WiFi
  initWiFi();
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  server.begin();

  // Init IMU
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);

  /*Initialize device*/
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if (mpu.testConnection() == false) {
    Serial.println("MPU6050 connection failed");
    while (true)
      ;
  } else {
    Serial.println("MPU6050 connection successful");
  }

  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  /* Making sure it worked (returns 0 if so) */
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));  //Turning ON DMP
    mpu.setDMPEnabled(true);

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();  //Get expected DMP packet size for later comparison
  } else {
    Serial.print(F("DMP Initialization failed (code "));  //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }

  #ifdef ENABLE_TIME_LOGGING
    Serial.println("WARNING - Time logging is enabled");
  #endif
}

// Main Loop ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Main Loop ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Main Loop ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Main Loop ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Main Loop ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void loop() {

  #ifdef ENABLE_TIME_LOGGING
    // Log loop start
    timeLoggingArray[timeLogIndex][0] = micros();
    timeLoggingArray[timeLogIndex][1] = 10;
    timeLogIndex += 1;
    timeLogIndex = timeLogIndex % 4000;
  #endif

  unsigned long loopStart = micros();  //Clock the loop start time to enforce cycle frequency
  unsigned long escOutput;
  unsigned long rudderOutput;
  unsigned long pitchOutput;


  // Read the current receiver input we are up to
  // This reads 1 input per loop, if the period becomes too slow, you have to read all 6 of them in one loop.
  switch (readState) {
    case 0:
      rollPulse = pulseIn(rollPin, HIGH);
      break;
    case 1:
      pitchPulse = pulseIn(aileronPin, HIGH);
      break;
    case 2:
      throttlePulse = pulseIn(throttlePin, HIGH);
      break;
    case 3:
      yawPulse = pulseIn(rudderPin, HIGH);
      break;
    case 4:
      aux1Pulse = pulseIn(auxPin1, HIGH);
      break;
    case 5:
      aux2Pulse = pulseIn(auxPin2, HIGH);
      break;
  }
  // Cycle through receiver inputs
  if (readState == 5) {
    readState = 0;
  } else {
    readState++;
  }

  if (throttlePulse < 1000) {
    // We have a critical failure, receiver connection lost
    aux1Pulse = 1000; // Force the boat into idle mode
  }

  // Read the ultrasonic sensor. This must always be done to not overfill the buffer
  #ifdef SENSOR_TIME_LOGGING
    timeLoggingArray[timeLogIndex][1] = 10;
    timeLoggingArray[timeLogIndex][0] = micros();
    timeLogIndex = (timeLogIndex + 1) % 4000;
  #endif
  
  updateUltrasonicSensor();

  #ifdef SENSOR_TIME_LOGGING
    timeLoggingArray[timeLogIndex][1] = 20;
    timeLoggingArray[timeLogIndex][0] = micros();
    timeLogIndex = (timeLogIndex + 1) % 4000;
  #endif

  

  // Read the GPS. Again this must be done every cycle to clear the buffer
  #ifdef SENSOR_TIME_LOGGING
    timeLoggingArray[timeLogIndex][1] = 100;
    timeLoggingArray[timeLogIndex][0] = micros();
    timeLogIndex = (timeLogIndex + 1) % 4000;
  #endif

  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
  // At the very end of reading data into the GPS, update our variables (if there are new coords)
  if (gps.location.isUpdated()) {
    areGPSCoordsValid = gps.location.isValid();
    curLatitude = gps.location.lat();
    curLongitude = gps.location.lng();
    gpsYaw = gps.course.deg();
    coordsUpdated = true;
  }

  #ifdef SENSOR_TIME_LOGGING
    timeLoggingArray[timeLogIndex][1] = 200;
    timeLoggingArray[timeLogIndex][0] = micros();
    timeLogIndex = (timeLogIndex + 1) % 4000;
  #endif


  // Read the IMU and process it. We do this always so we can update the UI
    #ifdef SENSOR_TIME_LOGGING
      timeLoggingArray[timeLogIndex][1] = 1;
      timeLoggingArray[timeLogIndex][0] = micros();
      timeLogIndex = (timeLogIndex + 1) % 4000;
    #endif
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {  // Get the Latest packet
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      // Update PID struct sensor values
      pitchPID.sensorValue = ypr[1];
      rollPID.sensorValue = ypr[2];
      imuYaw = ypr[0];

      // Now we need to correct the yaw so it grows clockwise and goes from 0 to 360 (heading convention)
      if (imuYaw < 0) {
        imuYaw = (2.0 * M_PI) + imuYaw; //Map to 0 to 360
      }

      // We have updated data, so enable the respective PID loops
      pitchPID.runPidCycle = true;
      rollPID.runPidCycle = true;
      yawPID.runPidCycle = true;

      #ifdef SENSOR_TIME_LOGGING
        timeLoggingArray[timeLogIndex][1] = 2;
        timeLoggingArray[timeLogIndex][0] = micros();
        timeLogIndex = (timeLogIndex + 1) % 4000;
      #endif

    }

  #ifdef ENABLE_TIME_LOGGING
    // Log finished receiver reading
    timeLoggingArray[timeLogIndex][0] = micros();
    timeLoggingArray[timeLogIndex][1] = 20;
    timeLogIndex += 1;
    timeLogIndex = timeLogIndex % 4000;
  #endif


  // We have three states based on the aux1 reading (note 1000 is all the way up)
  // IDLE State ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ IDLE State ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ IDLE State ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ IDLE State ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ IDLE State ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (aux1Pulse < 1300) {

    // Handle the web server (changing PID values)
    server.handleClient();

    // Reset the yaw flag
    firstYawRun = true;
    gpsFirstRun = true;

    // Send some idle outputs to avoid doing dangerous things
    rudderOutput = rudderZeroPoint;
    escOutput = 1500;
    portPulse = (unsigned long)(neutralPitch + portNeutralOffset); 
    starPulse = (unsigned long)(neutralPitch);
  }


  // Driving State ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ MANUAL Driving State ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ MANUAL Driving State ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ MANUAL Driving State ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  else if (aux1Pulse < 1700) {
    // Calculate aileron setpoints
    pitchOutput = map(pitchPulse, 1000, 2000, 1300, 1700);
    portPulse = pitchOutput + (1500 - rollPulse);
    starPulse = 3000 - pitchOutput + (1500 - rollPulse);
    // No need to limit ailerons, Servo Controller does it for us

    // Calculate non reverse ESC output. Power is capped at maximum PWM of 1830
    escOutput = map(throttlePulse, 1000, 2000, 1500, 1830);

    // If leftmost switch flicked down, reverse thruster
    if (aux2Pulse > 1500) {
      escOutput = 3000 - escOutput;
    }

    // Rudder output needs to be inverted
    rudderOutput = 3000 - yawPulse;
  }


  // AUTOMATIC Driving State ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ AUTOMATIC Driving State ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ AUTOMATIC Driving State ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ AUTOMATIC Driving State ~~~~~~~~~~~~~~~~
  else {

    // Handle the roll PID
    if (rollPID.runPidCycle) {
      calculate_PID_output(&rollPID);
      rollPID.runPidCycle = false;
    }

    float rollVal = rollPID.pidOutput;
    // No need to limit as Servo control does that for us

    #ifdef ENABLE_TIME_LOGGING
        // Log finished calculating roll PID
        timeLoggingArray[timeLogIndex][0] = micros();
        timeLoggingArray[timeLogIndex][1] = 40;
        timeLogIndex += 1;
        timeLogIndex = timeLogIndex % 4000;
    #endif

    if (newUltrasoundDistanceAvailable) {
      ussHeight = getHeightUSS(ypr[1], ypr[2]);
      // Set the altitude PID loop to run next cycle
      altitudePID.runPidCycle = true;
    }
    #ifdef ENABLE_TIME_LOGGING
        // Log finished reading ultrasonic sensor
        timeLoggingArray[timeLogIndex][0] = micros();
        timeLoggingArray[timeLogIndex][1] = 50;
        timeLogIndex += 1;
        timeLogIndex = timeLogIndex % 4000;
    #endif

    // Do altitude PID calculation
    if (altitudePID.runPidCycle) {
      altitudePID.setpoint = cruisingHeight;
      altitudePID.sensorValue = ussHeight;
      calculate_PID_output(&altitudePID);
      altitudePID.runPidCycle = false;
    }

    // Limit pitch command angle to within the input bound
    float pitchAngleSetpoint = altitudePID.pidOutput;
    pitchAngleSetpoint = max(pitchAngleSetpoint, -maximumPitchAngle);
    pitchAngleSetpoint = min(pitchAngleSetpoint, maximumPitchAngle);
    #ifdef ENABLE_TIME_LOGGING
      // Log finished calculating altitude PID
      timeLoggingArray[timeLogIndex][0] = micros();
      timeLoggingArray[timeLogIndex][1] = 60;
      timeLogIndex += 1;
      timeLogIndex = timeLogIndex % 4000;
    #endif

    // Handle the pitch PID
    if (pitchPID.runPidCycle) {
      pitchPID.setpoint = pitchAngleSetpoint;
      calculate_PID_output(&pitchPID);
      pitchPID.runPidCycle = false;
    }
    float pitchVal = pitchPID.pidOutput;
    // No need to limit as the servo controller will do that for us

    #ifdef ENABLE_TIME_LOGGING
      // Log finished calculating pitch PID
      timeLoggingArray[timeLogIndex][0] = micros();
      timeLoggingArray[timeLogIndex][1] = 70;
      timeLogIndex += 1;
      timeLogIndex = timeLogIndex % 4000;
    #endif

    // Handle the Yaw PID and whatnot  
    if (autonomousGpsEnabled) { // This case handles autonomous driving 
      yawPID.sensorValue = imuYaw; //Keep this updating frequently

      if (coordsUpdated) { // If we have new coordinates, crunch our maths
        
        // Checked if we have reached the gps location
        if (reachedGpsLoc(curLatitude, curLongitude, gpsWaypoints[currentWaypoint][0], gpsWaypoints[currentWaypoint][1])) {
          currentWaypoint = (currentWaypoint + 1) % waypointQuantity;
        }

        // Check that our orientation has been stable for long enough
        double imuError = calculateSmallestHeadingChange(yawPID.sensorValue, yawPID.setpoint);
        if (abs(imuError) < headingStableAngleThreshold || gpsFirstRun) {
          // This is good, check if enough time has elapsed
          if ((millis() - headingStableTime) >= HEADING_STABLE_TIME_THRESHOLD || gpsFirstRun) {
            // Calculate the heading we need to drive at
            double desiredHeading = gps.courseTo(curLatitude, curLongitude, gpsWaypoints[currentWaypoint][0], gpsWaypoints[currentWaypoint][1]);
            double currentHeading = gps.course.deg();

            // Convert both to radians
            desiredHeading = desiredHeading / 180.0 * M_PI;
            currentHeading = currentHeading / 180.0 * M_PI;

            double headingChange = calculateSmallestHeadingChange(currentHeading, desiredHeading);

            // Update the heading setpoint
            float yawSetpoint = (headingChange) + yawPID.sensorValue;
            
            // Ensure the yawSetpoint is within 0 - 360 deg
            if (yawSetpoint > (2.0 * M_PI)) {
              yawSetpoint -= 2.0 * M_PI;
            } else if (yawSetpoint < 0) {
              yawSetpoint += 2.0 * M_PI;
            }

            #ifdef GPS_COORD_LOGGING
              gpsLoggingArray[gpsLogIndex][5] = currentHeading;
              gpsLoggingArray[gpsLogIndex][6] = desiredHeading;
            #endif

            // Finally, push the modified value to the PID loop
            yawPID.setpoint = yawSetpoint;
            
            gpsFirstRun = false;
          }
        } else {
          // Error is too big, reset the timer
          headingStableTime = millis();
        }

        #ifdef GPS_COORD_LOGGING
              gpsLoggingArray[gpsLogIndex][0] = millis();
              gpsLoggingArray[gpsLogIndex][1] = curLatitude;
              gpsLoggingArray[gpsLogIndex][2] = curLongitude;
              gpsLoggingArray[gpsLogIndex][3] = gpsWaypoints[currentWaypoint][0];
              gpsLoggingArray[gpsLogIndex][4] = gpsWaypoints[currentWaypoint][1];
              gpsLoggingArray[gpsLogIndex][7] = yawPID.sensorValue;
              gpsLoggingArray[gpsLogIndex][8] = yawPID.setpoint;
              gpsLoggingArray[gpsLogIndex][9] = currentWaypoint;
              gpsLogIndex = (gpsLogIndex + 1) % 1000;
        #endif
        coordsUpdated = false;
      }

      // Handle yaw pid
      if (yawPID.runPidCycle) {
        calculate_PID_output_rollover_error(&yawPID);
        yawPID.runPidCycle = false;
      }
      rudderOutput = (unsigned long)(rudderZeroPoint - yawPID.pidOutput);
    }
    // Handle non autonomous mode, so manual driving while foiling
    else {
      rudderOutput = 3000 - yawPulse + int(rudderZeroPoint - 1500);
    }
    
    rudderOutput = constrain(rudderOutput, rudderZeroPoint - rudderCap, rudderZeroPoint + rudderCap);

    #ifdef ENABLE_TIME_LOGGING
      // Log finished calculating pitch PID
      timeLoggingArray[timeLogIndex][0] = micros();
      timeLoggingArray[timeLogIndex][1] = 75;
      timeLogIndex += 1;
      timeLogIndex = timeLogIndex % 4000;
    #endif

    // Now calculate aileron setpoints
    signed long pitchAileronPWM = (signed long)pitchVal;
    signed long rollAileronPWM = (signed long)rollVal;

    portPulse = (unsigned long)(neutralPitch - pitchAileronPWM - rollAileronPWM + portNeutralOffset); // Don't forget to add the pwm centre offset. 
    starPulse = (unsigned long)(neutralPitch + pitchAileronPWM - rollAileronPWM);
    // No need to limit ailerons, Servo Controller does it for us

    // Calculate non reverse ESC output. Power is capped at maximum PWM of 1830
    escOutput = map(throttlePulse, 1000, 2000, 1500, 1830);

    // If leftmost switch flicked down, reverse thruster
    if (aux2Pulse > 1500) {
      escOutput = 3000 - escOutput;
    }

    #ifdef ENABLE_TIME_LOGGING
      // Log finished all PID calculations
      timeLoggingArray[timeLogIndex][0] = micros();
      timeLoggingArray[timeLogIndex][1] = 80;
      timeLogIndex += 1;
      timeLogIndex = timeLogIndex % 4000;
    #endif
  }

  // Write to the servos
  updateServo(PORTSERVO, portPulse);
  updateServo(STARSERVO, starPulse);
  updateServo(RUDDERSERVO, rudderOutput);
  updateServo(ESCOUT, escOutput);

  #ifdef ENABLE_TIME_LOGGING
      // Log finished updating servos
      timeLoggingArray[timeLogIndex][0] = micros();
      timeLoggingArray[timeLogIndex][1] = 90;
      timeLogIndex += 1;
      timeLogIndex = timeLogIndex % 4000;
  #endif

  #ifdef ENABLE_TIME_LOGGING
    // Log the loop duration
    timeLoggingArray[timeLogIndex][0] = micros() - loopStart;
    timeLoggingArray[timeLogIndex][1] = 1000;
    timeLogIndex += 1;
    timeLogIndex = timeLogIndex % 4000;
  #endif

  unsigned long loopDuration = micros() - loopStart;  
  if (loopDuration < loopPeriod) {
    delayMicroseconds(loopPeriod - loopDuration);
  }

  #ifdef ENABLE_TIME_LOGGING
    // Log the actual loop duration
    timeLoggingArray[timeLogIndex][0] = micros() - loopStart;
    timeLoggingArray[timeLogIndex][1] = 1010;
    timeLogIndex += 1;
    timeLogIndex = timeLogIndex % 4000;
  #endif
}

// Function Definitions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Function Definitions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Function Definitions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Function Definitions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void calculate_PID_output(pidStruct* pidInput) {
  // Calculate some variables to be used
  unsigned long currentTime = millis();
  float timeChange = (float)(currentTime - pidInput->prevTime) / 1000.00;
  timeChange = min(timeChange, 0.025f);

  // Calculate current error
  float error = pidInput->setpoint - pidInput->sensorValue;

  // Update intergral value
  pidInput->integral += error * timeChange;

  // Do integral clamping if needed
  // Note we do it this way because the clamping factor is the output max, not the integral max
  // 0 means no clamping enabled
  if (pidInput->ki_clamping_bound >= 0.0000001) { 
    float bound = pidInput->ki_clamping_bound / pidInput->ki;
    pidInput->integral = min(pidInput->integral, bound);
    pidInput->integral = max(pidInput->integral, -bound);
  }

  // Handle whether we are using parabolas or not
  float P_term = pidInput->kp * error;

  if (pidInput->pParabolaThreshold >= 0) { // Check whether parabola is enabled

    if(abs(error) > pidInput->pParabolaThreshold) {
      int sign = round(error / abs(error)); // Record the sign, this is necessary as parabola will eliminate the sign
      P_term = pidInput->pParabolaGain * error * error;
      P_term *= (float)sign; // Restore the sign to the P output
    }
  } 

  // Calculate PID output
  pidInput->pidOutput = (P_term) + (pidInput->ki * pidInput->integral) + (pidInput->kd * ((error - pidInput->prevError) / timeChange));

  // Update our stored values
  pidInput->prevError = error;
  pidInput->prevTime = currentTime;

  // Log data
  unsigned long currentIndex = pidInput->currentLogIndex;
  pidInput->logTime[currentIndex] = millis();
  pidInput->logSensor[currentIndex] = pidInput->sensorValue;
  pidInput->logSetpoint[currentIndex] = pidInput->setpoint;
  pidInput->logOutput[currentIndex] = pidInput->pidOutput;
  pidInput->currentLogIndex += 1;
  pidInput->currentLogIndex = pidInput->currentLogIndex % dataArraySize; // Avoid overflow problems
}

//This is a modified PID function that calculates error with an angle rollover. It assumes the angle is in radians. 
void calculate_PID_output_rollover_error(pidStruct* pidInput) {
  // Calculate some variables to be used
  unsigned long currentTime = millis();
  float timeChange = (float)(currentTime - pidInput->prevTime) / 1000.00;
  timeChange = min(timeChange, 0.025f);

  // Calculate current error
  float error = calculateSmallestHeadingChange(pidInput->sensorValue, pidInput->setpoint);

  // Update intergral value
  pidInput->integral += error * timeChange;

  // Do integral clamping if needed
  // Note we do it this way because the clamping factor is the output max, not the integral max
  // 0 means no clamping enabled
  if (pidInput->ki_clamping_bound >= 0.0000001) { 
    float bound = pidInput->ki_clamping_bound / pidInput->ki;
    pidInput->integral = min(pidInput->integral, bound);
    pidInput->integral = max(pidInput->integral, -bound);
  }

  // Handle whether we are using parabolas or not
  float P_term = pidInput->kp * error;

  if (pidInput->pParabolaThreshold >= 0) { // Check whether parabola is enabled

    if(abs(error) > pidInput->pParabolaThreshold) {
      int sign = round(error / abs(error)); // Record the sign, this is necessary as parabola will eliminate the sign
      P_term = pidInput->pParabolaGain * error * error;
      P_term *= (float)sign; // Restore the sign to the P output
    }
  } 

  // Calculate PID output
  pidInput->pidOutput = (P_term) + (pidInput->ki * pidInput->integral) + (pidInput->kd * ((error - pidInput->prevError) / timeChange));

  // Update our stored values
  pidInput->prevError = error;
  pidInput->prevTime = currentTime;

  // Log data
  unsigned long currentIndex = pidInput->currentLogIndex;
  pidInput->logTime[currentIndex] = millis();
  pidInput->logSensor[currentIndex] = pidInput->sensorValue;
  pidInput->logSetpoint[currentIndex] = pidInput->setpoint;
  pidInput->logOutput[currentIndex] = pidInput->pidOutput;
  pidInput->currentLogIndex += 1;
  pidInput->currentLogIndex = pidInput->currentLogIndex % dataArraySize; // Avoid overflow problems
}

void updateUltrasonicSensor() {
  while (USS1_SERIAL.available()) {
    uint8_t byte = USS1_SERIAL.read();

    // Wait for header
    if (ussIndex == 0 && byte != USS_PACKET_HEAD) {
      continue;
    }

    ussBuffer[ussIndex++] = byte;

    if (ussIndex == 4) {
      ussIndex = 0;

      // Validate checksum
      uint8_t checksum = (ussBuffer[0] + ussBuffer[1] + ussBuffer[2]) & 0xFF;
      if (checksum != ussBuffer[3]) {
        ussChecksumFailureCount += 1; // Record the failure 
        continue;  // invalid frame
      }

      // Convert to distance
      unsigned int distance = (ussBuffer[1] << 8) | ussBuffer[2];

      if (distance < MIN_DISTANCE_MM || distance > MAX_DISTANCE_MM) {
        ussOutOfBounds += 1;
        continue;  // out of bounds
      }

      currentUltrasoundDistanceMM = distance;
      newUltrasoundDistanceAvailable = true;
      ussSuccessfulPackets += 1;

    }
  }
}

float getHeightUSS(float currentPitch, float currentRoll) {
  // Use latest value and apply pitch correction using cosine
  // This is a slightly more advance version that accounts for the fact that the USS is not on the pivot point
  // The second part (to the left) is the roll correction 
  float current_raw = ((currentUltrasoundDistanceMM * cos(currentPitch) - UssCoGoffset * sin(currentPitch)) * cos(currentRoll));
  newUltrasoundDistanceAvailable = false;
  
  if (medianFilterSize <= 1) {
    return(current_raw);
  } else {
    // Add raw value into median filter
    // What we actual return is the current median value in the median filter
    UssMedianFilter.add(current_raw);
    return UssMedianFilter.getMedian();
  }
  
}

bool reachedGpsLoc(float curLat, float curLong, float desLat, float desLong) {
  if (abs(desLat - curLat) < 0.00002) {
    if (abs(desLong - curLong) < 0.00002) {
      return true;
    }
  }
  return false;
}

// Note, headings must be in radians
double calculateSmallestHeadingChange(double currentHeading, double desiredHeading) {
  double clockwiseError = (desiredHeading - currentHeading + (2.0 * M_PI));
  if (clockwiseError > (2.0 * M_PI)) {
    clockwiseError -= (2.0 * M_PI);
  }
  double counterClockwiseError = (2.0 * M_PI) - clockwiseError;

  // Just check if the counterclockwise error is smaller
  if (counterClockwiseError < clockwiseError) {
    return -counterClockwiseError;
  }
  return clockwiseError; 
}

void updateServo(int servoChannel, int servoPulse) {
  servoPulse = servoPulse * 4;
  ServoTx.write(132);
  ServoTx.write(servoChannel);
  ServoTx.write((servoPulse & 127));
  ServoTx.write(((servoPulse >> 7) & 127));
}

void printReceiverReadings() {
  Serial.print("Port PWM: ");
  Serial.println(portPulse);
  Serial.print("Star PWM: ");
  Serial.println(starPulse);
  Serial.print("ESC PWM: ");
  Serial.println(throttlePulse);
  Serial.print("Yaw PWM: ");
  Serial.println(yawPulse);
  Serial.print("Switch 1 PWM: ");
  Serial.println(aux1Pulse);
  Serial.print("Switch 2 PWM: ");
  Serial.println(aux2Pulse);
}

void printIMUReadings() {
  // Ypr is the yaw, pitch, roll array
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(ypr[2] * 180 / M_PI);
  Serial.println();
}

pidStruct* getPidData(int i) {
  switch (i) {
    case 0:
      return &rollPID;
    case 1:
      return &pitchPID;
    case 2:
      return &altitudePID;
    case 3:
      return &yawPID;
    default: 
      return nullptr;
  }
}
