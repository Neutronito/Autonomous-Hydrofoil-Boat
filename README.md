# Synopsis
This repository contains the source code for a small, autonomous hydrofoil boat. 

The miscellaneous code directory contains the logging code, which should be run in Python to generate graphs based on the testing data. It also contains the Pololu Servo Controller Script, which checks for valid communications from the ESP32 and will cut power if this is not met. 

The Small-Autonomous-Hydrofoil-Boat directory contains the source code for the onboard ESP32 microcontroller. It is a Platformio project, and should be opened using Platformio. The target microncontroller is a Lilygo T-Display-S3 ESP32. 

# Software Parameters
## ESP32 Pinout
| Pin | In or Out | Description    | Type |
| --- | --------- | -------------- | ---- |
| 1   | IN        | Roll           | PWM  |
| 2   | IN        | Aileron        | PWM  |
| 3   | IN        | Throttle       | PWM  |
| 10  | IN        | Rudder         | PWM  |
| 11  | IN        | AUX1           | PWM  |
| 12  | IN        | AUX2           | PWM  |
| 16  | IN        | Ultrasonic RX  | UART  |
| 17  | OUT       | ServoRX Polulu | UART |
| 43  | OUT       | Clock Pin for IMU | SCL |
| 44  | IN       | Data Pin for IMU | SDA |
| 18  | IN       | GPS TX - RX on Arduino side | UART |
| 21  | OUT       | GPS RX - TX on Arduino side | UART |


## Polulu Servo Controller Pinout
| Pin | Description             |
| --- | ----------------------- |
| 0   | Rudder Servo            |
| 1   | Port Aileron Servo      |
| 2   | Starboard Aileron Servo |
| 3   | ESC PWM Control         |

## Serial Communications
| Channel | Type | Description             |
| -- | ----- |  ----------------------- |
| 0   | Hardware Serial | Serial Comms (Laptop)            |
| 1   | Hardware Serial | GPS Sensor                       |  
| 2   | Hardware Serial | Pololu Servo Controller          |
| N/A | Software Serial | Ultrasonic Sensor                |


# Data Streaming
## Key Things to Note
* Whatever terminal is used, you must ensure that DTR and RTS are disabled. This is to ensure the terminal does not force the ESP32 to restart, wiping its data
* Teraterm is the recommended terminal to use
* You must unplug the ESP32 before closing the terminal session. If you do not, there is a risk that closing the terminal session will reset the ESP32, causing data loss. 
  
## Instructions
<ol>
<li>Plug the ESP32 into the laptop</li>
<li>Open Teraterm, but do not open a connection</li>
<li>Ensure the Serial settings on Teraterm are not using DTR or RTS (set control flow to none) and that the baud rate is correct (115,200)</li>
<li>Open a connection to the ESP32</li>
<li>Start a logging session</li>
<li>Navigate to the web browser and click on download data</li>
<li>Wait for the transfer to finish, and close the logging session (but not the connection)</li>
<li>Unplug the ESP32 from the laptop</li>
<li>Now you may close the terminal/connection and analyse the data</li>
</ol>

# Data Analysis
The first step before starting data analysis is to segment the CSV file downloaded from the onboard microcontroller. The main CSV file should be split into two separate CSV files, one with all the GPS points/GPS data and one with the PID loop data. This is because the PID and GPS logging softwares cannot handle mixed data and must receive their respective data in seperate CSVs. 

## PID Graphing
There are two Python scripts that can be used to plot the PID graphs: the data_plotter and the simplified_data_plotter. The data_plotter should be used for any new data obtained. The simplified plotter is needed to graph some of the older data, as the CSV was more sparse, which causes errors if the newer data_plotter script is used. 

## GPS Driving Logging
Google My Maps is used to render the driven GPS points. However, several scripts are provided to add the heading lines and to remove duplicate coordinates (which the raw data will have). The GPS CSV should first be manually cleaned by removing the start and end text lines. It should then be passed into the GPS cleaner, which removes all duplicates. The cleaned CSV should then be passed into the KML converter, which will generate 2 KML files. It will generate a KML file with each GPS point, and a KML file with two heading lines for each point. They should each be imported separately into the same My Maps map, which allows the inbuilt My Maps feature to be used to label the GPS points in the order that the vessel traversed them.  
