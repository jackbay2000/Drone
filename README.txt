PART LIST:
CL53L1X TOF400C Laser Ranging Sensor Module
PMW3901 Optical Flow Sensor
MPU-6050 IMU
2205 2300KV 5045 Propellers with 30A ESCs
Arduino Teensy 4.1
5V Buck Converter Board
320A PDB
3S Lipo Battery

Sensor integration:
IMU:
	attitude only
	Shifted 90 degrees in x-y plane relative to optical flow sensor and drone origin
	Added median filter to account for sudden ~4.0 inch inch accel in x and y

Optical Flow Sensor:
	No Changes needed yet
	Output raw data from lib to main

Rangefinder:
	Only 50 Hz, limit max Z vel
	Raw data, output to main


Controller Interface:
	In progress...
	Standard PID with attitude controller
	outfit with max roll/pitch -> max y/x vel

Props:
	1000 to 2000 us PWM range
	
	
