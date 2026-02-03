#include <ICM_20948.h> //IMU library
#include <Arduino.h>
#include "controller.h"

/*
IMU setup
*/
#define SERIAL_PORT Serial

#define SPI_PORT SPI
#define CS_PIN 2

#ifdef USE_SPI
ICM_20948_SPI myICM; //usage of SPI gives 1-8kHz refresh rate on IMU
#endif
/*
end IMU setup
*/

/*
controller setup
*/
constexpr int HIST_SIZE = 100;
float histZ[HIST_SIZE]; //Z
float histR[HIST_SIZE]; //Roll
float histP[HIST_SIZE]; //Pitch
float histW[HIST_SIZE]; //Yaw

constexpr float dt = 0.001f;

// Z gains
constexpr float kpZ = 0.0f;
constexpr float kiZ = 0.0f;
constexpr float kdZ = 0.0f;

//Roll gains
constexpr float kpR = 0.0f;
constexpr float kiR = 0.0f;
constexpr float kdR = 0.0f;

//Pitch gains
constexpr float kpP = 0.0f;
constexpr float kiP = 0.0f;
constexpr float kdP = 0.0f;

//Yaw gains
constexpr float kpW = 0.0f;
constexpr float kiW = 0.0f;
constexpr float kdW = 0.0f;

PID PIDz(kpZ, kiZ, kdZ, dt);
PID PIDr(kpR, kiR, kdR, dt);
PID PIDp(kpP, kiP, kdP, dt);
PID PIDw(kpW, kiW, kdW, dt);
/*
end controller setup
*/


/* 
============== program begin ====================
*/ 


void setup() {
  Serial.begin(115200);

  while (!SERIAL_PORT);
  {
    // wait for serial port response
  }

  #ifdef USE_SPI
    SPI_PORT.begin(); //initialize IMU communication
  #else
    Serial.println("Failed to start SPI");
  #endif

  bool initialized = false;
  while (!initialized)
  {
    myICM.begin(CS_PIN, SPI_PORT);

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
      SERIAL_PORT.println("Success");
    }
  }

  // fill empty position history lists
  for (int i = 0; i < HIST_SIZE; i++) 
  {
    histZ[i] = 0.0f;
    histR[i] = 0.0f;
    histP[i] = 0.0f;
    histW[i] = 0.0f;
  }
}

/*
================ MAIN ================================
*/

void loop() {

  /*
  IMPORTANT:
  ADD EMERGENCY STOP BUTTON TO CUT ALL POWER IN CASE OF PID RUNAWAY OR IMMANENT CRASHES
  */

  if (myICM.dataReady())
  {
    myICM.getAGMT();
    //print raw data:
    printRawAGMT(myICM.agmt);
    Serial.println(myICM.agmt);
    printScaledAGMT(&myICM);
    delay(30);
  }




  static unsigned long lastMicros = 0;
  unsigned long now = micros();

  if (now - lastMicros >= (unsigned long)(dt * 1e6)) {
    unsigned float lastMicros = now;

    // READ CURRENT POSITION FROM IMU FOR IMPORTANT POSITIONS

    // FUNTION TO UPDATE POSITION HISTORIES (MOVE EVERYTHING UP BY ONE PLACE APPEND CURRENT TO END)

    // RUN COMPUTE
    PIDz.compute(histZ, des, &out);


    // calc control vector

    //send control vector
  }

}
