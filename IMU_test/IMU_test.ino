#include <ICM_20948.h>
#include <Arduino.h>

#define SERIAL_PORT Serial

#define SPI_PORT SPI
#define CS_PIN 2

ICM_20948_SPI myICM; //create ICM object

void setup() {
  Serial.begin(115200);

  while (!SERIAL_PORT)
  {
    // wait for serial port response
  }

  SPI_PORT.begin();

  bool initialized = false;
  while (!initialized)
  {
    myICM.begin(CS_PIN, SPI_PORT);

    SERIAL_PORT.print(F("Initialization of sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again....");
      delay(500);
    } else {
      initalized = true;
      SERIAL_PORT.println("Success");
    }
  }

}

void loop() {
  if (myICM.dataReady())
  {
    myICM.getAGMT();
    printRawAGMT(myICM.agmt);
    Serial.println(myICM.agmt);
    printScaledAGMT(&myICM);
    delay(30);
  }

}
