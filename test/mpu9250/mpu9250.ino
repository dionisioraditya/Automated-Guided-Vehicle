#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void setup(void)
{
  Serial.begin(9600);
  Serial.println("HMC5883 Magnetometer Test"); Serial.println("");
 
  /* Initialise the sensor */
  if(!mag.begin())
  {
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
}

void loop() {
    sensors_event_t event;
    mag.getEvent(&event);
    Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");
    float heading = atan2(event.magnetic.y, event.magnetic.x);
    float declinationAngle = 0.663;
    heading += declinationAngle;
    Serial.print("Heading (degrees): ");
    Serial.println(heading);
    delay(500);

}