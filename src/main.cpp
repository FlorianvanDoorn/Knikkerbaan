
// +--------------------------------------------------------------------------------------------------+
// | Knikkerbaan.ino                                                                                  |
// |                                                                                                  |
// | F. van Doorn                                                                                     |
// | Student Nr. 2223288                                                                              |
// | Academie voor Deeltijd                                                                           |
// | Avans Hogeschool Breda                                                                           |
// |                                                                                                  |
// | Knikkerbaan met Goede filtereigenschappen en een werkende regelkring.                            |
// |                                                                                                  |
// | Libraries:                                                                                       |
// |  - Arduino.h                                                                                     | 
// |                                                                                                  |
// | v0.1 - 07.02.2026                                                                                |
// +--------------------------------------------------------------------------------------------------+

#include <Arduino.h>



// Define communication variables.
#define START_CHAR '$'
#define END_CHAR   '*'
char buffer[32];
uint8_t BufIndex = 0;


float ActPos = 0; // Actual position of the knikker, received from the sensor. This variable will be used in the control loop to adjust the speed of the motor accordingly.
 
float GetPos(); // Function prototype for retrieving the actual position.

void setup() 
{
  // Start serial communication to log via serial port.
  Serial.begin(115200);
}

void loop() {


  ActPos = GetPos(); // Call the function to get the actual position of the knikker.
  Serial.print("Actual Position: ");
  Serial.println(ActPos); // Print the actual position to the serial monitor for debugging purposes



}


float GetPos() 
{
  static bool receiving = false;
  static float Pos = 0.0; // veiliger dan 0

  while (Serial.available())
  {
    char inChar = Serial.read();

    if (inChar == START_CHAR) {
      BufIndex = 0;
      receiving = true;
    } 
    else if (inChar == END_CHAR && receiving) {
      buffer[BufIndex] = '\0';
      Pos = atof(buffer);
      receiving = false;
      return Pos;
    } 
    else if (receiving && BufIndex < sizeof(buffer) - 1) {
      buffer[BufIndex++] = inChar;
    }
  }

  return Pos;
}