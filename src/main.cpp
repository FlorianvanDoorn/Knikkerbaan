
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
 


void setup() 
{
  // Start serial communication to log via serial port.
  Serial.begin(115200);
}

void loop() {

  if (Serial.available() > 0) 
  {
    char inChar = Serial.read(); // Read the incoming byte.

    if (inChar == START_CHAR) 
    {
      BufIndex = 0; // Reset buffer index when start character is received.
    } 
    else if (inChar == END_CHAR) 
    {
      buffer[BufIndex] = '\0'; // Null-terminate the string.
      ActPos = atof(buffer); // Convert the buffer to a float and store it in ActPos.
      Serial.print("Received Position: ");
      Serial.println(ActPos); // Log the received position for debugging.
    } 
    else if (BufIndex < sizeof(buffer) - 1) 
    {
      buffer[BufIndex++] = inChar; // Add character to buffer and increment index.
    }
  }


}