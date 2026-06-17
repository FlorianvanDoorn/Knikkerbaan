// +----------------------------------------------------------------------------------+
// | Project:      Knikkerbaan Regelsysteem                                           |
// | Bestand:      Knikkerbaan.ino                                                    |
// | Auteur:       Florian van Doorn                                                  |
// | Opleiding:    Mechatronica - Avans Hogeschool Breda                              |
// | Datum:        28-05-2026                                                         |
// | Versie:                                                                          |
// |                                                                                  |
// | Beschrijving:                                                                    |
// | Stuurt commando's naar de servo motor op basis van de PID regelkring             |
// |                                                                                  |
// | Functionaliteit:                                                                 |
// | - Regelkring                                                                     |
// | - PID regelaar                                                                   |
// | - Sensor ingangsdata verschaling                                                 |
// | - Data filtering                                                                 |
// | -                                                                                |
// |                                                                                  |
// | Libraries:                                                                       |
// | - Arduino.h                                                                      |
// |                                                                                  |
// +----------------------------------------------------------------------------------+

#include <Arduino.h>

// Pins inputs
#define SensorPin 34

// Pins outputs
#define ServoPin 18

// Define global variables.
float DesirPos = 100.0;
// float         voltage       = 0.0;
float ActPos = 0.0;
float RawDist = 0.0;
double ActSpeed = 0.0;
double ActRawSpeed = 0.0;
int Mode = 0; // 0 = Sensor, 1 = Vision

int ServoChannel = 0;

// Define filter variables.
unsigned int SampleTime = 1;    // Sample time in ms. [1]
unsigned int SampleAmount = 20; // Amount of samples used to filter the input signal. 4+1=5 zero counts as one sample position. [3]
int SpeedSampleTime = 1;        // Sample time for speed calculation in ms. [3]
float PrevSample[20];
float TauDistance = 6; // Time constant for first order filter. [5]
float TauSpeed = 10;   // Time constant for first order filter. [15]

// Define communication variables.
#define START_CHAR '$'
#define END_CHAR '*'
#define DELIMITER ','
char buffer[32];
uint8_t BufIndex = 0;

// PID variables.
float Error = 0.0;           // Actual difference between Desired and actual position.
float Kp = 0.035;            // Proportional cotroller variable. [0.035]
float Td = 0.6;              // Differentiating controller variable. [0.6]
unsigned int DesirAngle = 0; // Result of controller.

// Define constants.
const int pwmFreq = 50;
const int pwmResolution = 16;
const float g = 9.81;

uint32_t DutyMin = 1638; // ≈ 0.5 ms
uint32_t DutyMax = 8192; // ≈ 2.5 ms

// Define function
void SensMapping();
void DistFilter();
void FirstOrderDistanceFilter();
void FirstOrderSpeedFilter();
void CalculateSpeed();
void GetData();

void setup()
{
  // Set parameters for servo controls

  ledcSetup(ServoChannel, pwmFreq, pwmResolution);
  ledcAttachPin(ServoPin, ServoChannel);

  // Start serial communication to log via serial port.
  Serial.begin(115200);
}

void loop()
{

  // Calculate error.
  Error = DesirPos - ActPos;

  // Calculate Desired Angle of knikkerbaan.
  DesirAngle = (Error - ActSpeed * Td) * Kp * g; // (Error - ActSpeed * Td) * Kp * g

  // Send servo to desired angle.
  ledcWrite(ServoChannel, (DutyMin + ((DutyMax - DutyMin) * (88 - DesirAngle) / 180)));

  Serial.print(">RawPosition: ");
  Serial.println(RawDist);
  Serial.print(">ActualPosition: ");
  Serial.println(ActPos);

  Serial.print(">RawSpeed: ");
  Serial.println(ActRawSpeed);
  Serial.print(">ActualSpeed: ");
  Serial.println(ActSpeed);

  // Serial.println(String (voltage));

  // Call function to read Actual position from serial input.
  GetData();

  if (Mode == 0)
  {

    // Call Function for scaling sensor signal.
    SensMapping();

    // Call Function for Filtering sensor signal.
    // DistFilter();

    // Call Function for Filtering sensor signal with a first order filter.
    FirstOrderDistanceFilter();
  }

  // Call Function for calculating actual speed.
  CalculateSpeed();

  // Call Function for Filtering speed signal with a first order filter.
  FirstOrderSpeedFilter();
}

void SensMapping()
{

  float voltage = 0.0;

  // Read analog value from sensor.
  RawDist = analogRead(SensorPin);

  voltage = RawDist * 3.3 / 4095.0;

  // Scale input voltage to distance in millimeters.
  RawDist = 111.0 / (voltage - 0.13) - 36.8;
}

void FirstOrderDistanceFilter()
{

  // Declare local variables.
  static int PrevMillis;

  // Run code if sample time has passed.
  if (millis() > (PrevMillis + SampleTime))
  {

    // Calculate actual position as a result of previous position and actual raw distance.
    ActPos = ActPos + (RawDist - ActPos) * (SampleTime / TauDistance);

    // Store actual runtime
    PrevMillis = millis();
  }
}

void FirstOrderSpeedFilter()
{

  // Declare local variables.
  static int PrevMillis;

  // Run code if sample time has passed.
  if (millis() > (PrevMillis + SpeedSampleTime))
  {

    // Calculate actual position as a result of previous position and actual raw distance.
    ActSpeed = ActSpeed + (ActRawSpeed - ActSpeed) * (SampleTime / TauSpeed);

    // Store actual runtime
    PrevMillis = millis();
  }
}

// Sensor input value to actual distance.
void DistFilter()
{

  // Declare local variables.
  static int PrevMillis;
  static float SampleSum;

  // Run code if sample time has passed.
  if (millis() > (PrevMillis + SampleTime))
  {

    // Shift values of previous samples in register.
    for (int i = (SampleAmount - 1); i > 0; i--)
    {
      PrevSample[i] = PrevSample[i - 1];
    }

    // Store actual position in shift register.
    PrevSample[0] = RawDist;

    // Make sum of all stored samples
    for (int i = 0; i < SampleAmount; i++)
    {
      SampleSum = SampleSum + PrevSample[i];
    }

    // Calculate mean position as a result of stored previous positions
    ActPos = SampleSum / static_cast<float>(SampleAmount);

    SampleSum = 0.0; // Reset sample sum for next calculation.

    // Store actual runtime
    PrevMillis = millis();
  }
}

void CalculateSpeed()
{

  // Declare local variables
  static int PrevMillis;
  static double PosDiff = 0.0;
  static double TimeDiff = 0.0;
  static float PrevPos = 0.0;
  static float PrevTime = 0.0;

  if (millis() > (PrevMillis + SpeedSampleTime))
  {

    // Calculate Position Differential in millimeters.
    PosDiff = ActPos - PrevPos;
    // Store actual position for next calculation.
    PrevPos = ActPos;

    // Calculate Time Differential in seconds.
    TimeDiff = millis() - PrevTime;
    // Store actual time for next calculation.
    PrevTime = millis();

    // Calculate actual speed.
    ActRawSpeed = PosDiff * 1000 / TimeDiff;

    // Store actual runtime
    PrevMillis = millis();
  }
}

void GetData()
{
  static bool receiving = false;
  String VariableName;

  while (Serial.available())
  {
    char inChar = Serial.read();

    if (inChar == START_CHAR)
    {
      BufIndex = 0;
      receiving = true;
    }
    else if (inChar == END_CHAR && receiving)
    {
      buffer[BufIndex] = '\0';
      if (VariableName == "Actpos")
      {
        ActPos = atof(buffer);
        Serial.print(">Received Position: ");
        Serial.println(ActPos);
        receiving = false;
        return;
      }
      else if (VariableName == "Desirpos")
      {
        DesirPos = atof(buffer);
        Serial.print(">Received Desired Position: ");
        Serial.println(DesirPos);
        receiving = false;
        return;
      }
      else if (VariableName == "Kp")
      {
        Kp = atof(buffer);
        Serial.print(">Received Kp: ");
        Serial.println(Kp);
        receiving = false;
        return;
      }
      else if (VariableName == "Td")
      {
        Td = atof(buffer);
        Serial.print(">Received Td: ");
        Serial.println(Td);
        receiving = false;
        return;
      }
      else if (VariableName == "Mode")
      {
        Mode = atof(buffer);
        Serial.print(">Received Mode: ");
        Serial.println(Mode);
        receiving = false;
        return;
      }
      else
      {
        Serial.print(">Unknown Variable Name: ");
        Serial.println(VariableName);
        receiving = false;
        return;
      }
    }
    else if (inChar == DELIMITER && receiving)
    {
      buffer[BufIndex] = '\0';
      VariableName = String(buffer);
      BufIndex = 0; // reset buffer index for next variable
    }
    else if (receiving && BufIndex < sizeof(buffer) - 1)
    {
      buffer[BufIndex++] = inChar;
    }
  }

  return;
}