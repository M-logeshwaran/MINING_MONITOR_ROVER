/*
  ==============================================================
  DRILLPULSE ROVER - FINAL PRIORITY / NON-BLOCKING CONTROLLER
  ==============================================================

  PURPOSE
  -------
  Designed for Arduino Uno + HC-05 + SoftwareSerial.

  Priority:
    1. Bluetooth motor commands
    2. Motor safety
    3. Non-blocking sensors
    4. Sensor telemetry when command traffic is idle

  IMPORTANT:
    Arduino Uno is single-core and SoftwareSerial is not true
    hardware full-duplex. This sketch therefore gives incoming
    motor commands priority over outgoing sensor telemetry.

  NO delay()
  NO pulseIn()
  NO readStringUntil()
  NO blocking Bluetooth parser

  ==============================================================
  HARDWARE
  ==============================================================

  HC-05:
    TXD -> Arduino D10
    RXD -> Arduino D11
    VCC -> 5V
    GND -> GND

  MQ-4:
    Analog  -> A0
    Digital -> D2

  DHT22:
    DATA -> A1

  LEFT ULTRASONIC:
    TRIG -> A2
    ECHO -> A3

  RIGHT ULTRASONIC:
    TRIG -> A4
    ECHO -> A5

  MOTORS:
    M1 = Front Left
    M2 = Rear Left
    M3 = Front Right
    M4 = Rear Right

  ==============================================================
  SPEED MODES
  ==============================================================

    0 = SLOW   = 40%
    1 = NORMAL = 70%
    2 = TURBO  = 100%

  ==============================================================
  COMMAND PROTOCOL
  ==============================================================

  Preferred:
    <CMD,seq,x,y,speed>

  Example:
    <CMD,125,0.000,1.000,2>

  Current ROS2 protocol is also accepted:
    CMD,seq,x,y,speed\n

  Also accepted:
    CMD,x,y,speed\n

  ==============================================================
  SENSOR TELEMETRY
  ==============================================================

  S,TEMP,HUMIDITY,MQ4_ANALOG,MQ4_DIGITAL,LEFT_DISTANCE,RIGHT_DISTANCE\n

  Example:
    S,0.00,0.00,60,0,12.4,9.8

  ==============================================================
  MOTOR SAFETY
  ==============================================================

  A valid movement command continuously refreshes the watchdog.

  If the PC/Bluetooth stops producing valid commands while the
  rover is moving, the Arduino forces RELEASE after
  MOTOR_FAILSAFE_MS.

  STOP command:
    <CMD,seq,0.000,0.000,speed>

  immediately releases all motors.

  ==============================================================
*/

#include <AFMotor_R4.h>
#include <DHT.h>
#include <SoftwareSerial.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// ============================================================
// BLUETOOTH
// ============================================================

SoftwareSerial Bluetooth(10, 11);  // RX, TX
const unsigned long BT_BAUD = 9600;

// ============================================================
// SENSOR PINS
// ============================================================

#define MQ4_ANALOG   A0
#define MQ4_DIGITAL  2

#define DHTPIN       A1
#define DHTTYPE      DHT22

#define LEFT_TRIG    A2
#define LEFT_ECHO    A3

#define RIGHT_TRIG   A4
#define RIGHT_ECHO   A5

DHT dht(DHTPIN, DHTTYPE);

// ============================================================
// MOTORS
// ============================================================

AF_DCMotor motor1(1);  // Front Left
AF_DCMotor motor2(2);  // Rear Left
AF_DCMotor motor3(3);  // Front Right
AF_DCMotor motor4(4);  // Rear Right

// ============================================================
// SPEED MODES
// ============================================================

enum SpeedMode : uint8_t
{
  SPEED_SLOW   = 0,
  SPEED_NORMAL = 1,
  SPEED_TURBO  = 2
};

SpeedMode currentSpeedMode = SPEED_NORMAL;

const uint8_t PWM_SLOW   = 102;
const uint8_t PWM_NORMAL = 178;
const uint8_t PWM_TURBO  = 255;

// ============================================================
// GLOBAL TIMERS
// ============================================================

const unsigned long SENSOR_UPDATE_MS = 50;
const unsigned long TELEMETRY_MS = 100;
const unsigned long DHT_UPDATE_MS = 2000;
const unsigned long DEBUG_UPDATE_MS = 2000;

/*
  This is ONLY the emergency motor timeout.

  It is deliberately much shorter than 1 second so a disconnected
  controller cannot leave the rover driving.

  It must still be longer than the normal ROS2 moving keepalive.
*/
const unsigned long MOTOR_FAILSAFE_MS = 450;

/*
  Do not add a post-command quiet period. RX is serviced first on every
  loop, and telemetry checks for waiting command bytes before transmitting.
*/
const unsigned long COMMAND_TRAFFIC_IDLE_MS = 0;

unsigned long lastValidCommandMs = 0;
unsigned long lastCommandActivityMs = 0;

unsigned long lastSensorUpdateMs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastDhtUpdateMs = 0;
unsigned long lastDebugMs = 0;

// ============================================================
// SENSOR VALUES
// ============================================================

float temperature = 0.0f;
float humidity = 0.0f;

int mq4Analog = 0;
int mq4Digital = 0;

float leftDistance = -1.0f;
float rightDistance = -1.0f;

bool dhtValid = false;

// ============================================================
// DRIVE STATE
// ============================================================

float commandX = 0.0f;
float commandY = 0.0f;

bool motorsRunning = false;
bool movementActive = false;
bool fullControlActive = false;

// ============================================================
// BLUETOOTH RX BUFFER
// ============================================================

const uint8_t RX_BUFFER_SIZE = 80;

char rxBuffer[RX_BUFFER_SIZE];
uint8_t rxIndex = 0;

bool framedMode = false;
bool legacyMode = false;

// ============================================================
// NON-BLOCKING ULTRASONIC STATE MACHINE
// ============================================================

enum UltraState : uint8_t
{
  ULTRA_IDLE = 0,

  LEFT_TRIGGER,
  LEFT_WAIT_RISE,
  LEFT_WAIT_FALL,

  RIGHT_TRIGGER,
  RIGHT_WAIT_RISE,
  RIGHT_WAIT_FALL
};

UltraState ultraState = ULTRA_IDLE;

unsigned long ultraTimerUs = 0;
unsigned long echoStartUs = 0;
unsigned long nextUltraUs = 0;

const unsigned long ULTRA_TRIGGER_US = 10;
const unsigned long ULTRA_TIMEOUT_US = 8000;
const unsigned long ULTRA_GAP_US = 700;

// ============================================================
// SPEED -> PWM
// ============================================================

int getSpeedPWM()
{
  switch (currentSpeedMode)
  {
    case SPEED_SLOW:
      return PWM_SLOW;

    case SPEED_NORMAL:
      return PWM_NORMAL;

    case SPEED_TURBO:
      return PWM_TURBO;

    default:
      return PWM_NORMAL;
  }
}

// ============================================================
// STOP MOTORS
// ============================================================

void stopMotors()
{
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);

  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);

  motorsRunning = false;
  movementActive = false;
}

// ============================================================
// APPLY MOTOR COMMAND
// ============================================================

void applyMotorCommand(float x, float y)
{
  const float DEADZONE = 0.15f;

  if (fabs(x) < DEADZONE)
    x = 0.0f;

  if (fabs(y) < DEADZONE)
    y = 0.0f;

  // ----------------------------------------------------------
  // STOP
  // ----------------------------------------------------------

  if (x == 0.0f && y == 0.0f)
  {
    stopMotors();
    return;
  }

  int pwm = getSpeedPWM();

  // ----------------------------------------------------------
  // FORWARD
  // ----------------------------------------------------------

  if (y > 0.5f && fabs(x) < 0.5f)
  {
    motor1.setSpeed(pwm);
    motor2.setSpeed(pwm);
    motor3.setSpeed(pwm);
    motor4.setSpeed(pwm);

    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);

    motorsRunning = true;
    movementActive = true;
    return;
  }

  // ----------------------------------------------------------
  // BACKWARD
  // ----------------------------------------------------------

  if (y < -0.5f && fabs(x) < 0.5f)
  {
    motor1.setSpeed(pwm);
    motor2.setSpeed(pwm);
    motor3.setSpeed(pwm);
    motor4.setSpeed(pwm);

    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);

    motorsRunning = true;
    movementActive = true;
    return;
  }

  // ----------------------------------------------------------
  // RIGHT
  // ----------------------------------------------------------

  if (x > 0.5f && fabs(y) < 0.5f)
  {
    motor1.setSpeed(pwm);
    motor2.setSpeed(pwm);
    motor3.setSpeed(pwm);
    motor4.setSpeed(pwm);

    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);

    motorsRunning = true;
    movementActive = true;
    return;
  }

  // ----------------------------------------------------------
  // LEFT
  // ----------------------------------------------------------

  if (x < -0.5f && fabs(y) < 0.5f)
  {
    motor1.setSpeed(pwm);
    motor2.setSpeed(pwm);
    motor3.setSpeed(pwm);
    motor4.setSpeed(pwm);

    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);

    motorsRunning = true;
    movementActive = true;
    return;
  }

  // ----------------------------------------------------------
  // DIFFERENTIAL / MIXED
  // ----------------------------------------------------------

  float leftPower = constrain(y + x, -1.0f, 1.0f);
  float rightPower = constrain(y - x, -1.0f, 1.0f);

  int leftPWM = (int)(fabs(leftPower) * pwm);
  int rightPWM = (int)(fabs(rightPower) * pwm);

  motor1.setSpeed(leftPWM);
  motor2.setSpeed(leftPWM);
  motor3.setSpeed(rightPWM);
  motor4.setSpeed(rightPWM);

  if (leftPower > DEADZONE)
  {
    motor1.run(FORWARD);
    motor2.run(FORWARD);
  }
  else if (leftPower < -DEADZONE)
  {
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
  }
  else
  {
    motor1.run(RELEASE);
    motor2.run(RELEASE);
  }

  if (rightPower > DEADZONE)
  {
    motor3.run(FORWARD);
    motor4.run(FORWARD);
  }
  else if (rightPower < -DEADZONE)
  {
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
  }
  else
  {
    motor3.run(RELEASE);
    motor4.run(RELEASE);
  }

  motorsRunning = (leftPWM > 0 || rightPWM > 0);
  movementActive = motorsRunning;
}

// ============================================================
// FLOAT PARSER
// ============================================================

bool parseFloatField(const char *text, float &value)
{
  if (text == NULL || *text == '\0')
    return false;

  char *endPtr = NULL;

  value = (float)strtod(text, &endPtr);

  if (endPtr == text)
    return false;

  if (*endPtr != '\0')
    return false;

  if (!isfinite(value))
    return false;

  return true;
}

// ============================================================
// COMMAND PARSER
//
// Accepts:
//
//   CMD,x,y,speed
//
//   CMD,seq,x,y,speed
//
// ============================================================

bool parseFullControl(char *frame)
{
  if (strncmp(frame, "FULL,", 5) != 0)
    return false;

  char *endPtr = NULL;
  long mode = strtol(frame + 5, &endPtr, 10);

  if (endPtr == frame + 5 || *endPtr != '\0' || (mode != 0 && mode != 1))
    return false;

  bool wasFullControlActive = fullControlActive;
  fullControlActive = (mode == 1);
  unsigned long now = millis();
  lastCommandActivityMs = now;

  if (fullControlActive != wasFullControlActive)
  {
    // Discard the complete sensor snapshot so FC cannot leak old readings.
    temperature = 0.0f;
    humidity = 0.0f;
    mq4Analog = 0;
    mq4Digital = 0;
    leftDistance = -1.0f;
    rightDistance = -1.0f;
    dhtValid = false;
    lastTelemetryMs = now;
  }

  if (fullControlActive)
    ultraState = ULTRA_IDLE;

  Serial.print("FULL_CONTROL=");
  Serial.println(fullControlActive ? "ON" : "OFF");
  return true;
}

bool parseCommand(char *frame)
{
  if (strncmp(frame, "CMD,", 4) != 0)
    return false;

  char work[RX_BUFFER_SIZE];

  strncpy(work, frame + 4, RX_BUFFER_SIZE - 1);
  work[RX_BUFFER_SIZE - 1] = '\0';

  char *fields[5];
  uint8_t count = 0;

  char *token = strtok(work, ",");

  while (token != NULL && count < 5)
  {
    fields[count++] = token;
    token = strtok(NULL, ",");
  }

  // Too many fields.
  if (token != NULL)
    return false;

  float x = 0.0f;
  float y = 0.0f;
  long speed = -1;

  // ----------------------------------------------------------
  // CMD,x,y,speed
  // ----------------------------------------------------------

  if (count == 3)
  {
    if (!parseFloatField(fields[0], x))
      return false;

    if (!parseFloatField(fields[1], y))
      return false;

    char *endPtr = NULL;

    speed = strtol(fields[2], &endPtr, 10);

    if (endPtr == fields[2] || *endPtr != '\0')
      return false;
  }

  // ----------------------------------------------------------
  // CMD,seq,x,y,speed
  // ----------------------------------------------------------

  else if (count == 4)
  {
    char *endPtr = NULL;

    long sequence = strtol(fields[0], &endPtr, 10);

    if (endPtr == fields[0] || *endPtr != '\0')
      return false;

    (void)sequence;

    if (!parseFloatField(fields[1], x))
      return false;

    if (!parseFloatField(fields[2], y))
      return false;

    endPtr = NULL;

    speed = strtol(fields[3], &endPtr, 10);

    if (endPtr == fields[3] || *endPtr != '\0')
      return false;
  }
  else
  {
    return false;
  }

  // ----------------------------------------------------------
  // SPEED VALIDATION
  // ----------------------------------------------------------

  if (speed < 0 || speed > 2)
    return false;

  // ----------------------------------------------------------
  // STORE
  // ----------------------------------------------------------

  commandX = constrain(x, -1.0f, 1.0f);
  commandY = constrain(y, -1.0f, 1.0f);

  currentSpeedMode = (SpeedMode)speed;

  unsigned long now = millis();

  lastValidCommandMs = now;
  lastCommandActivityMs = now;

  // ----------------------------------------------------------
  // APPLY IMMEDIATELY
  // ----------------------------------------------------------

  applyMotorCommand(commandX, commandY);

  // ----------------------------------------------------------
  // USB DIAGNOSTIC
  // ----------------------------------------------------------

  Serial.print("RX_OK x=");
  Serial.print(commandX, 3);

  Serial.print(" y=");
  Serial.print(commandY, 3);

  Serial.print(" speed=");
  Serial.print((int)currentSpeedMode);

  Serial.print(" pwm=");
  Serial.print(getSpeedPWM());

  Serial.print(" moving=");
  Serial.print(movementActive ? 1 : 0);

  Serial.print(" t=");
  Serial.println(now);

  return true;
}

// ============================================================
// BLUETOOTH RECEIVE
//
// Framed protocol:
//   <CMD,...>
//
// Compatibility:
//   CMD,...\n
//
// Incoming data is handled one byte at a time.
// ============================================================

void serviceBluetoothRx()
{
  while (Bluetooth.available())
  {
    char c = (char)Bluetooth.read();

    lastCommandActivityMs = millis();

    // --------------------------------------------------------
    // START MARKER
    // --------------------------------------------------------

    if (c == '<')
    {
      framedMode = true;
      legacyMode = false;
      rxIndex = 0;
      rxBuffer[0] = '\0';

      continue;
    }

    // --------------------------------------------------------
    // FRAMED MODE
    // --------------------------------------------------------

    if (framedMode)
    {
      if (c == '>')
      {
        rxBuffer[rxIndex] = '\0';

        bool ok = (strncmp(rxBuffer, "FULL,", 5) == 0)
              ? parseFullControl(rxBuffer)
              : parseCommand(rxBuffer);

        if (!ok)
        {
          Serial.print("RX_FRAME_BAD <");
          Serial.print(rxBuffer);
          Serial.println(">");
        }

        framedMode = false;
        rxIndex = 0;

        continue;
      }

      if (rxIndex < RX_BUFFER_SIZE - 1)
      {
        rxBuffer[rxIndex++] = c;
      }
      else
      {
        rxIndex = 0;
        framedMode = false;

        Serial.println("RX_FRAME_OVERFLOW");
      }

      continue;
    }

    // --------------------------------------------------------
    // LEGACY MODE
    // Only begins on 'C', reducing random garbage parsing.
    // --------------------------------------------------------

    if (!legacyMode)
    {
      if (c == 'C' || c == 'F')
      {
        legacyMode = true;
        rxIndex = 0;
        rxBuffer[rxIndex++] = c;
      }

      continue;
    }

    // --------------------------------------------------------

    if (legacyMode)
    {
      if (c == '\n' || c == '\r')
      {
        if (rxIndex > 0)
        {
          rxBuffer[rxIndex] = '\0';

          bool ok = (strncmp(rxBuffer, "FULL,", 5) == 0)
                      ? parseFullControl(rxBuffer)
                      : parseCommand(rxBuffer);

          if (!ok)
          {
            Serial.print("RX_LEGACY_BAD ");
            Serial.println(rxBuffer);
          }
        }

        legacyMode = false;
        rxIndex = 0;

        continue;
      }

      if (rxIndex < RX_BUFFER_SIZE - 1)
      {
        rxBuffer[rxIndex++] = c;
      }
      else
      {
        legacyMode = false;
        rxIndex = 0;

        Serial.println("RX_LEGACY_OVERFLOW");
      }
    }
  }
}

// ============================================================
// NON-BLOCKING ULTRASONIC
//
// No pulseIn().
// No delay().
// ============================================================

void serviceUltrasonic()
{
  unsigned long nowUs = micros();

  switch (ultraState)
  {
    // --------------------------------------------------------
    // START LEFT
    // --------------------------------------------------------

    case ULTRA_IDLE:

      if ((long)(nowUs - nextUltraUs) >= 0)
      {
        digitalWrite(LEFT_TRIG, HIGH);

        ultraTimerUs = nowUs;

        ultraState = LEFT_TRIGGER;
      }

      break;

    // --------------------------------------------------------
    // LEFT TRIGGER
    // --------------------------------------------------------

    case LEFT_TRIGGER:

      if ((unsigned long)(nowUs - ultraTimerUs) >=
          ULTRA_TRIGGER_US)
      {
        digitalWrite(LEFT_TRIG, LOW);

        ultraTimerUs = nowUs;

        ultraState = LEFT_WAIT_RISE;
      }

      break;

    // --------------------------------------------------------
    // LEFT WAIT RISE
    // --------------------------------------------------------

    case LEFT_WAIT_RISE:

      if (digitalRead(LEFT_ECHO) == HIGH)
      {
        echoStartUs = micros();

        ultraState = LEFT_WAIT_FALL;
      }
      else if ((unsigned long)(nowUs - ultraTimerUs) >=
               ULTRA_TIMEOUT_US)
      {
        leftDistance = -1.0f;

        nextUltraUs = micros() + ULTRA_GAP_US;

        ultraState = RIGHT_TRIGGER;
      }

      break;

    // --------------------------------------------------------
    // LEFT WAIT FALL
    // --------------------------------------------------------

    case LEFT_WAIT_FALL:

      if (digitalRead(LEFT_ECHO) == LOW)
      {
        unsigned long pulseWidth =
            micros() - echoStartUs;

        float distance =
            pulseWidth * 0.0343f / 2.0f;

        if (distance >= 2.0f && distance <= 300.0f)
          leftDistance = distance;
        else
          leftDistance = -1.0f;

        nextUltraUs = micros() + ULTRA_GAP_US;

        ultraState = RIGHT_TRIGGER;
      }
      else if ((unsigned long)(nowUs - echoStartUs) >=
               ULTRA_TIMEOUT_US)
      {
        leftDistance = -1.0f;

        nextUltraUs = micros() + ULTRA_GAP_US;

        ultraState = RIGHT_TRIGGER;
      }

      break;

    // --------------------------------------------------------
    // START RIGHT
    // --------------------------------------------------------

    case RIGHT_TRIGGER:

      if ((long)(nowUs - nextUltraUs) >= 0)
      {
        digitalWrite(RIGHT_TRIG, HIGH);

        ultraTimerUs = nowUs;

        ultraState = RIGHT_WAIT_RISE;
      }

      break;

    // --------------------------------------------------------
    // END RIGHT TRIGGER
    // --------------------------------------------------------

    case RIGHT_WAIT_RISE:

      if ((unsigned long)(nowUs - ultraTimerUs) >=
          ULTRA_TRIGGER_US)
      {
        digitalWrite(RIGHT_TRIG, LOW);

        ultraTimerUs = nowUs;
        echoStartUs = 0;

        ultraState = RIGHT_WAIT_FALL;
      }

      break;

    // --------------------------------------------------------
    // RIGHT ECHO
    // --------------------------------------------------------

    case RIGHT_WAIT_FALL:

      // Waiting for echo rising edge.
      if (echoStartUs == 0)
      {
        if (digitalRead(RIGHT_ECHO) == HIGH)
        {
          echoStartUs = micros();
        }
        else if ((unsigned long)(nowUs - ultraTimerUs) >=
                 ULTRA_TIMEOUT_US)
        {
          rightDistance = -1.0f;

          nextUltraUs = micros() + ULTRA_GAP_US;

          ultraState = ULTRA_IDLE;
        }
      }
      else
      {
        // Waiting for echo falling edge.
        if (digitalRead(RIGHT_ECHO) == LOW)
        {
          unsigned long pulseWidth =
              micros() - echoStartUs;

          float distance =
              pulseWidth * 0.0343f / 2.0f;

          if (distance >= 2.0f && distance <= 300.0f)
            rightDistance = distance;
          else
            rightDistance = -1.0f;

          echoStartUs = 0;

          nextUltraUs = micros() + ULTRA_GAP_US;

          ultraState = ULTRA_IDLE;
        }
        else if ((unsigned long)(nowUs - echoStartUs) >=
                 ULTRA_TIMEOUT_US)
        {
          rightDistance = -1.0f;

          echoStartUs = 0;

          nextUltraUs = micros() + ULTRA_GAP_US;

          ultraState = ULTRA_IDLE;
        }
      }

      break;
  }
}

// ============================================================
// DHT SERVICE
// ============================================================

void serviceDHT()
{
  unsigned long now = millis();

  if ((unsigned long)(now - lastDhtUpdateMs) <
      DHT_UPDATE_MS)
  {
    return;
  }

  lastDhtUpdateMs = now;

  /*
    The standard DHT library uses timing-sensitive communication
    internally. It is called only every 2 seconds.

    Since your DHT22 is disconnected right now, TEMP/HUM remain
    at their initialized 0.00 values.
  */

  float t = dht.readTemperature();
  float h = dht.readHumidity();

  if (!isnan(t) && !isnan(h))
  {
    temperature = t;
    humidity = h;

    dhtValid = true;
  }
}

// ============================================================
// SENSOR UPDATE + TELEMETRY
// ============================================================

void serviceSensors()
{
  if (fullControlActive)
  {
    return;
  }

  unsigned long now = millis();

  if ((unsigned long)(now - lastSensorUpdateMs) <
      SENSOR_UPDATE_MS)
  {
    return;
  }

  lastSensorUpdateMs = now;

  // ----------------------------------------------------------
  // MQ4 - cheap/non-blocking
  // ----------------------------------------------------------

  mq4Analog = analogRead(MQ4_ANALOG);
  mq4Digital = digitalRead(MQ4_DIGITAL);

  // ----------------------------------------------------------
  // TELEMETRY PRIORITY RULE
  // ----------------------------------------------------------
  //
  // If rover is currently moving or a control packet was just
  // received, DO NOT transmit sensor telemetry.
  //
  // If the last command is a STOP/idle command, telemetry may
  // resume.
  //

  if (movementActive)
  {
    return;
  }

  // Never begin a SoftwareSerial TX while bytes are waiting in RX.
  if (Bluetooth.available())
  {
    return;
  }

  if ((unsigned long)(now - lastCommandActivityMs) <
      COMMAND_TRAFFIC_IDLE_MS)
  {
    return;
  }

  if ((unsigned long)(now - lastTelemetryMs) <
      TELEMETRY_MS)
  {
    return;
  }

  lastTelemetryMs = now;

  sendTelemetry();
}

// ============================================================
// SEND TELEMETRY
// ============================================================

void sendTelemetry()
{
  /*
    Compact packet reduces serial airtime.

    At 9600 baud, keeping packets short is important.
  */

  Bluetooth.print("S,");

  Bluetooth.print(temperature, 2);
  Bluetooth.print(",");

  Bluetooth.print(humidity, 2);
  Bluetooth.print(",");

  Bluetooth.print(mq4Analog);
  Bluetooth.print(",");

  Bluetooth.print(mq4Digital);
  Bluetooth.print(",");

  Bluetooth.print(leftDistance, 1);
  Bluetooth.print(",");

  Bluetooth.print(rightDistance, 1);

  Bluetooth.print("\n");
}

// ============================================================
// MOTOR FAILSAFE
// ============================================================

void serviceMotorFailsafe()
{
  unsigned long now = millis();

  if (!motorsRunning)
    return;

  if ((unsigned long)(now - lastValidCommandMs) >
      MOTOR_FAILSAFE_MS)
  {
    commandX = 0.0f;
    commandY = 0.0f;

    stopMotors();

    Serial.print("MOTOR_FAILSAFE_STOP t=");
    Serial.println(now);
  }
}

// ============================================================
// USB DEBUG
// ============================================================

void serviceDebug()
{
  unsigned long now = millis();

  if ((unsigned long)(now - lastDebugMs) <
      DEBUG_UPDATE_MS)
  {
    return;
  }

  lastDebugMs = now;

  Serial.print("TEMP=");
  Serial.print(temperature, 2);

  Serial.print(" HUM=");
  Serial.print(humidity, 2);

  Serial.print(" MQ4=");
  Serial.print(mq4Analog);

  Serial.print(" DIG=");
  Serial.print(mq4Digital);

  Serial.print(" L=");
  Serial.print(leftDistance, 1);

  Serial.print(" R=");
  Serial.print(rightDistance, 1);

  Serial.print(" SPEED=");
  Serial.print((int)currentSpeedMode);

  Serial.print(" MOVING=");
  Serial.println(movementActive ? 1 : 0);
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
  // ----------------------------------------------------------
  // USB
  // ----------------------------------------------------------

  Serial.begin(115200);

  // ----------------------------------------------------------
  // HC-05
  // ----------------------------------------------------------

  Bluetooth.begin(BT_BAUD);

  // ----------------------------------------------------------
  // DHT
  // ----------------------------------------------------------

  dht.begin();

  // ----------------------------------------------------------
  // MQ4
  // ----------------------------------------------------------

  pinMode(MQ4_ANALOG, INPUT);
  pinMode(MQ4_DIGITAL, INPUT);

  // ----------------------------------------------------------
  // ULTRASONIC
  // ----------------------------------------------------------

  pinMode(LEFT_TRIG, OUTPUT);
  pinMode(LEFT_ECHO, INPUT);

  pinMode(RIGHT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);

  digitalWrite(LEFT_TRIG, LOW);
  digitalWrite(RIGHT_TRIG, LOW);

  // ----------------------------------------------------------
  // MOTORS
  // ----------------------------------------------------------

  stopMotors();

  currentSpeedMode = SPEED_NORMAL;

  commandX = 0.0f;
  commandY = 0.0f;

  unsigned long now = millis();

  lastValidCommandMs = now;
  lastCommandActivityMs = 0;

  lastSensorUpdateMs = now;
  lastTelemetryMs = 0;
  lastDhtUpdateMs = now;
  lastDebugMs = now;

  nextUltraUs = micros() + 1000UL;
  ultraState = ULTRA_IDLE;
  echoStartUs = 0;

  // ----------------------------------------------------------
  // STARTUP MESSAGE
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("========================================");
  Serial.println(" DRILLPULSE PRIORITY CONTROLLER");
  Serial.println("========================================");
  Serial.println("HC-05 @ 9600");
  Serial.println("RX D10 / TX D11");
  Serial.println();
  Serial.println("MOTOR CONTROL = HIGH PRIORITY");
  Serial.println("SENSOR TX = ONLY WHEN MOTOR CONTROL IDLE");
  Serial.println("0 = SLOW");
  Serial.println("1 = NORMAL");
  Serial.println("2 = TURBO");
  Serial.println();
  Serial.println("No delay()");
  Serial.println("No pulseIn()");
  Serial.println("No readStringUntil()");
  Serial.println();
  Serial.println("Motor failsafe = 450 ms");
  Serial.println("Sensor telemetry = 100 ms when idle");
  Serial.println("========================================");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  /*
    IMPORTANT EXECUTION ORDER

    1. Receive Bluetooth first.
    2. Apply motor command immediately.
    3. Continue ultrasonic state machine.
    4. Update DHT slowly.
    5. Read MQ4.
    6. Send sensors only when motor control is idle.
    7. Run motor emergency watchdog.
  */

  serviceBluetoothRx();

  // Safety must run before any sensor work, including DHT sampling.
  serviceMotorFailsafe();

  if (!fullControlActive)
  {
    serviceUltrasonic();

    // The DHT library uses timing-sensitive reads internally. Do not
    // invoke it while the rover is under active motor control.
    if (!movementActive &&
        !Bluetooth.available() &&
        (unsigned long)(millis() - lastCommandActivityMs) >=
            COMMAND_TRAFFIC_IDLE_MS)
    {
      serviceDHT();
    }

    serviceSensors();

    serviceDebug();
  }
}
