
#include <AFMotor_R4.h>
#include <DHT.h>
#include <SoftwareSerial.h>

// ============================================================
// DRILLPULSE REALTIME ARDUINO CONTROLLER
// ============================================================
//
// HC-05
// TXD -> Arduino D10
// RXD -> Arduino D11
//
// IMPORTANT:
// Arduino receives movement commands immediately.
// No readStringUntil().
// No blocking Bluetooth command parser.
//
// ============================================================


// ============================================================
// BLUETOOTH
// ============================================================

SoftwareSerial Bluetooth(10, 11);   // RX, TX

const unsigned long BT_BAUD = 9600;


// ============================================================
// MOTORS
// ============================================================

AF_DCMotor motor1(1);   // Front Left
AF_DCMotor motor2(2);   // Rear Left
AF_DCMotor motor3(3);   // Front Right
AF_DCMotor motor4(4);   // Rear Right


// ============================================================
// MOTOR SETTINGS
// ============================================================

const int MAX_PWM = 255;

// 0 = SLOW
// 1 = NORMAL
// 2 = TURBO

int speedMode = 1;

const float SPEED_SLOW   = 0.40;
const float SPEED_NORMAL = 0.70;
const float SPEED_TURBO  = 1.00;


// ============================================================
// MOTOR DIRECTION
// ============================================================

bool INVERT_M1 = false;
bool INVERT_M2 = false;
bool INVERT_M3 = false;
bool INVERT_M4 = false;


// ============================================================
// REALTIME COMMAND SAFETY
// ============================================================

// If laptop stops sending commands, motors stop.
// 250 ms gives responsive safety without making normal control jittery.

const unsigned long COMMAND_TIMEOUT = 250;

unsigned long lastCommandTime = 0;


// ============================================================
// SENSOR PINS
// ============================================================

// MQ4
#define MQ4_PIN A0
#define MQ4_DO_PIN 2

// DHT22
#define DHT_PIN A1
#define DHT_TYPE DHT22

DHT dht(DHT_PIN, DHT_TYPE);


// LEFT ULTRASONIC
#define LEFT_TRIG_PIN A2
#define LEFT_ECHO_PIN A3

// RIGHT ULTRASONIC
#define RIGHT_TRIG_PIN A4
#define RIGHT_ECHO_PIN A5


// ============================================================
// SENSOR TIMING
// ============================================================

// Sensor packet every 500 ms
const unsigned long SENSOR_INTERVAL = 500;

// DHT22 only needs about 2 seconds
const unsigned long DHT_INTERVAL = 2000;

unsigned long lastSensorTime = 0;
unsigned long lastDHTTime = 0;


// ============================================================
// ULTRASONIC
// ============================================================

// 12 ms ~= about 2 meters.
// Much shorter than the old 30 ms timeout.

const unsigned long ULTRASONIC_TIMEOUT = 12000;


// ============================================================
// SENSOR VALUES
// ============================================================

int mq4AnalogValue = 0;
int mq4DigitalValue = 0;

float temperature = 0.0;
float humidity = 0.0;

float leftDistance = -1.0;
float rightDistance = -1.0;


// ============================================================
// BLUETOOTH INPUT BUFFER
// ============================================================

// IMPORTANT:
// We do NOT use String.
// We do NOT use readStringUntil().
//
// This parser receives one character at a time and immediately
// processes a complete command.

const int BT_BUFFER_SIZE = 40;

char btBuffer[BT_BUFFER_SIZE];
byte btIndex = 0;


// ============================================================
// SPEED
// ============================================================

float getSpeedMultiplier()
{
  if (speedMode == 0)
    return SPEED_SLOW;

  if (speedMode == 1)
    return SPEED_NORMAL;

  return SPEED_TURBO;
}


// ============================================================
// STOP MOTORS
// ============================================================

void stopMotors()
{
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}


// ============================================================
// SET MOTOR
// ============================================================

void setMotor(
  AF_DCMotor &motor,
  float command,
  bool inverted
)
{
  command = constrain(command, -1.0, 1.0);

  if (inverted)
    command = -command;

  command *= getSpeedMultiplier();

  int pwm = (int)(abs(command) * MAX_PWM);

  pwm = constrain(pwm, 0, 255);

  if (pwm == 0)
  {
    motor.run(RELEASE);
    return;
  }

  motor.setSpeed(pwm);

  if (command > 0)
    motor.run(FORWARD);
  else
    motor.run(BACKWARD);
}


// ============================================================
// DRIVE
// ============================================================

void drive(float x, float y)
{
  // ----------------------------------------------------------
  // FORWARD
  // ----------------------------------------------------------

  if (y > 0.5)
  {
    setMotor(motor1,  1.0, INVERT_M1);
    setMotor(motor2,  1.0, INVERT_M2);
    setMotor(motor3,  1.0, INVERT_M3);
    setMotor(motor4,  1.0, INVERT_M4);
  }

  // ----------------------------------------------------------
  // BACKWARD
  // ----------------------------------------------------------

  else if (y < -0.5)
  {
    setMotor(motor1, -1.0, INVERT_M1);
    setMotor(motor2, -1.0, INVERT_M2);
    setMotor(motor3, -1.0, INVERT_M3);
    setMotor(motor4, -1.0, INVERT_M4);
  }

  // ----------------------------------------------------------
  // RIGHT
  // ----------------------------------------------------------

  else if (x > 0.5)
  {
    setMotor(motor1,  1.0, INVERT_M1);
    setMotor(motor2,  1.0, INVERT_M2);

    setMotor(motor3, -1.0, INVERT_M3);
    setMotor(motor4, -1.0, INVERT_M4);
  }

  // ----------------------------------------------------------
  // LEFT
  // ----------------------------------------------------------

  else if (x < -0.5)
  {
    setMotor(motor1, -1.0, INVERT_M1);
    setMotor(motor2, -1.0, INVERT_M2);

    setMotor(motor3,  1.0, INVERT_M3);
    setMotor(motor4,  1.0, INVERT_M4);
  }

  // ----------------------------------------------------------
  // STOP
  // ----------------------------------------------------------

  else
  {
    stopMotors();
  }
}


// ============================================================
// SEND SPEED ACK
// ============================================================

void sendSpeedStatus()
{
  Bluetooth.print("SPEED:");

  if (speedMode == 0)
    Bluetooth.println("SLOW");

  else if (speedMode == 1)
    Bluetooth.println("NORMAL");

  else
    Bluetooth.println("TURBO");
}


// ============================================================
// PROCESS SPEED
// ============================================================

void processSpeedCommand(char *command)
{
  // Expected:
  //
  // SPEED,0
  // SPEED,1
  // SPEED,2

  int mode = atoi(command + 6);

  if (mode >= 0 && mode <= 2)
  {
    speedMode = mode;

    sendSpeedStatus();

    Bluetooth.print("ACK,SPEED,");
    Bluetooth.println(speedMode);
  }
}


// ============================================================
// PROCESS MOVEMENT
// ============================================================

void processCommand(char *command)
{
  if (command == NULL)
    return;

  if (command[0] == '\0')
    return;


  // ----------------------------------------------------------
  // SPEED
  // ----------------------------------------------------------

  if (strncmp(command, "SPEED,", 6) == 0)
  {
    processSpeedCommand(command);
    return;
  }


  // ----------------------------------------------------------
  // MOVEMENT
  // ----------------------------------------------------------

  if (strncmp(command, "CMD,", 4) == 0)
  {
    char *comma = strchr(command + 4, ',');

    if (comma == NULL)
      return;

    *comma = '\0';

    float x = atof(command + 4);
    float y = atof(comma + 1);

    x = constrain(x, -1.0, 1.0);
    y = constrain(y, -1.0, 1.0);

    // CRITICAL:
    // Drive immediately.

    drive(x, y);

    // Refresh safety timer immediately.

    lastCommandTime = millis();

    return;
  }


  // ----------------------------------------------------------
  // RESET
  // ----------------------------------------------------------

  if (strcmp(command, "RESET") == 0)
  {
    stopMotors();

    speedMode = 1;

    lastCommandTime = millis();

    Bluetooth.println("ARDUINO RESET: OK");

    return;
  }


  // ----------------------------------------------------------
  // UNKNOWN COMMAND
  // ----------------------------------------------------------

  // Do NOT spam Bluetooth with error packets.
  //
  // This is intentionally ignored.
  //
  // Spamming ERROR packets can make a realtime control link
  // even worse.

}


// ============================================================
// REALTIME BLUETOOTH RECEIVER
// ============================================================

void processBluetooth()
{
  // Process EVERY available byte immediately.

  while (Bluetooth.available() > 0)
  {
    char c = Bluetooth.read();

    // --------------------------------------------------------
    // Newline = complete command
    // --------------------------------------------------------

    if (c == '\n' || c == '\r')
    {
      if (btIndex > 0)
      {
        btBuffer[btIndex] = '\0';

        processCommand(btBuffer);

        btIndex = 0;
      }

      continue;
    }


    // --------------------------------------------------------
    // Store character
    // --------------------------------------------------------

    if (btIndex < BT_BUFFER_SIZE - 1)
    {
      btBuffer[btIndex++] = c;
    }
    else
    {
      // Buffer overflow.
      // Drop corrupted command.

      btIndex = 0;
    }
  }
}


// ============================================================
// ULTRASONIC
// ============================================================

float readUltrasonic(
  int trigPin,
  int echoPin
)
{
  digitalWrite(trigPin, LOW);

  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);

  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);


  unsigned long duration =
    pulseIn(
      echoPin,
      HIGH,
      ULTRASONIC_TIMEOUT
    );


  if (duration == 0)
    return -1.0;


  float distance =
    duration * 0.0343 / 2.0;


  if (distance < 2.0 || distance > 500.0)
    return -1.0;


  return distance;
}


// ============================================================
// SEND SENSOR DATA
// ============================================================

void sendSensorData()
{
  Bluetooth.print("TEMP=");
  Bluetooth.print(temperature, 2);

  Bluetooth.print(",HUMIDITY=");
  Bluetooth.print(humidity, 2);

  Bluetooth.print(",MQ4_ANALOG=");
  Bluetooth.print(mq4AnalogValue);

  Bluetooth.print(",MQ4_DIGITAL=");
  Bluetooth.print(mq4DigitalValue);

  Bluetooth.print(",LEFT_DISTANCE=");
  Bluetooth.print(leftDistance, 2);

  Bluetooth.print(",RIGHT_DISTANCE=");
  Bluetooth.println(rightDistance, 2);
}


// ============================================================
// USB DEBUG SENSOR DATA
// ============================================================

void sendSensorDataUSB()
{
  Serial.print("TEMP=");
  Serial.print(temperature, 2);

  Serial.print(",HUMIDITY=");
  Serial.print(humidity, 2);

  Serial.print(",MQ4_ANALOG=");
  Serial.print(mq4AnalogValue);

  Serial.print(",MQ4_DIGITAL=");
  Serial.print(mq4DigitalValue);

  Serial.print(",LEFT_DISTANCE=");
  Serial.print(leftDistance, 2);

  Serial.print(",RIGHT_DISTANCE=");
  Serial.println(rightDistance, 2);
}


// ============================================================
// READ SENSORS
// ============================================================

void readSensors()
{
  // ----------------------------------------------------------
  // MQ4
  // ----------------------------------------------------------

  mq4AnalogValue = analogRead(MQ4_PIN);

  mq4DigitalValue = digitalRead(MQ4_DO_PIN);


  // ----------------------------------------------------------
  // DHT22
  // ----------------------------------------------------------

  if (millis() - lastDHTTime >= DHT_INTERVAL)
  {
    lastDHTTime = millis();

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (!isnan(h) && !isnan(t))
    {
      humidity = h;
      temperature = t;
    }
  }


  // ----------------------------------------------------------
  // LEFT ULTRASONIC
  // ----------------------------------------------------------

  leftDistance =
    readUltrasonic(
      LEFT_TRIG_PIN,
      LEFT_ECHO_PIN
    );


  // ----------------------------------------------------------
  // IMPORTANT:
  // Check Bluetooth again immediately after LEFT sensor.
  // ----------------------------------------------------------

  processBluetooth();


  // ----------------------------------------------------------
  // RIGHT ULTRASONIC
  // ----------------------------------------------------------

  rightDistance =
    readUltrasonic(
      RIGHT_TRIG_PIN,
      RIGHT_ECHO_PIN
    );


  // ----------------------------------------------------------
  // SEND SENSOR PACKET
  // ----------------------------------------------------------

  sendSensorData();

  sendSensorDataUSB();
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
  // MQ4
  // ----------------------------------------------------------

  pinMode(
    MQ4_DO_PIN,
    INPUT
  );


  // ----------------------------------------------------------
  // ULTRASONIC
  // ----------------------------------------------------------

  pinMode(
    LEFT_TRIG_PIN,
    OUTPUT
  );

  pinMode(
    LEFT_ECHO_PIN,
    INPUT
  );

  pinMode(
    RIGHT_TRIG_PIN,
    OUTPUT
  );

  pinMode(
    RIGHT_ECHO_PIN,
    INPUT
  );


  digitalWrite(
    LEFT_TRIG_PIN,
    LOW
  );

  digitalWrite(
    RIGHT_TRIG_PIN,
    LOW
  );


  // ----------------------------------------------------------
  // DHT
  // ----------------------------------------------------------

  dht.begin();


  // ----------------------------------------------------------
  // MOTORS
  // ----------------------------------------------------------

  stopMotors();


  // ----------------------------------------------------------
  // TIMERS
  // ----------------------------------------------------------

  lastCommandTime = millis();

  lastSensorTime = millis();

  lastDHTTime = millis();


  // ----------------------------------------------------------
  // STARTUP
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("========================================");
  Serial.println("DRILLPULSE REALTIME ARDUINO");
  Serial.println("========================================");
  Serial.println("HC-05: D10/D11");
  Serial.println("BAUD: 9600");
  Serial.println("COMMAND PARSER: NON-BLOCKING");
  Serial.println("========================================");


  Bluetooth.println("DRILLPULSE ARDUINO READY");

  Bluetooth.println("HC-05 CONNECTED");

  Bluetooth.println("BAUD=9600");

  sendSpeedStatus();
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  // ==========================================================
  // PRIORITY #1
  // BLUETOOTH
  // ==========================================================

  // This is ALWAYS first.

  processBluetooth();


  // ==========================================================
  // PRIORITY #2
  // SAFETY STOP
  // ==========================================================

  if (
    millis() - lastCommandTime >
    COMMAND_TIMEOUT
  )
  {
    stopMotors();
  }


  // ==========================================================
  // PRIORITY #3
  // SENSORS
  // ==========================================================

  if (
    millis() - lastSensorTime >=
    SENSOR_INTERVAL
  )
  {
    lastSensorTime = millis();

    readSensors();
  }
}


