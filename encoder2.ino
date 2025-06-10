// Encoder pins (must be interrupt-capable pins on Arduino)
#define LEFT_ENCODER_A 2
#define RIGHT_ENCODER_A 3

// Motor control pins
#define LEFT_MOTOR_FORWARD 6
#define LEFT_MOTOR_BACKWARD 7
#define RIGHT_MOTOR_FORWARD 8
#define RIGHT_MOTOR_BACKWARD 9

// Variables for encoder counts
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Robot parameters
const float wheelDiameter = 7;            // cm (example value)
const float wheelCircumference = wheelDiameter * 3.14159;
//const int encoderTicksPerRevolution = 778;  // Example value
const int encoderTicksPerRevolution = 980;  // Example value
const float distancePerTick = wheelCircumference / encoderTicksPerRevolution;

// Movement parameters
const int cellSize = 30;                    // cm (maze cell size)
const long ticksPerCell = cellSize / distancePerTick;
const long ticksForTurn = 435;              // Ticks for 90-degree turn

void setup() {
  // Initialize motor pins
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), updateRightEncoder, CHANGE);

  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Waiting for path...");
}

void loop() {
  // Wait for path commands from Python via Serial
  /*
  delay(2000);
  moveForward(cellSize);
  moveForward(cellSize);
  turnLeft();
  delay(1000);
  moveForward(cellSize);
  turnLeft();
  delay(1000);
  moveForward(cellSize);
  */
 
  if (Serial.available() > 0) {
    char command = Serial.read();
    switch (command) {
      case 'F': moveForward(cellSize); break; // Move forward one cell
      case 'L': turnLeft(); break;           // Turn left 90 degrees
      case 'R': turnRight(); break;          // Turn right 90 degrees
      case 'S': stopMotors(); break;         // Stop (end of path)
    }
    
  }
 
 //  Serial.println(leftEncoderCount);
}

// Interrupt service routines for encoders
void updateLeftEncoder() {
  leftEncoderCount++;
}

void updateRightEncoder() {
  rightEncoderCount++;
}

// Function to move forward a specific distance (cm)
void moveForward(float distance) {
  long targetTicks = distance / distancePerTick;

  // Reset encoder counts
  leftEncoderCount = 0;
  rightEncoderCount = 0;

  // Move forward until the target distance is reached
  while (abs(leftEncoderCount) < targetTicks && abs(rightEncoderCount) < targetTicks) {
    digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
    digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
    digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
    digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  }

  stopMotors();
}

// Function to turn left 90 degrees
void turnLeft() {
  // Reset encoder counts
  leftEncoderCount = 0;
  rightEncoderCount = 0;

  // Rotate robot in place
  while (abs(leftEncoderCount) < ticksForTurn && abs(rightEncoderCount) < ticksForTurn) {
    digitalWrite(LEFT_MOTOR_FORWARD, LOW);
    digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
    digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
    digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  }
  //moveForward(cellSize);
  stopMotors();
}

// Function to turn right 90 degrees
void turnRight() {
  // Reset encoder counts
  leftEncoderCount = 0;
  rightEncoderCount = 0;

  // Rotate robot in place
  while (abs(leftEncoderCount) < ticksForTurn && abs(rightEncoderCount) < ticksForTurn) {
    digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
    digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
    digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
    digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
  }
  //moveForward(cellSize);
  stopMotors();
}

// Function to stop the motors
void stopMotors() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}
