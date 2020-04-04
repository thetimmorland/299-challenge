//
// CONFIG VARIABLES
//

const float wheelDiameter; // measured diameter of wheels
const float robotBaseline; // measured distance between wheels

const int startButtonPin;
const int rightMotorDirectionPin;
const int rightMotorPowerPin;
const int rightMotorEncoderPin;
const int leftMotorDirectionPin;
const int leftMotorPowerPin;
const int leftMotorEncoderPin;

// how close does the robot need to be to it's target angle or position to
// consider the moment finished?
const float angleTolerance = 0.1; // 0.1 radians is about 5 degrees
const float positionTolerance = 0.05; // 5 cm

// how quickly to the motors ramp up
const float kpHeading = 10;
const float kpPosition = 10;

const float initialXPos = 0;
const float initialYPos = 0;
const float initialHeading = M_PI/2;

//
// TYPES
//

enum state_t { SEARCH, WAIT, AT_HOME, GO_HOME };

class Motor {
  int directionPin = 0;
  int powerPin = 0;
  float direction = 0;

  volatile float velocity = 0;


 public:
 Motor(int directionPin, int powerPin) : directionPin(directionPin), powerPin(powerPin) {
    pinMode(directionPin, OUTPUT);
    pinMode(powerPin, OUTPUT);
  }

  // set motor power, sign of input sets direction
  void setPower(int power) {
    if (power > 0) {
      direction = 1;
      digitalWrite(directionPin, HIGH);
    } else {
      direction = -1;
      digitalWrite(directionPin, LOW);
    }

    analogWrite(powerPin, power * direction);
  };

  float getVelocity() {
    return velocity;
  }

  void isr() {
    static int lastMillis = 0;
    int thisMillis = millis();
    velocity = direction * wheelDiameter / 16 / (thisMillis - lastMillis) / 1000;
    lastMillis = thisMillis;
  }
};

class Robot {
  Motor rightMotor = Motor(rightMotorDirectionPin, rightMotorPowerPin);
  Motor leftMotor = Motor(leftMotorDirectionPin, leftMotorPowerPin);

  float xPos = initialXPos;
  float yPos = initialYPos;
  float heading = initialHeading;

  state_t state = SEARCH;

  // return distance travelled since function was last called, and update
  // absolute position variables
  float recalcPosition() {
    static int lastMillis = 0;
    int thisMillis = millis();
    const int delta = rightMotor.getVelocity() * (thisMillis - lastMillis) / 1000;

    xPos += delta * cos(heading);
    yPos += delta * sin(heading);

    lastMillis = thisMillis;
    return delta;
  }

  // return change in heading since last function was last called, and update
  // abosulute heading variable
  float recalcHeading() {
    static int lastMillis = 0;
    int thisMillis = millis();

    // calculation for heading change Page 72 of Elements of Robotics
    // https://doi.org/10.1007/978-3-319-62533-1
    float headingDelta = (rightMotor.getVelocity() - leftMotor.getVelocity())
      * (thisMillis - lastMillis) / 1000 / robotBaseline;
    heading += headingDelta;

    lastMillis = thisMillis;
    return headingDelta;
  }

  int approxEqual(float a, float b, float tolerance) {
    return a < b + tolerance || a < b - tolerance;
  }

 public:
  state_t getState() { return state; }
  float getXPos() { return xPos; }
  float getYPos() { return yPos; }
  float getHeading() { return heading; }

  int startButtonPushed() { return !digitalRead(startButtonPin); }

  void rightMotorISR() { rightMotor.isr(); }
  void leftMotorISR() { leftMotor.isr(); }
  void bumperISR() {}
  void IRSensorISR() {}

  // positive values turn clockwise, negative values turn counter-clockwise
  void turn(float radiansToTurn) {
    float radiansTurned = 0;
    while(!approxEqual(radiansToTurn, radiansTurned, angleTolerance)) {
      const int power = kpHeading * (radiansToTurn - radiansTurned);
      rightMotor.setPower(power);
      leftMotor.setPower(-power);
      radiansTurned += recalcHeading();
    }
  };

  // positive values drive forwards, negative values drive backwards
  void drive(float distanceToDrive) {
    float distanceDriven = 0;
    while(!approxEqual(distanceToDrive, distanceDriven, positionTolerance)) {
      const int power = kpPosition * (distanceToDrive - distanceDriven);
      distanceDriven += recalcPosition();
    }
  };
};

//
// Main Program
//

Robot robot = Robot();

// this wrapper around the ISR is required to match the function signature
// accepted by attachInterupt
void rightMotorISR() { robot.rightMotorISR(); }
void leftMotorISR() { robot.rightMotorISR(); }

void setup() {
  attachInterrupt(rightMotorEncoderPin, rightMotorISR, CHANGE);
  attachInterrupt(leftMotorEncoderPin, leftMotorISR, CHANGE);

  // wait for someone to push the button
  while(!robot.startButtonPushed()) {}
  delay(1000);

  interrupts();
}

void loop() {
  robot.drive(1); // drive 1m forwards
  robot.turn(M_PI); // turn pi radians;
}
