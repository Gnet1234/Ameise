#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Include FNQR (Freenove Quadruped Robot) library
#include <FNQR.h>

FNQR robot;

// Initialize PWM driver with default address
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Constants
#define ANGLE_MIN 0    // Minimum angle
#define ANGLE_MAX 180  // Maximum angle
#define PWM_MIN 150    // PWM value for 0 degrees
#define PWM_MAX 600    // PWM value for 180 degrees
#define SERVO_FREQ 50  // Servo frequency in Hz


// Fixed channel for testing
const uint8_t servoChannel = 2;

// defines pins numbers
const int trigPin = A0;
const int echoPin = A1;
// defines variables
long duration;
int distance;
int result;
int result1;
int result2;
int counter = 0;


void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication
  pwm.begin();
  pwm.setOscillatorFrequency(27000000); // Set the internal oscillator frequency
  pwm.setPWMFreq(SERVO_FREQ);  // Set PWM frequency
  robot.Start();
  delay(10);
}
void loop() {
  result = Sense();
  if (result < 20) {
    // Move servo from 0 to 180 degrees
    Serial.println("It has turned to 0 degrees");
    moveServo(ANGLE_MIN, ANGLE_MAX);
    result1 = Sense();

    Serial.println("Waiting for timer.");
    delay(450);  // Hold at 180 degrees for a bit

    // Move servo from 180 to 0 degrees
    Serial.println("It has turned to 180 degrees");
    moveServo(ANGLE_MAX, ANGLE_MIN);
    result2 = Sense();

    if (result1 < result2){
      // Move in the direction of servo at 0 degrees.
      robot.TurnLeft();
      robot.TurnLeft();
      robot.TurnLeft();
      robot.TurnLeft();
      delay(1000);
    }
    else if (result2 < result1){
      // Move in the direction of the servo at 180 degrees
      robot.TurnRight();
      robot.TurnRight();
      robot.TurnRight();
      robot.TurnRight();
      delay(1000);
    }
    else {
      // Move backwards
      robot.CrawlBackward();
      robot.CrawlBackward();
      robot.CrawlBackward();
      robot.CrawlBackward();
      delay(1000);
    }
  }
  else {
    // Move forward
    robot.CrawlForward();
    delay(1000);
  }

}

void moveServo(uint8_t startAngle, uint8_t endAngle) {
  int step = (startAngle < endAngle) ? 1 : -1;
  for (uint8_t angle = startAngle; angle != endAngle + step; angle += step) {
    uint16_t pwmValue = angleToPWM(angle);
    pwm.setPWM(servoChannel, 0, pwmValue);
    delay(20);  // Adjust delay for smoothness of movement
    counter = counter + 1;
    Serial.println(counter);
    if (counter > 500){
      Serial.println("Ending the process");
      counter = 0;
      break;
    }
  }
}

int Sense(){
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  return distance;
}

// Function to map angle to PWM pulse width
uint16_t angleToPWM(uint8_t angle) {
  return map(angle, ANGLE_MIN, ANGLE_MAX, PWM_MIN, PWM_MAX);
}

