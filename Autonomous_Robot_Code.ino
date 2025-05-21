#include <Servo.h>

// Include FNQR (Freenove Quadruped Robot) library
#include <FNQR.h>

FNQR robot;

// Constants
#define ANGLE_MIN 0    // Minimum angle
#define ANGLE_MAX 180  // Maximum angle

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

Servo myServo;

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  Serial.begin(9600); // Starts the serial communication

  // Attach servo motors
  myServo.attach(4);
  
  // Starts the Robot
  robot.Start();
  delay(10);

}

void loop() {
  result = Sense();
  if (result < 20) {
    // Move servo from 0 to 180 degrees
    Serial.println("It has turned to 0 degrees");
    myServo.write(0);
    result1 = Sense();

    Serial.println("Waiting for timer.");
    delay(450);  // Hold at 180 degrees for a bit

    // Move servo from 180 to 0 degrees
    Serial.println("It has turned to 180 degrees");
    myServo.write(180);
    result2 = Sense();

    if (result1 < result2){
      // Move in the direction of servo at 0 degrees.
      TurnLeft();
    }
    else if (result2 < result1){
      // Move in the direction of the servo at 180 degrees
      TurnRight();
    }
    else {
      // Move backwards
      MovingBackwards();
    }
  }
  else {
    // Move forward
    robot.CrawlForward();
    delay(1000);
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

void LookLeft() {
  // Move in the direction of servo at 0 degrees.
  robot.TurnLeft();
  robot.TurnLeft();
  robot.TurnLeft();
  robot.TurnLeft();
  delay(1000);
}

void LookRight(){
  // Move in the direction of the servo at 180 degrees
  robot.TurnRight();
  robot.TurnRight();
  robot.TurnRight();
  robot.TurnRight();
  delay(1000);
}

void MovingBackwards(){
  // Move backwards
  robot.CrawlBackward();
  robot.CrawlBackward();
  robot.CrawlBackward();
  robot.CrawlBackward();
  delay(1000);
}
