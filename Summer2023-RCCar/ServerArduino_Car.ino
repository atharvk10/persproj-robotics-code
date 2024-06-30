#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>


RF24 radio(7, 8);  // CE, CSN


//Pins for lights.
int greenLedPin = 39;
int redLedPin = 40;


//Pins for servos
int servoPin = 5;
int clawPin = 10;


//Servo objects for robot
Servo steeringServo;
Servo clawServo;


//Motors pins attached to L298N
int pwmPin = 3;
int in1 = 29;
int in2 = 31;

int MO = 53;

//Specifies the middleXValue for left joystick.
int forwardXValue = 450;
int backwardXValue = 576;

int middleYValue = 512;
int xValue, yValue, butValue, countValue;
int motorSpeed;


const byte address[6] = "00001";




//Creates a data package that will be used to read data from transmitter.
struct Data_Package {
  int leftXValue = 0;
  int rightYValue = 0;
  int buttonValue = 0;
};


Data_Package joyData;


void setup() {
  //Starting the Serial:
  Serial.begin(9600);


  //Sets the function of the driving LEDS
  pinMode(greenLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);

  pinMode(MO, OUTPUT);


  //Attaching the servo pin to the servo that will control
  //steering of the robot.
  pinMode(servoPin, OUTPUT);
  steeringServo.attach(servoPin);
 // steeringServo.write(135);




  //This block of code sets up the motor for the robot.
  pinMode(pwmPin, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  digitalWrite(in1, LOW);  //Next two lines initially turns off of the motor
  digitalWrite(in2, LOW);


  //Attaches the claw pin to the claw of the robot that will pick up objects.
  pinMode(clawPin, OUTPUT);
  clawServo.attach(clawPin);


  //Sets up this arduino as the reciever, waiting to hear data from the transmitter.
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}


void loop() {
  if (radio.available()) {
    radio.read(&joyData, sizeof(Data_Package));
    xValue = joyData.leftXValue;
    yValue = joyData.rightYValue;
    butValue = joyData.buttonValue;

    Serial.println("Recieved data!");
  
  }

  steeringServo.write(yValue);

  if (xValue < forwardXValue) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    motorSpeed = map(xValue, forwardXValue, 0, 100, 255);


  } else if (xValue > backwardXValue) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    motorSpeed = map(xValue, backwardXValue, 1023, 100, 255);

} else {

    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    motorSpeed = 0;

  }


  analogWrite(pwmPin, motorSpeed);

 // grabObject(butValue);
 
}

void grabObject(int butValue) {

  int countValue = 1;

      if(butValue == 1) {
      
            clawServo.write(0);

            delay(1000);

            clawServo.write(135);

            delay(1000);

      }
         
      
        clawServo.write(100);
  }





