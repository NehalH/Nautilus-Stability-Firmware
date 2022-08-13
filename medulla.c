// Stepper motor on Wokwi!

#include <Stepper.h>
#include <Wire.h>

const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 
int minVal=265;
int maxVal=402;
 
int x;
//int y;
//int z;

int tiltAngle;

#define motor1_pin1 2
#define motor1_pin2 3
#define motor2_pin1 4
#define motor2_pin2 5

int motor1_speed;
int motor2_speed;
int stepsPerRevolution= 100;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

void setup() {

  // set the speed at 60 rpm:
  myStepper.setSpeed(60);

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // initialize the serial port:
  Serial.begin(9600);

}

void loop() {

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);

  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();

  int xAng = map(AcX,minVal,maxVal,-90,90);
  int yAng = map(AcY,minVal,maxVal,-90,90);
  int zAng = map(AcZ,minVal,maxVal,-90,90);
  
  x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  //y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  //z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
  
  if(x>90) x-=360;

  tiltAngle= 50*(x/90);

  Serial.print("AngleX= ");
  Serial.println(x);
  
  //Serial.print("AngleY= ");
  //Serial.println(y);
  
  //Serial.print("AngleZ= ");
  //Serial.println(z);

/////////////////////////////////////////////////////////////////

  if(!(x<15) && !(x>345)){

      motor1_speed = 125+x; //To move first motor
      motor2_speed = 125+x; //To move second motor

      if(x>180 && x<270){
        analogWrite (motor1_pin2, motor1_speed);
        analogWrite (motor2_pin2, 0);
        }

        
      if(x<180 && x>90){
        analogWrite (motor1_pin2, 0);
        analogWrite (motor2_pin2, motor1_speed);
        }

      //analogWrite (motor1_pin2, motor1_speed);
      //analogWrite (motor2_pin2, motor2_speed);

  }

  else{
      analogWrite (motor1_pin2, 0);
      analogWrite (motor2_pin2, 0);
    }
    
  Serial.print ("Motor1 Speed = ");
  Serial.print (motor1_speed, DEC);
  
  Serial.print ("Motor2 Speed = ");
  Serial.println (motor2_speed, DEC);
  Serial.println("-----------------------------------------");

/////////////////////////////////////////////////////////////////

    // step one revolution  in one direction:
    //Serial.println("clockwise");
    myStepper.step(tiltAngle);
   
    // step one revolution in the other direction:
    //Serial.println("counterclockwise");
    //myStepper.step(-stepsPerRevolution);
    delay(1000);

}