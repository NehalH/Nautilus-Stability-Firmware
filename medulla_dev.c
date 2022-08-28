
#include <Stepper.h>
#include <Wire.h>
#include <Servo.h>


//################################################### Untested
byte servoPin = 2;           // Change pin nos
byte potentiometerPin = A0;
Servo servo;

byte servoPin2 = 3;           // Change pin nos
byte potentiometerPin2 = A1;
Servo servo2;

byte potentiometerPin3 = A2;

//################################################### 

const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 
int minVal=265;
int maxVal=402;
 
int x;
int y;
//int z;

int tiltAngle;
int stepperRPM =60;

int prevX= 0;

#define motor1_pin1 22
#define motor1_pin2 24
#define motor2_pin1 26
#define motor2_pin2 28

int motor_speed;
//int motor2_speed;
int stepsPerRevolution= 100;  // Steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper StepperR(stepsPerRevolution, 8, 9, 10, 11);
Stepper StepperL(stepsPerRevolution, 4, 5, 6, 7);

/////////////////////////////////////////////////////////////////

void setup() {

  // set the speed at 60 rpm
  StepperR.setSpeed(stepperRPM);                                // Assign Stepper motor speed
  StepperL.setSpeed(stepperRPM);

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
 
 
  //################################################### Untested
  servo.attach(servoPin);
  servo.writeMicroseconds(1500);
 
  servo2.attach(servoPin2);
  servo2.writeMicroseconds(1500);
 
  //################################################### 
  // initialize the serial port:
  Serial.begin(9600);

}

/////////////////////////////////////////////////////////////////

void loop() {
  //################################################### Untested
  int potVal = analogRead(potentiometerPin);
  int pwmVal= map (potVal,0, 1023, 1100, 1900);
  servo.writeMicroseconds(pwmVal);
 
  int potVal2 = analogRead(potentiometerPin2);
  int pwmVal2= map (potVal2,0, 1023, 1100, 1900);
  servo2.writeMicroseconds(pwmVal2);
 
  int potVal3 = analogRead(potentiometerPin3);
  int pwmVal3= map (potVal3,0, 1023, 1100, 1900);
  servo.writeMicroseconds(pwmVal3);
  servo2.writeMicroseconds(pwmVal3);
 //###################################################
 
 
  // Read angle
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
  
  x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);                 // Convert rad to deg
  y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  //z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
  
  if(x>90) x-=360;                                          // Convert angles (x>180) to Negative angles (0 to -180)
  if(y>90) y-=360;  
 
  tiltAngle= 50*(x/90);

  Serial.println("\n-----------------------------------------\n");
  Serial.print("AngleX= ");
  Serial.println(x);
  
  //Serial.print("AngleY= ");
  //Serial.println(y);
  
  //Serial.print("AngleZ= ");
  //Serial.println(z);

/////////////////////////////////////////////////////////////////    roll stability using ballasts

  if(!(x<15) || !(x>-15)){                                   // If x is not between +15 to -15 deg

      motor_speed = 125*x; //To move first motor
      //motor2_speed = 125*x; //To move second motor

      if(x<90){                                            // If nautillus tilts 90 deg 
        analogWrite (motor1_pin2, motor_speed);
        analogWrite (motor2_pin2, 0);

          Serial.print ("Motor1 Speed = ");
          Serial.print (-motor_speed, DEC);
          
          Serial.print ("\nMotor2 Speed = 0");
        }

        
      else if(x<-269){                                             // x jumps to -270 deg after 90 deg (Conversion logic bug)
        analogWrite (motor2_pin2, motor_speed);
        analogWrite (motor1_pin2, 0);

          Serial.print ("Motor1 Speed = 0");
          
          Serial.print ("\nMotor2 Speed = ");
          Serial.println (motor_speed, DEC);
        }

  }

  else{
      analogWrite (motor1_pin2, 0);
      analogWrite (motor2_pin2, 0);

          Serial.print ("Motor1 Speed = 0");
          
          Serial.print ("\nMotor2 Speed = 0");
    }

    tiltAngle= x;                                                   // Thruster vector angle

/////////////////////////////////////////////////////////////////    Thrust vectoring for roll stability

    if(x>0){                                                        // Nautilus tilts Right
      if(x>prevX){                                                  // Comparing cruuent roll angle reading with last reading
        StepperR.step(x-prevX);
        Serial.print ("\nTiltR =");
        Serial.print (prevX+(x-prevX));
        Serial.print ("\tIncrement = ");
        Serial.print (x-prevX);
      }
      else{
        StepperR.step(x-prevX);
        Serial.print ("\nTiltR =");
        Serial.print (prevX+(x-prevX));
        Serial.print ("\tIncrement = ");
        Serial.print (x-prevX);
      }
      Serial.print ("\nTiltL = 0");
    }

    else{                                                           // Nautilus tilts Left
      Serial.print ("\nTiltR = 0");
      if(x>prevX){                                                  // Comparing cruuent roll angle reading with last reading
        StepperL.step(x-prevX);
        Serial.print ("\nTiltL =");
        Serial.print (-(prevX+(x-prevX)));
        Serial.print ("\tIncrement = ");
        Serial.print (x-prevX);
      }
      else{
        StepperL.step(x-prevX);
        Serial.print ("\nTiltL =");
        Serial.print (-(prevX+(x-prevX)));
        Serial.print ("\tIncrement = ");
        Serial.print (x-prevX);
      }
    }
    
    prevX= x;                                                     // Save previous x value reading
    
 /////////////////////////////////////////////////////////////////    Thrust vectoring for pitch stability
 
 
    delay(1000);                                                  // Delay (To be removed for production code)

}
