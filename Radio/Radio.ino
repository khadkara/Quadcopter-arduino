
//Interrrupt
#include <PinChangeInt.h>
#include <Servo.h>

// FREEIMU
#include <ADXL345.h>
#include <bma180.h>
#include <HMC58X3.h>
#include <ITG3200.h>
#include <MS561101BA.h>
#include <I2Cdev.h>
#include <MPU60X0.h>
#include <EEPROM.h>

//#define DEBUG
#include "DebugUtils.h"
#include "CommunicationUtils.h"
#include "FreeIMU.h"
#include <Wire.h>
#include <SPI.h>
//PID
#include <PID_v1.h>










#define CHNL_HIGH 1900
#define CHNL_LOW 1100

#define THROTTLE_CHNL 2


#define CW1_IN 9
#define CW2_IN 10
#define CW3_IN 11
#define CW4_IN 12

#define ARM_DEG 21

#define THROTTLE_FLAG 1



Servo servoCW1;
Servo servoCW2;

Servo servoCW3;
Servo servoCW4;

int servoCW1_DEG = 0;
int servoCW2_DEG = 0;



volatile uint8_t bUpdateFlagsShared;
volatile uint16_t unThrottleInShared;
volatile int throttleOffset; 
uint32_t ulThrottleStart;



float ypr[3]; // yaw pitch roll
//FreeIMU my3IMU = FreeIMU(); // Set the FreeIMU object


  // my3IMU.getYawPitchRoll(ypr);
  // Serial.print("Yaw: ");
  // Serial.print(ypr[0]);
  // Serial.print(" Pitch: ");
  // Serial.print(ypr[1]);
  // Serial.print(" Roll: ");
  // Serial.print(ypr[2]);
  // Serial.println("");






void setup()
{

  //my3IMU.init(); // the parameter enable or disable fast mode
  
  servoCW1.attach(CW1_IN);
  servoCW2.attach(CW2_IN); 

  servoCW3.attach(CW3_IN);
  servoCW4.attach(CW4_IN);

  Serial.begin(9600);
  Serial.println("GET IT");
  arm();
  attachInterrupt(0, calcThrottle,CHANGE);
  delay(5);
}

void loop()
{

  static uint16_t unThrottleIn;
  static uint8_t bUpdateFlags;

  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); 

    bUpdateFlags = bUpdateFlagsShared;
   
    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }
  
    bUpdateFlagsShared = 0;
   
    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    
  }

 if(bUpdateFlags & THROTTLE_FLAG)
 
 {
   Serial.println(unThrottleIn);
      throttleChange(unThrottleIn);
     

 }

  bUpdateFlags = 0;
}



void arm()
{

  servoCW1.write(ARM_DEG);
  servoCW2.write(ARM_DEG);

  servoCW3.write(ARM_DEG);
  servoCW4.write(ARM_DEG);

}

void throttleChange(int cha_val)
{


   servoCW1.writeMicroseconds(cha_val);
   servoCW2.writeMicroseconds(cha_val);
    
 

   servoCW3.writeMicroseconds(cha_val);
   servoCW4.writeMicroseconds(cha_val);

}

void calcThrottle()
{
  if(digitalRead(THROTTLE_CHNL) == HIGH)
  {
    ulThrottleStart = micros();
  }else
  {

    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

