#include <FastLED.h>
#include <Servo.h>
#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary
#include <math.h>
#include <string.h>

#define NUM_LEDS 12    // APA102 LED setup
#define DATA_PIN 38
#define CLOCK_PIN 36

#define DISTANCE_BUCKETS 10

CRGB leds[NUM_LEDS];

double Pk1 = 0.5;  
double Ik1 = 0;
double Dk1 = 0.01;

double Setpoint1, Input1, Output1, Output1a;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup

double Pk2 = 0.5;  
double Ik2 = 0;
double Dk2 = 0.01;

double Setpoint2, Input2, Output2, Output2a;    // PID variables
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID Setup

Servo servo1;
Servo servo2;

float distance;
float angle;
bool  startBit;
byte  quality;

// variables for each segment - yes I am using arrays!

int distanceBuckets[DISTANCE_BUCKETS] = {};
int distanceFiltered[DISTANCE_BUCKETS] = {};
int distanceBookmarks[DISTANCE_BUCKETS] = {};

// constraints and other variables

int minDistance = 150;
int maxDistance = 3000;
int threshold = 950;

int button;
int flag = 0;
int segmentDetected = 0;
int bulletCount = 7;

// timer variables

unsigned long currentMillis;
unsigned long previousArmMillis;
unsigned long previousMillis;

// wheel encoder interrupts

#define encoder0PinA 2      // encoder 1
#define encoder0PinB 3

#define encoder1PinA 18     // encoder 2
#define encoder1PinB 19

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2

float demandRot;
float demandRotFiltered;

// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
#include <RPLidar.h>

// You need to create an driver instance 
RPLidar lidar;

#define RPLIDAR_MOTOR 12 // The PWM pin for control the speed of RPLIDAR's motor.
                       
                        
void setup() {

  FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, BGR>(leds, NUM_LEDS);    // LED setup

  setLEDs(0,0,100);  

  servo1.attach(53);    // servo pins
  servo2.attach(51);

  servo1.write(180);
  servo2.write(180);

  pinMode(4, OUTPUT);     // motor PWM pins
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  pinMode(encoder0PinA, INPUT_PULLUP);    // encoder pins
  pinMode(encoder0PinB, INPUT_PULLUP);

  pinMode(encoder1PinA, INPUT_PULLUP); 
  pinMode(encoder1PinB, INPUT_PULLUP);

  attachInterrupt(0, doEncoderA, CHANGE);
  attachInterrupt(1, doEncoderB, CHANGE); 

  attachInterrupt(4, doEncoderC, CHANGE);
  attachInterrupt(5, doEncoderD, CHANGE); 

  pinMode(41, OUTPUT);        // relay for blasters
  pinMode(49, INPUT_PULLUP);  // switch

  digitalWrite(41, HIGH);     // motors off to start with
  
  // bind the RPLIDAR driver to the arduino hardware serial
  lidar.begin(Serial2);
  Serial.begin(115200);
  
  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);

  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-200, 200);  
  PID1.SetSampleTime(10);

  PID2.SetMode(AUTOMATIC);              
  PID2.SetOutputLimits(-200, 200);  
  PID2.SetSampleTime(10);
}

void loop() {

    currentMillis = millis();   // bookmark the time   
     
    if (IS_OK(lidar.waitPoint())) {
      distance = lidar.getCurrentPoint().distance; //distance value in mm unit
      angle    = lidar.getCurrentPoint().angle; //anglue value in degree
      startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
      quality  = lidar.getCurrentPoint().quality; //quality of the current measurement
      
      //perform data processing here... 
  
      if (quality >= 15) { 
        int bucket = getBucket(angle, DISTANCE_BUCKETS);
        distanceBuckets[bucket] = constrain (distance, minDistance,maxDistance);
        distanceFiltered[bucket] = (0.8 * distanceFiltered[bucket]) + (0.2 * distanceBuckets[bucket]);
      }
    
    }   // end of lidar is good & processesing

    
    //  *** Rest of Lidar config ****
     
    else {
      analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
      
      // try to detect RPLIDAR... 
      rplidar_response_device_info_t info;
      if (IS_OK(lidar.getDeviceInfo(info, 100))) {
         // detected...
         lidar.startScan();
         
         // start motor rotating at max allowed speed
         analogWrite(RPLIDAR_MOTOR, 255);
         delay(1000);
      }
    }

     if (currentMillis - previousMillis >= 10) {  // start timed event for motor driving only - PID works better if we slow the loop down
        previousMillis = currentMillis;
        
         if (Serial.available()>0) {       // manual control of wheesls via terminal
            char c = Serial.read();
  
            if (c == 'a') {            
              demandRot = 6300;
            }
            else if (c == 'b') {      
              demandRot = -6300;
            }
            else if (c == 'c') {       
              demandRot = 0;
            }

        }

        //Serial.println(distance1);

        demandRotFiltered = filter(demandRot, demandRotFiltered, 0);    // filter demand position so it moves nicely

        Setpoint1 = demandRotFiltered;
        Input1 = encoder0Pos;
        PID1.Compute();

        Setpoint2 = demandRotFiltered*-1;
        Input2 = encoder1Pos;
        PID2.Compute();
       
        // drive motor

        if (Output1 > 0) {
          Output1a = abs(Output1);
          analogWrite(6, Output1a);
          analogWrite(7, 0);
        }
        else if (Output1 < 0) {
          Output1a = abs(Output1);
          analogWrite(7, Output1a);
          analogWrite(6, 0);
        }
        else {
          analogWrite(7,0);
          analogWrite(6, 0);
        }

        // other motor

        if (Output2 > 0) {
          Output2a = abs(Output2);
          analogWrite(5, Output2a);
          analogWrite(4, 0);
        }
        else if (Output2 < 0) {
          Output2a = abs(Output2);
          analogWrite(4, Output2a);
          analogWrite(5, 0);
        }
        else {
          analogWrite(4,0);
          analogWrite(5, 0);
        }
    
    


        // *** button & fire processing ***
    
        button = digitalRead(49);     // look for arm switch
        if (button == 0) {
            setLEDs(50,50,0);         // set the LEDs to amber
            previousArmMillis = currentMillis;
            flag = 1;
        } 
    
        if (bulletCount <= 1 && flag == 1) {
            Serial.println("Bullets are out");
            setLEDs(0,100,0);     // set the LEDs to green
            bulletCount = 7;    // reset counts for thenext go
            flag = 0;
        }
      
        else if (currentMillis - previousArmMillis > 4000 && flag == 1) {
    
          bulletCount = bulletCount - 1;
          Serial.print("Bullets: ");
          Serial.println(bulletCount);
          
          // ** yes I understand what for loops are also, thanks! **
          memcpy(distanceBookmarks, distanceFiltered, sizeof(int) * DISTANCE_BUCKETS);

          for(int i = 0; i<DISTANCE_BUCKETS; i++){
            Serial.print("Ready, Bookmarked ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(distanceBookmarks[i]);
          }
    
          setLEDs(100,0,0);     // set the LEDs to red
          
          previousArmMillis = currentMillis;
          flag = 2;
        }
    
        else if (flag == 2) {
          for(int i = 0; i<DISTANCE_BUCKETS; i++){
            if(distanceFiltered[i] <= distanceBookmarks[i] - threshold){
              segmentDetected = i+1;
              Serial.print("TARGET DETECTED ");
              Serial.println(segmentDetected);
              Serial.println(distanceFiltered[i]);
              Serial.println("MOTORS ON");
              previousArmMillis = currentMillis;
              flag = 3;
              break;
            }
          }
        } // end of flag 2
    
        else if (flag == 3) {
            digitalWrite(41, LOW);    // turn on blasters
            if (segmentDetected < DISTANCE_BUCKETS / 2) {   // turn CW
                demandRot = (6300/(DISTANCE_BUCKETS / 2))*segmentDetected;
            }
            else if (segmentDetected > DISTANCE_BUCKETS / 2) {  // turn CCW
                demandRot = (6300/(DISTANCE_BUCKETS / 2))*(DISTANCE_BUCKETS-segmentDetected)*-1;
            }           
            flag = 4;
        }
    
        else if (currentMillis - previousArmMillis > 2000 && flag == 4) {
            encoder0Pos = 0;
            encoder1Pos = 1;
            demandRot = 0;
            Serial.println("FIRE 1");
            servo1.write(0);
            previousArmMillis = currentMillis;
            flag = 5;  
        }
    
        else if (currentMillis - previousArmMillis > 700 && flag == 5) {
            Serial.println("FIRE 2");
            servo1.write(180);
            servo2.write(0); 
            previousArmMillis = currentMillis;
            flag = 6;      
        }
    
        else if (currentMillis - previousArmMillis > 700 && flag == 6) {
            servo1.write(180);
            servo2.write(180); 
            previousArmMillis = currentMillis;
            flag = 7;      
        }
    
        else if (currentMillis - previousArmMillis > 500 && flag == 7) {
            Serial.println("MOTORS OFF");
            digitalWrite(41, HIGH);
            previousArmMillis = currentMillis;
            flag = 1;
            setLEDs(50,50,0);     // set the LEDs back to amber          
        } 

   }   // end of timed events for motor PID only.  

   
} // end of main loop




// ************** encoders interrupts **************

// ************** encoder 1 *********************


void doEncoderA(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
 
}

void doEncoderB(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  

}

// ************** encoder 2 *********************

void doEncoderC(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder1PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder1PinB) == LOW) {  
      encoder1Pos = encoder1Pos - 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder1PinB) == HIGH) {   
      encoder1Pos = encoder1Pos - 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;          // CCW
    }
  }
 
}

void doEncoderD(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder1PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder1PinA) == HIGH) {  
      encoder1Pos = encoder1Pos - 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder1PinA) == LOW) {   
      encoder1Pos = encoder1Pos - 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;          // CCW
    }
  }
  

}

// filter funcition
float filter(float prevValue, float currentValue, int filter) {  
  float lengthFiltered =  (prevValue + (currentValue * filter)) / (filter + 1);  
  return lengthFiltered;  
}

// gets what bucket an angle should be recorded to
int getBucket(float angle, int numBuckets){
  float anglePerBucket = 360.f / numBuckets;
  // shift everything by half a bucket so the first bucket is centered on 0
  float centeredAngle = fmod((angle+anglePerBucket/2), 360.f);
  int bucketIndex = (centeredAngle * numBuckets) / 360.f;
  return bucketIndex;
}


void setLEDs(int red, int green, int blue) {
    for (int i = 0; i <= 11; i++) {
        leds[i] = CRGB(red, green, blue);
    }
    FastLED.show();
}
