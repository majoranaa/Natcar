// CURRENT VERSION
// NATCAR KANG - TEAM 21 - UCLA 2015
// RECORD: 10.40sec - 8.46fps looking @ 22cm in front @ MIN_SPEED 120 MAX_SPEED 200

/*******************************************

              STRUCTURE OF CODE

READ VALUES FROM CAMERA
 - Finds maximum value and index in linescan camera sample to determine position & existence of track

PID CONTROL
 - Standard PID to create variable [output], which is the offset angle from MIDPNT
 
SET SPEED
 - Set value of motor speed [motor_speed]. If the magnitude of error is less than UP_THRESH, then we accelerate,
     ie: increase [motor_speed] by ACCEL
 - If magnitude of error is greater than DOWN_THRESH, then we decelerate (decrease [motor_speed] by DECEL)
 
IF THE TRACK IS NOT SEEN (ie: [max_val] is less than LIGHT_THRESH)
 - Look at the position of the last maximum detected when the track was in sight
 - If it's to the left of MIDPNT, change [output] to OFFSET deg in that direction and vice versa
 - Change speed [motor_speed] to minimum MIN_SPEED

Physically write [output+MIDPNT] to servo

Determine if you should fast stop this cycle based on this cycle's diff

Physically write [motor_speed] to motor

delay by DELAY_TIME milliseconds

*******************************************/

#include <Servo.h>
#include "params.h"

#define SERVO 21

// define pins
const int brightness = 20; //A0; A6 camera input
const int SI = 0; //1; to camera - ask for image
const int clk = 1; //4; to camera - clock
const int md_1 = 23; // A9 motor driver 1
const int md_2 = 22; // A8 motor driver 2

// define PID coefficients & PID globals
const float KP = K_P; //0.7;
const float KD = K_D; //0.07; // this should be negative
const float KI = K_I; //0.2;//.0001;
float accum,de,dt,diff,diff_ma;
unsigned long last_time, now;

// construct servo controller
Servo myServo;

// index variables
int i;

// READ VALUES FROM CAMERA globals
int max_val, max_pos, max_pos2;
int mid, val, output, find_sec, last_output;
int right_edge, right_found, right_edge2, right_found2;

int error, last_err;
int last_mid;

#ifdef DEBUG_STOP
int count;
#endif

// MOTOR globals
float motor_speed, this_speed, that_speed;;
int stop_count, stopping;

void setup()
{
  // initialize pins
  myServo.attach(SERVO);
  Serial.begin(9600);
  pinMode(brightness, INPUT);
  pinMode(SI, OUTPUT);
  pinMode(clk, OUTPUT);
  pinMode(md_1, OUTPUT);
  pinMode(md_2, OUTPUT);
  
  // initialize vars
  accum = 0;
  last_time = millis();
  
  val = 0;
  max_val = 0;
  max_pos = 1;
  right_edge = 128;
  right_found = 0;
  find_sec = 0;
  max_pos2 = 1;
  right_edge2 = 128;
  right_found2 = 0;
  
  last_output = 0;
  
  error = 0;
  last_err = 0;
  last_mid = MIDDLE;
  
  #ifdef DEBUG_STOP
  count = 0;
  #endif
  
  motor_speed = (MAX_SPEED+MIN_SPEED)/2;
  stop_count = 0;
  stopping = 0;
  delay(1000);
}

void stop() {
  analogWrite(md_2, 0);
  analogWrite(md_1, 0);
  while(1) {}
}

void loop()
{
  // READ VALUES FROM CAMERA
  max_pos = 1;
  max_val = 0;
  right_edge = 128;
  right_found = 0;
  
  digitalWrite(SI,HIGH);
  digitalWrite(clk,HIGH);
  digitalWrite(SI,LOW);
  digitalWrite(clk,LOW);
  for (i = 1; i <= 128; i++)
  {
    digitalWrite(clk, HIGH);
    val = analogRead(brightness);
    val =(val*9.)/1024.;
    digitalWrite(clk,LOW);

    if (val > max_val) {
      max_val = val;
      max_pos = i;
      right_found = 0;
    } else if (val < max_val && !right_found) {
      right_edge = i;
      right_found = 1;
    }
    if (right_found) { // already found candidate for max
      if (val == max_val && !find_sec) {
        max_pos2 = i;
        right_found2 = 0;
        find_sec = 1;
      } else if (val < max_val && !right_found2) {
        right_edge2 = i;
        right_found2 = 1;
        find_sec = 0;
        if (abs(((right_edge2-1 + max_pos2)/2)-last_mid) < abs(((right_edge-1 + max_pos)/2)-last_mid)) { // this max is closer than the previous max
          max_pos = max_pos2;
          right_edge = right_edge2;
        }
      }
    }
    
    #ifdef DEBUG_CAM
    Serial.print(val);
    #endif
  }
  
  #ifdef DEBUG_CAM
  Serial.println();
  #endif
  
  mid = (right_edge-1 + max_pos)/2;
  
  #ifdef DEBUG_CAM
  for (i = 1; i < right_edge; i++) {
     if (i == max_pos || i == right_edge-1) {
       Serial.print("|");
     } else if (i == mid && i != last_mid) {
       Serial.print("*");
     } else if (i == mid && i == last_mid) {
       Serial.print("!");
     } else if (i == last_mid && i != mid) {
       Serial.print("0");
     }else {
       Serial.print(" ");
     }
  }
  Serial.println();
  #endif
  
  // CONTROL ALGORITHM - PID
  error = mid - MIDDLE;
  now = millis();
  dt = ((float)(now-last_time))/1000;
  de = ((float)error)*dt;
  if (((de<0)&&(accum>0))||((de>0)&&(accum<0))) accum = de; // reset integral if the car crosses the line. We want this so that the car will account for turns but won't
                                                            // mess with control once the car crosses
  else accum += de;
  diff = ((float)(error - last_err))/dt;
  output = KP*error + KD*diff + KI*accum; // /(now/3000); // we may consider this term to attenuate I over time
  
  #ifdef DEBUG_CONT
  Serial.print("Output: ");
  Serial.println(output);
  Serial.print("Accum: ");
  Serial.println(accum);
  Serial.print("Diff: ");
  Serial.println(diff);
  #endif
  
  // SET SPEED
  if (error < UP_THRESH && error > -UP_THRESH) motor_speed = (motor_speed>=MAX_SPEED)?MAX_SPEED:motor_speed+ACCEL;
  if (error > DOWN_THRESH || error < -DOWN_THRESH) motor_speed = (motor_speed<=MIN_SPEED)?MIN_SPEED:motor_speed-DECEL;
  
  if (max_val < LIGHT_THRESH) { // if track is not seen
    mid = last_mid;
    error = last_err; // these lines ensure that the last_mid and last_err are the mid position and error from the last cycle when the track was in view
    
    output = last_output;
    //output = (mid<MIDDLE)?-OFFSET:((mid>MIDDLE)?OFFSET:0);
    
    #ifdef DEBUG_CONT
    Serial.print("Off the line with output: ");
    Serial.println(output);
    #endif
    
    motor_speed = MIN_SPEED;
    #ifdef DEBUG_STOP
    count++;
    if (count > THRESH) {
      stop();
    }
    #endif
  } else { // sees track
    #ifdef DEBUG_STOP
    count = 0;
    #endif
    // DETERMINE FAST STOP
    //if (abs(diff) > DIFF_THRESH && motor_speed > ((1-MAX_WEIGHT)*MIN_SPEED + MAX_WEIGHT*MAX_SPEED)) {
    if (abs(error) > ERR_THRESH) { // && motor_speed > ((1-MAX_WEIGHT)*MIN_SPEED + MAX_WEIGHT*MAX_SPEED)) {
      stop();
      stopping = 1;
      stop_count = 0;
    }
    if (stopping) {
      if (stop_count < NUM_STOP) {
        this_speed = 0;
        that_speed = 0;
        stop_count++;
      } else {
        stopping = 0;
        this_speed = motor_speed;
        that_speed = 0;
      }
    } else {
      this_speed = motor_speed;
      that_speed = 0;
    }
  }
  
  #ifdef DEBUG_MOTOR
  Serial.print("Motor speed: ");
  Serial.println(motor_speed);
  Serial.print("ERROR: ");
  Serial.println(error);
  Serial.print("Max pos: ");
  Serial.println(mid);
  #endif
  
  last_err = error;
  last_time = now;
  last_mid = mid;
  last_output = output;
  
  // SET SERVO
  //myServo.write((((output + MIDPNT)>RIGHT_MAX)?RIGHT_MAX:(((output+MIDPNT)<LEFT_MAX)?LEFT_MAX:output+MIDPNT))+(motor_speed/10)); // attempt to correct midpoint when motor speed changes,
                                                                                                                                   // but this doesn't seem to be relevant anymore
  myServo.write(((output + MIDPNT)>RIGHT_MAX)?RIGHT_MAX:(((output+MIDPNT)<LEFT_MAX)?LEFT_MAX:output+MIDPNT));
  //myServo.write(MIDPNT);

  // WRITE SPEED
  #ifndef NO_MOTOR
  //analogWrite(md_2, motor_speed);
  analogWrite(md_2, this_speed);
  analogWrite(md_1, that_speed);
  #endif
  
  /*#ifdef DEBUG_STOP
  if (millis() > 8000) {
    analogWrite(md_1, 0);
    analogWrite(md_2,0);
  }
  #endif*/
  
  delay(DELAY_TIME);
}
