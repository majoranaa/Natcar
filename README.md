#NATCAR KONG - UCLA 2015
##Checklist
 - Tires: None
 - Camera focal length
 - Camera height - 5in
 - Camera angle - 27cm

##Structure of Code
###READ VALUES FROM CAMERA
 - Finds maximum value and index in linescan camera sample to determine position & existence of track

###PID CONTROL
 - Standard PID to create variable [output], which is the offset angle from MIDPNT
 
###SET SPEED
 - Set value of motor speed [motor_speed]. If the magnitude of error is less than UP_THRESH, then we accelerate,
     ie: increase [motor_speed] by ACCEL
 - If magnitude of error is greater than DOWN_THRESH, then we decelerate (decrease [motor_speed] by DECEL)
 
###IF THE TRACK IS NOT SEEN
 - ie: [max_val] is less than LIGHT_THRESH
 - Look at the position of the last maximum detected when the track was in sight
 - If it's to the left of MIDPNT, change [output] to OFFSET deg in that direction and vice versa
 - Change speed [motor_speed] to minimum MIN_SPEED

Physically write [output+MIDPNT] to servo and [motor_speed] to motor

Delay by DELAY_TIME milliseconds

##Things to do
 - Look further and go faster (try 75 deg angle. if too little light maybe add light. increase MAX_SPEED to 255 and MIN_SPEED to 155?)
 - Gather linescan data? Could try supervised learning
 	- Would need to get data onto computer or smth
 		- Wireless Communication
 		- Save to file (eg: EEPROM/flash) and then write serially to comp (pyserial) afterwards using another program
 - Use reinforcement learning in the Teensy & save the state table in EEPROM