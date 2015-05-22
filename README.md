#NATCAR KONG - UCLA 2015

##Records
 - IEEE Lab 4/17/2015 - 88ft in 10.40sec = 8.46fps looking @ 22cm in front @ MIN_SPEED 120 MAX_SPEED 200
 - IEEE Lab 4/18/2015 - 88ft in 10.03sec = 8.77fps
 - UCSD Grand PrIEEE 4/19/2015 - 170ft in 22.68sec = 7.496fps

##Checklist
 - Tires: Buggy
 - Camera focal length
 - Camera height - 5 7/8in
 - Camera angle - 42cm
 - With weight
 - Battery

##Things to do
 - To prevent tipping we should add weighs at a low position, move the wheels out more, or aadd "wings" to push the car back upright
 	- Should consider tires made specifically for carpet for optimal traction
 - Look further and go faster (try 75 deg angle. if too little light maybe add light. increase MAX_SPEED to 255 and MIN_SPEED to 155?)
 - Use encoder to measure true speed and adjust speed accordingly
 - Use second camera to look much further
 - Polish fast stop to only detect curves (second derivative?)
 - Gather linescan data? Could try supervised learning
 	- Would need to get data onto computer or smth
 		- Wireless Communication
 		- Save to file (eg: EEPROM/flash) and then write serially to comp (pyserial) afterwards using another program
 - Use reinforcement learning in the Teensy & save the state table in EEPROM

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
 - ie: [max_val] is less than LIGHT_THRESH or error is greater than ERR_THRESH or the width of the track is too large
 - Look at the position of the last maximum detected when the track was in sight
 - If it's to the left of MIDPNT, change [output] to OFFSET deg in that direction and vice versa
 - Change speed [motor_speed] to minimum MIN_SPEED

Physically write [output+MIDPNT] to servo

##Determine if you should fast stop NUM_STOP cycles based on if this cycle's diff is greater than DIFF_THRESH or if the track isn't seen
 - don't fast stop again until the diff falls below DIFF_THRESH again

Physically write [motor_speed] to motor

Dealy by [exp_delay] milliseconds, which changes based on the value of the maximum light reading