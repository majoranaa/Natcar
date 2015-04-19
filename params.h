#ifndef PARAMS_H
#define PARAMS_H

// CURRENTLY LOOKING:
// 27cm

// debugging
#define DEBUG_CAM           // uncomment to print out debugging info about the linescan cam
#define DEBUG_CONT          // uncomment to print out debugging info about PID control
#define DEBUG_MOTOR         // uncomment to print out debugging info about motor speed & error
//#define NO_MOTOR
//#define DEBUG_STOP          // uncomment to stop car when it goes THRESH cycles without seeing the track

#define LEFT_MAX     55     // Maximum angle that servo can turn on one side
#define RIGHT_MAX    145    // Maximum angle that servo can turn on other side
#define OFFSET       20     // DEPRECATED; When Natcar doesn't see the track anymore, turn OFFSET degrees from the MIDPNT in the direction of the previously seen maximum
#define MIDPNT       100    // The experimentally determined midpoint of the servo (ideally 90deg)
#define MIDDLE       64     // The index of the center of the track in a sample from the linescan camera
#define MAX_SPEED    140    // Maximum speed (0-255) that car can go
#define MIN_SPEED    100    // Minimum speed (0-255) that car can go
#define THRESH       100    // Number of iterations of loop() that the car will go without seeing the track until it stops (assuming DEBUG_STOP is defined)
#define UP_THRESH    21     // The maximum error (distance of track from MIDDLE) allowed under which the car will accelerate at rate ACCEL
#define DOWN_THRESH  22     // The minimum error required above which the car will decelerate at rate DECEL
#define ACCEL        .6     // Acceleration rate
#define DECEL        12     // Deceleration rate
#define LIGHT_THRESH 2      // Consider the maximum from a sample of the linescan. If it is below LIGHT_THRESH then conclude that the track is not seen
#define DELAY_TIME   12     // Amount of time to delay (in milliseconds) between iterations of loop(). Important because smaller delays doesn't allow the camera
                            //     to refresh, ie: it allows in less light so all of the values it reporats are smaller
#define DIFF_THRESH  400    // The diff threshold above which we fast stop this cycle
//#define ERR_THRESH   33     // Error threshold equivalent for DIFF_THRESH
#define BACK_SPEED   100     // Speed to go backwards when fast stopping
#define NUM_STOP     5      // Cycles to stop for each time; appx 100 cycles per second
#define MAX_WEIGHT   .5     // Weight to put on MAX_SPEED when calculating speed threshold for fast stop

#define K_P          0.7
#define K_D          0.09
#define K_I          0.2

#endif
