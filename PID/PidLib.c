/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2012                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     PidLib.c                                                     */
/*    Author:     James Pearman                                                */
/*    Created:    24 Oct 2012                                                  */
/*                                                                             */
/*    Revisions:                                                               */
/*                30 Oct 2012 - Added Kbias                                    */
/*-----------------------------------------------------------------------------*/

#ifndef __PIDLIB__
#define __PIDLIB__

// Version 1.01
#define kPidLibVersion   101

// We make extensive use of pointers so need recent versions of ROBOTC
#ifndef kRobotCVersionNumeric
#include "FirmwareVersion.h"
#endif

#if kRobotCVersionNumeric < 351
#error "PidLib requires ROBOTC Version 3.51 or newer"
#endif

// _Target_Emulator_ is new for 3.55, define for older versions
#if kRobotCVersionNumeric < 355
 #if (_TARGET == "Emulator")
  #ifndef _Target_Emulator_
   #define _Target_Emulator_   1
  #endif
 #endif
#endif

// Structure to hold all data for one instance of a PID controller

typedef struct {
    // Turn on or off the control loop
    short        enabled;

    // PID constants, Kbias is used to compensate for gravity or similar
    float        Kp;
    float        Ki;
    float        Kd;
    float        Kbias;

    // working variables
    float        error;
    float        last_error;
    float        integral;
    float        integral_limit;
    float        derivative;
    float        error_threshold;

    // output
    float        drive;
    short        drive_raw;
    short        drive_cmd;

    tSensors     sensor_port;
    short        sensor_reverse;
    TSensorTypes sensor_type;
    long         sensor_value;

    long         target_value;
    } pidController;


// Allow 4 pid controllers
#define MAX_PID               4

// static storage - we have no malloc
static  pidController   _pidControllers[ MAX_PID ];
static  short           nextPidControllerPtr = 0;

// lookup table to linearize control
#define PIDLIB_LUT_SIZE     128
#define PIDLIB_LUT_FACTOR  20.0
#define PIDLIB_LUT_OFFSET    10
static  short   PidDriveLut[PIDLIB_LUT_SIZE];
#define _LinearizeDrive( x )    PidDriveLut[abs(x)] * sgn(x)
void    PidControllerMakeLut();

// This causes the motor never to be given more than a 0.25 drive command due to integral
#define PIDLIB_INTEGRAL_DRIVE_MAX   0.25

/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*  Initialize the PID controller                                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

pidController *
PidControllerInit( float Kp, float Ki, float Kd, tSensors port, short sensor_reverse = 0 )
{
    pidController   *p;

    if( nextPidControllerPtr == MAX_PID )
        return(NULL);

    p = (pidController *)&_pidControllers[ nextPidControllerPtr++ ];

    // pid constants
    p->Kp    = Kp;
    p->Ki    = Ki;
    p->Kd    = Kd;
    p->Kbias = 0.0;

    // zero out working variables
    p->error           = 0;
    p->last_error      = 0;
    p->integral        = 0;
    p->derivative      = 0;
    p->drive           = 0.0;
    p->drive_cmd       = 0;
    if(Ki != 0)
        p->integral_limit  = (PIDLIB_INTEGRAL_DRIVE_MAX / Ki);
    else
        p->integral_limit  = 0;

    p->error_threshold = 10;

    // sensor port
    p->sensor_port     = port;
    p->sensor_reverse  = sensor_reverse;
    p->sensor_type     = SensorType[ port ];
    p->sensor_value    = 0;

    p->target_value    = 0;

    // We need a valid sensor for pid control, pot or encoder
    if( ( p->sensor_type == sensorPotentiometer ) ||
        ( p->sensor_type == sensorQuadEncoder ) ||
        ( p->sensor_type == sensorQuadEncoderOnI2CPort )
      )
        p->enabled    = 1;
    else
        p->enabled    = 0;

    PidControllerMakeLut();

    return(p);
}

/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*  Initialize the PID controller - includes bias                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

pidController *
PidControllerInit( float Kp, float Ki, float Kd, float Kbias, tSensors port, short sensor_reverse = 0 )
{
    pidController   *p;
    p = PidControllerInit( Kp, Ki, Kd, port, sensor_reverse );
    if( p != NULL)
        p->Kbias = Kbias;

    return(p);
}

/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*  Update the process command                                                 */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

short
PidControllerUpdate( pidController *p )
{
    if( p == NULL )
        return(0);

    if( p->enabled )
        {
        // check for sensor port
        // otherwise externally calculated error
        if( p->sensor_port >= 0 )
            {
#ifdef _Target_Emulator_
            int inc = p->drive_cmd / 8;
            p->sensor_value += inc;
#else
            // Get raw position value, may be pot or encoder
            p->sensor_value = SensorValue[ p->sensor_port ];
#endif

            // A reversed sensor ?
            if( p->sensor_reverse )
                {
                if( p->sensor_type == sensorPotentiometer )
                    // reverse pot
                    p->sensor_value = 4095 - p->sensor_value;
                else
                    // reverse encoder
                    p->sensor_value = -p->sensor_value;
                }

            p->error = p->target_value - p->sensor_value;
            }

        // force error to 0 if below threshold
        if( abs(p->error) < p->error_threshold )
            p->error = 0;

        // integral accumulation
        if( p->Ki != 0 )
            {
            p->integral += p->error;

            // limit to avoid windup
            if( abs( p->integral ) > p->integral_limit )
                p->integral = sgn(p->integral) * p->integral_limit;
            }
        else
            p->integral = 0;

        // derivative
        p->derivative = p->error - p->last_error;
        p->last_error = p->error;

        // calculate drive - no delta T in this version
        p->drive = (p->Kp * p->error) + (p->Ki * p->integral) + (p->Kd * p->derivative) + p->Kbias;

        // drive should be in the range +/- 1.0
        if( abs( p->drive ) > 1.0 )
            p->drive = sgn(p->drive);

        // final motor output
        p->drive_raw = p->drive * 127.0;
        }

    else
        {
        // Disabled - all 0
        p->error      = 0;
        p->last_error = 0;
        p->integral   = 0;
        p->derivative = 0;
        p->drive      = 0.0;
        p->drive_raw  = 0;
        }

    // linearize - be careful this is a macro
    p->drive_cmd = _LinearizeDrive( p->drive_raw );

    // return the thing we are really interested in
    return( p->drive_cmd );
}


/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*  Create a power based lut                                                   */
/*                                                                             */
/*-----------------------------------------------------------------------------*/


void
PidControllerMakeLut()
{
    int   i;
    float x;

    for(i=0;i<PIDLIB_LUT_SIZE;i++)
        {
        // check for valid power base
        if( PIDLIB_LUT_FACTOR > 1 )
            {
            x = pow( PIDLIB_LUT_FACTOR, (float)i / (float)(PIDLIB_LUT_SIZE-1) );

            if(i >= (PIDLIB_LUT_OFFSET/2))
               PidDriveLut[i] = (((x - 1.0) / (PIDLIB_LUT_FACTOR - 1.0)) * (PIDLIB_LUT_SIZE-1-PIDLIB_LUT_OFFSET)) + PIDLIB_LUT_OFFSET;
            else
               PidDriveLut[i] = i * 2;
            }
        else
            {
            // Linear
            PidDriveLut[i] = i;
            }
        }
}

#endif  // __PIDLIB__
