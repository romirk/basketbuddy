/**
 * @file definitions.h
 * @brief Definitions header file.
 * @author Romir Kulshrestha
 * @date 2022-12-27
 * @version 0.1
 *
 * This file contains the definitions for the BasketBuddy project.
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <L298NX2.h>
#include <Adafruit_INA260.h>
#include <basketbuddy/pinout.h>

// constants

#define VERSION "1.0.0" // version number
#define DATE    "Feb 2023"
#define VERSION_DATE VERSION " " DATE

#define CLOCK_RATE 200     // clock rate in Hz
#define SERIAL_TIMEOUT 100 // serial timeout in ms
#define CMD_BUFFER_SIZE 32 // command buffer size in bytes

#define TYPE_OFFSET 48   // arbitrary offset for message types, currently set to '0'
#define SIGNAL_OFFSET 97 // arbitrary offset for signal types, currently set to 'a'

#define STEPPER_TOP_HEIGHT 1000
#define STEPPER_BOTTOM_HEIGHT 0
#define STEPPER_RAISING_SPEED 400
#define STEPPER_LOWERING_SPEED -400
#define STEPPER_STARTING_POSITION 0

#define MINIMUM_VELOCITY_DURATION 10   // minimum duration in ms
#define MAXIMUM_VELOCITY_DURATION 5000 // maximum duration in ms

#define STEPPER_PULSE_DELAY 5   // microseconds
#define STEPPER_STEP_DELAY 1000 // microseconds

#define STEPPER_STEP_HZ 1000
#define STEPPER_ASYNC_TIME 9000 // microseconds ~= 9000 steps

#define LIFT_COMPLETED 0
#define LIFT_TERMINATED_EARLY 1
#define LIFT_FAILED 2
#define LIFT_INPROGRESS 3
#define LIFT_ASYNC_STARTED 4

#define MAX_STEPS 14000 // 1 step = 0.01 mm
#define LIFT_UP_POSITION 13000
#define LIFT_DOWN_POSITION 0

// hardware

extern Adafruit_INA260 ina260_A;
extern Adafruit_INA260 ina260_B;
extern Adafruit_INA260 ina260_C;

extern L298NX2 motors;

// state definitions

enum RobotState
{
    R_Startup,
    R_Ready,
    R_Autonomous,
    R_Manual,
    R_Lift,
    R_Alignment,
    R_Error,
    R_Estop,
    R_Shutdown,

};

class Message;
enum MessageType
{
    M_Log = TYPE_OFFSET,
    M_Signal,
    M_Move,
    M_Lift,
    M_Telemetry,
};

enum SignalType
{
    S_Startup = SIGNAL_OFFSET,
    S_Shutdown,
    S_Estop,
    S_EstopRelease,
    S_Error,
    S_ErrorRelease,
    S_Heartbeat,
    S_Acknowledge,
};

enum EstopState
{
    ES_Disabled,
    ES_Enabled,
};

enum LiftState
{
    LS_Down,
    LS_MovingDown,
    LS_MovingUp,
    LS_Up,
    LS_Fault
};

typedef int (*TaskFunction)(void);
typedef uint8_t status_t;

typedef struct
{
    uint32_t position;
    LiftState state;
} __attribute__((packed)) Lift;

/**
 * @brief The Velocity struct
 * This struct represents the velocity of the robot.
 * It contains the x, y, z, and w velocities.
 * It also contains the end time.
 */
typedef struct VelocityStruct
{
private:
    int phi_l;
    int phi_r;
    uint32_t end_time;
    uint32_t start_time;

    bool applied = false;

public:
    VelocityStruct() : phi_l(0), phi_r(0), end_time(millis()), start_time(millis()) {}
    VelocityStruct(int phi_l, int phi_r, uint32_t duration) : phi_l(phi_l), phi_r(phi_r), end_time(millis() + duration), start_time(millis()) {}
    VelocityStruct(int phi_l, int phi_r) : VelocityStruct(phi_l, phi_r, 0) {}

    /**
     *
     * @brief get time left
     */
    uint32_t countdown();

    /**
     *
     * @brief get time elapsed
     */
    uint32_t elapsed();

    /**
     * @brief set velocity
     * @param phi_l left velocity
     * @param phi_r right velocity
     * @param duration duration in ms
     */
    void set(int, int, uint32_t);

    /**
     * @brief reset velocity
     */
    void reset();

    /**
     * @brief stop
     */
    void brake();

    /**
     * @brief get left velocity
     */
    int left();

    /**
     * @brief get right velocity
     */
    int right();

    /**
     * @brief check if velocity is applied
     */
    bool is_applied();

    /**
     * @brief apply velocity
     */
    void apply();

} __attribute__((packed)) Velocity;