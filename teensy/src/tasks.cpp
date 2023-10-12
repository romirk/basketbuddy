/**
 * @file tasks.cpp
 * @brief Tasks source file.
 * @author Romir Kulshrestha
 * @date 2022-01-29
 * @version 0.1
 *
 * This file contains the implementations of the tasks for the BasketBuddy project.
 */
#include <basketbuddy/tasks.h>

#define E_SERIAL_UNAVAILABLE 1
#define E_PARSE_ERROR 2
#define E_CHECKSUM_ERROR 3
#define E_PROCESS_ERROR 4

std::vector<TaskFunction> Tasks::tasks;
unsigned Tasks::task_list_size = 0;

/**
 * @brief Task for communications.
 * @return int
 * @retval 0 if successful
 * @retval 1 if failed (serial unavailable)
 * @retval 2 if failed (parse error)
 * @retval 3 if failed (checksum error)
 * @retval 4 if failed (process error)
 */
int task_comms()
{
    if (!Serial)
        return E_SERIAL_UNAVAILABLE;

    char msg[CMD_BUFFER_SIZE];
    short i = 0;
    while (Serial.available() >= CMD_BUFFER_SIZE)
    {
        if (i++ > 3)
            // only read 4 messages at a time
            return 0;

        Serial.readBytes(msg, CMD_BUFFER_SIZE);
        Message *m = Message::parse(msg);
        if (m == nullptr)
        {
            // broken pipe
            handle_broken_pipe();
            return E_PARSE_ERROR;
        }
        // LogMessage::send("Received message: " + String(m->type) + " " + String(m->data));
        // validate message
        if (!m->validate())
        {
            SignalMessage::send(S_Error, " checksum error");
            // LogMessage::send("expected: " + String(m->gen_checksum()) + " got: " + String(m->checksum));
            return E_CHECKSUM_ERROR;
        }
        // LogMessage::send("Validated message: " + String(m->type) + " " + String(m->data));
        // process message
        return m->process();
    }
    return 0;
}

/**
 * @brief Task for locomotion.
 * @return int
 * @retval 0 if successful
 * @retval 1 if failed (not implemented)
 *
 * @todo Implement error handling.
 *
 * @details
 * This task is responsible for locomotion. It is called every clock cycle.
 */
int task_locomotion()
{
    auto *velocity = &BasketBuddy::velocity;
    auto t = velocity->countdown();
    auto elapsed = velocity->elapsed();
    auto left = velocity->left();
    auto right = velocity->right();
    auto applied = velocity->is_applied();

    if ((!t || !(left | right)))
    // if countdown is 0 or left and right are 0
    {
        if (applied)
            return 0;
        motors.stop();
        velocity->reset();
        velocity->apply();
        MoveMessage::send(*velocity);
    }
    else
    {
        if (applied)
            return 0;

        auto coeff = elapsed < 1000 ? (1 - easeInExpo(elapsed / 1000.0)) : 0;

        auto left_calculated = abs(left + (240 - left) * coeff);
        auto right_calculated = abs(right + (240 - right) * coeff);

        motors.setSpeedA(left_calculated);
        motors.setSpeedB(right_calculated);

        if ((left ^ right) >> 31)
        // if left and right have different signs
        {
            if (left > 0)
            {
                motors.forwardA();
                motors.backwardB();
            }
            else
            {
                motors.backwardA();
                motors.forwardB();
            }
        }
        else
        {
            if (left > 0)
                motors.forward();
            else
                motors.backward();
        }

        if (!t || elapsed > 1000)
            velocity->apply();
        MoveMessage::send(Velocity(left_calculated, right_calculated));
    }
    return 0;
}

bool send_lift_update = false;

int task_lift()
{
    if (BasketBuddy::lift.state == LS_Fault)
    {
        async_lift_stop();
        steppers_enable();
        lift_sync();
        LiftMessage::send();
        steppers_sleep();
        send_lift_update = false;
    }
    else if (!send_lift_update && (BasketBuddy::lift.state == LS_MovingUp || BasketBuddy::lift.state == LS_MovingDown))
        send_lift_update = true;

    else if (send_lift_update && (BasketBuddy::lift.state == LS_Up || BasketBuddy::lift.state == LS_Down))
    {
        send_lift_update = false;
        LiftMessage::send();
    }

    return 0;
}

void Tasks::init()
{
    register_task(task_comms);
    register_task(task_locomotion);
    // register_task(led_loop);
    register_task(task_lift);
}

void Tasks::run()
{
    for (auto task : tasks)
    {
        auto result = (task)();
        if (result)
        {
            LogMessage::send("Task failed with code " + String(result));
        }
    }
}