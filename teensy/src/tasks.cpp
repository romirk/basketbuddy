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
    serial_read();
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
    set_motors();
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
        // send();
        steppers_sleep();
        send_lift_update = false;
    }
    else if (!send_lift_update && (BasketBuddy::lift.state == LS_MovingUp || BasketBuddy::lift.state == LS_MovingDown))
        send_lift_update = true;

    else if (send_lift_update && (BasketBuddy::lift.state == LS_Up || BasketBuddy::lift.state == LS_Down))
    {
        send_lift_update = false;
        // send();
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
            // send("Task failed with code " + String(result));
        }
    }
}