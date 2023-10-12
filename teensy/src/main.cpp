/**
 * @file main.cpp
 * @brief Main source file for the BasketBuddy project.
 * @author Romir Kulshrestha
 * @date 2022-12-27
 * @version 0.1
 */

#include <basketbuddy/main.h>

constexpr uint8_t CLOCK_PERIOD = 1000 / CLOCK_RATE; // ms
uint8_t loop_number = 0;

void setup()
{
  // set pinmodes
  setPinModes();

  // enable killswitch interrupt before powering on
  attachInterrupt(digitalPinToInterrupt(P_KILLSWITCH), estop_isr, CHANGE);

  // pull power relay high, this will keep the motors powered
  digitalWrite(P_POWER, HIGH);

  // initialize tasks
  Tasks::init();

  // initialize hardware
  initialize();

  // attachInterrupt(digitalPinToInterrupt(P_BUTTON_STATE), shutdown_isr, RISING);
}

void loop()
{
  auto t_start = millis();
  auto t_end = t_start + CLOCK_PERIOD;

  if (BasketBuddy::estop && BasketBuddy::robot_state != R_Estop)
    BasketBuddy::emergency_stop();
  else if (BasketBuddy::robot_state == R_Shutdown)
    BasketBuddy::shutdown();

  Tasks::run();

  if (!Serial)
  {
    // reset serial if it is disconnected
    initialize();
  }
  delay(t_end - millis());
  loop_number = (loop_number + 1) % (CLOCK_RATE);
  // if (loop_number == 0)
  //   LogMessage::send(String(BasketBuddy::robot_state + '0') + " " + String(BasketBuddy::estop + '0'));
}
