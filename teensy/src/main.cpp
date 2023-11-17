/**
 * @file main.cpp
 * @brief Main source file for the BasketBuddy project.
 * @author Romir Kulshrestha
 * @date 2022-12-27
 * @version 0.1
 */

#include <basketbuddy/main.h>

constexpr uint8_t CLOCK_PERIOD = 1000 / CLOCK_RATE;  // ms
uint8_t loop_number = 0;

void setup() {
    // set pinmodes
    setPinModes();

    // enable killswitch interrupt before powering on
    attachInterrupt(digitalPinToInterrupt(P_KILLSWITCH), estop_isr, CHANGE);

    // pull power relay high, this will keep the motors powered
    digitalWrite(P_POWER, HIGH);
    attachInterrupt(digitalPinToInterrupt(P_BUTTON_STATE), shutdown_isr,
                    FALLING);

    // initialize hardware
    // initialize();
    setup_led();

    attachInterrupt(digitalPinToInterrupt(P_BUTTON_STATE), shutdown_isr,
                    RISING);
}

auto last_time = millis();
void loop() {
    //if (!Serial) initialize();

    if (lift_sync_step()) {
        delayMicroseconds(STEPPER_STEP_DELAY);
        return;
    }

    led_loop();
    read_and_exec();

    // delay(last_time + CLOCK_PERIOD - millis());
    delay(CLOCK_PERIOD);
    last_time = millis();
    loop_number = (loop_number + 1) % (CLOCK_RATE);
}
