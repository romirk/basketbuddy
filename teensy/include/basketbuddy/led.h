/**
 * @file led.h
 * @brief LED header file.
 * @author Jordi van Setten
 * @date 2023-01-19
 * @version 0.1
 * 
 * This file contains the LED control for the BasketBuddy project.
*/

#pragma once

#include <FastLED.h>
#include <basketbuddy/basketbuddy.h>

#define NUM_LEDS 8
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define DATA_PIN P_LED_PWM_PIN
// #define CLK_PIN       4
#define VOLTS 5
#define MAX_MA 480
#define UPDATES_PER_SECOND 100
#define BRIGHTNESS 122
#define WAVELENGTH (NUM_LEDS * 4)

void setup_led();
void test1();
int led_loop();
void set_turning_leds(int);
