/**
 * @file led.cpp
 * @brief LED source file.
 * @author Jordi van Setten
 * @date 2023-01-19
 * @version 0.1
 *
 * This file contains the LED control for the BasketBuddy project.
 */
#include <basketbuddy/led.h>

CRGB leds[NUM_LEDS];

CRGBPalette16 currentPalette;
TBlendType currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;

void setup_led() {
    delay(1000);  // power-up safety delay
    FastLED.addLeds<LED_TYPE, P_LED_PWM_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    //.setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(BRIGHTNESS);

    currentPalette = RainbowColors_p;
    currentBlending = LINEARBLEND;
}

void set_color_leds(CRGB color) {
    for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = color;
    }
}

double sin_led(uint32_t x, uint32_t t, int direction) {
    const auto lambda = 2 * PI / NUM_LEDS;
    const auto omega = direction * 2 * PI / 100;
    const auto phi = 0;
    const auto A = BRIGHTNESS;

    return A * sin(lambda * x + omega * t + phi);
}

void set_turning_leds(int direction) {
    uint32_t t = (millis() * UPDATES_PER_SECOND / 1000);
    for (int i = 0; i < NUM_LEDS; i++) {
        float brightnessWave = max(sin_led(i, t, direction), 0);
        leds[i] =
            ColorFromPalette(RainbowColors_p, 1, brightnessWave, LINEARBLEND);
    }
}

int led_loop() {
    if (BasketBuddy::estop == EstopState::ES_Enabled) {
        set_color_leds(CRGB::Red);
        goto end;
    }

    switch (BasketBuddy::robot_state) {
        case RobotState::R_Autonomous:
            set_color_leds(CRGB::Green);
            break;
        case RobotState::R_Manual:
            set_color_leds(CRGB::LightCyan);
            break;
        case RobotState::R_Ready:
            set_color_leds(CRGB::LightGreen);
            break;
        case RobotState::R_Startup:
            set_color_leds(CRGB::Blue);
            break;
        case RobotState::R_Shutdown:
            set_color_leds(CRGB::Yellow);
            break;
        case RobotState::R_Estop:
            set_color_leds(CRGB::Red);
            break;
        case RobotState::R_Error:
            set_color_leds(CRGB::Pink);
            break;
        case RobotState::R_Alignment:
            set_color_leds(CRGB::Orange);
            break;
        case RobotState::R_Lift:
            set_color_leds(CRGB::Orange);
            break;
        default:
            set_color_leds(CRGB::White);
            break;
    }

end:
    FastLED.show();
    return 0;
}
