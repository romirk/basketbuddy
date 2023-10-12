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

void setup_led()
{
    delay(1000); // power-up safety delay
    FastLED.addLeds<LED_TYPE, P_LED_PWM_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(BRIGHTNESS);

    currentPalette = RainbowColors_p;
    currentBlending = LINEARBLEND;
}

void set_color_leds(CRGB color)
{
    for (int i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = color;
    }
}

double sin_led(uint32_t x, uint32_t t, int direction)
{
    const auto lambda = 2 * PI / NUM_LEDS;
    const auto omega = direction * 2 * PI / 100;
    const auto phi = 0;
    const auto A = BRIGHTNESS;

    return A * sin(lambda * x + omega * t + phi);
}

void set_turning_leds(int direction)
{
    uint32_t t = (millis() * UPDATES_PER_SECOND / 1000);
    for (int i = 0; i < NUM_LEDS; i++)
    {
        float brightnessWave = max(sin_led(i, t, direction), 0);
        leds[i] = ColorFromPalette(RainbowColors_p, 1, brightnessWave, LINEARBLEND);
    }
    // LogMessage::send(String(leds[0].r) + " " + String(leds[0].g) + " " + String(leds[0].b));
}

int led_loop()
{
    if (BasketBuddy::estop == EstopState::ES_Enabled)
    {
        set_color_leds(CRGB::Red);
    }
    else if (BasketBuddy::robot_state == RobotState::R_Autonomous)
    {
        if (BasketBuddy::velocity.right() < BasketBuddy::velocity.left())
        {
            set_turning_leds(1);
        }
        else if (BasketBuddy::velocity.right() > BasketBuddy::velocity.left())
        {
            set_turning_leds(-1);
        }
        else
        {
            set_color_leds(CRGB::Green);
        }
    }
    else if (BasketBuddy::lift.state == LiftState::LS_Up || BasketBuddy::lift.state == LiftState::LS_Down)
    {
        set_color_leds(CRGB::Orange);
    }
    else
    {
        set_color_leds(CRGB::Purple);
    }
    FastLED.show();
    return 0;
}
