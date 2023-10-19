/**
 * @file util.cpp
 * @brief Utility source file for the BasketBuddy project.
 * @author Romir Kulshrestha
 * @date 2023-01-13
 * @version 0.1
 */

#include <basketbuddy/led.h>
#include <basketbuddy/util.h>

void initialize() {
    digitalWrite(13, LOW);

    noInterrupts();
    BasketBuddy::robot_state = R_Startup;
    BasketBuddy::estop = ES_Disabled;
    interrupts();

    // initialize hardware

    lift_sync();
    steppers_sleep();

    // initialize serial line to Jetson

    if (!Serial) {
        Serial.setTimeout(SERIAL_TIMEOUT);
        Serial.begin(115200);
        while (!Serial) {
            // wait for serial to initialize
        }
    }

    ina260_A.begin(0x40, &Wire2);
    digitalWrite(13, HIGH);

    noInterrupts();
    BasketBuddy::robot_state = R_Ready;
    interrupts();
}

float Q_rsqrt(float number) {
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y = number;
    i = *(long *)&y;            // evil floating point bit level hacking
    i = 0x5f3759df - (i >> 1);  // what the fuck?
    y = *(float *)&i;
    y = y * (threehalfs -
             (x2 * y * y));  // 1st iteration
                             //	y  = y * ( threehalfs - ( x2 * y * y ) );   //
                             // 2nd iteration, this can be removed

    return y;
}

float easeInOutCubic(float x) {
    return x < 0.5 ? 4 * x * x * x : 1 - pow(-2 * x + 2, 3) / 2;
}

String padString(String str, unsigned len, char pad) {
    while (str.length() < len) str += pad;
    return str.substring(0, len);
}

String padString(String str, unsigned len) { return padString(str, len, ' '); }

String padString(const char *str, unsigned len) {
    return padString(String(str), len);
}

void scanI2C() {
    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);
            Serial.println("  !");

            nDevices++;
        } else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");

    delay(5000);  // wait 5 seconds for next scan
}
