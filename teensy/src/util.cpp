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
    // ina260_A.begin(0x40, &Wire);
    // ina260_B.begin(0x40, &Wire1);
    // ina260_C.begin(0x40, &Wire2);

    // initialize serial line to Jetson

    if (!Serial) {
        Serial.setTimeout(SERIAL_TIMEOUT);
        Serial.begin(115200);
        while (!Serial) {
            // wait for serial to initialize
        }
    }

    digitalWrite(13, HIGH);

    // signal communication start with Jetson
    // send(S_Startup);
    // blocking_wait_for(S_Startup);
    // send("BasketBuddy Core");
    // send(VERSION_DATE);

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
                             //2nd iteration, this can be removed

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


uint8_t buffer[8];

bool serial_read() {
    bool r = false;
    while (Serial.available()) {
        auto c = Serial.read();
        switch (c) {
            case 'V':
                Serial.readBytes(buffer, 2);
                rover_velocity.linear = buffer[0];
                rover_velocity.angular = buffer[1];
                r = true;
                // Serial.write('A');
                // Serial.write(rover_velocity.linear);
                // Serial.write(rover_velocity.angular);
                // Serial.write('\n');
                rover_velocity.linear =
                    map(rover_velocity.linear, 0, 200, -100, 100);
                rover_velocity.angular =
                    map(rover_velocity.angular, 0, 200, -100, 100);
                rover_velocity.linear ? set_motors() : stop();
                break;

            case 'S':
                stop();
                r = true;
                break;
            case 'B':
                brake();
                r = true;
                break;
            default:
                break;
        }
    }
    return r;
}

void set_motors() {
    // differential drive
    float left, right;
    forward_kinematics(rover_velocity.linear, rover_velocity.angular, left,
                       right);

    auto left_v = constrain((int)(255 * left / V_MAX), -255, 255);
    auto right_v = constrain((int)(255 * right / V_MAX), -255, 255);

    // set motor speeds
    if (left_v > 0) {
        analogWrite(PIN_M_L1, left_v);
        digitalWrite(PIN_M_L2, LOW);
    } else {
        analogWrite(PIN_M_L2, -left_v);
        digitalWrite(PIN_M_L1, LOW);
    }

    if (right_v > 0) {
        analogWrite(PIN_M_R1, right_v);
        digitalWrite(PIN_M_R2, LOW);
    } else {
        analogWrite(PIN_M_R2, -right_v);
        digitalWrite(PIN_M_R1, LOW);
    }
}

void stop() {
    digitalWrite(PIN_M_L2, LOW);
    digitalWrite(PIN_M_L1, LOW);
    digitalWrite(PIN_M_R2, LOW);
    digitalWrite(PIN_M_R1, LOW);
}

void brake() {
    digitalWrite(PIN_M_L2, HIGH);
    digitalWrite(PIN_M_L1, HIGH);
    digitalWrite(PIN_M_R2, HIGH);
    digitalWrite(PIN_M_R1, HIGH);
}

void motor_test() {
    // ramp up forward
    digitalWrite(PIN_M_L1, LOW);
    for (int i = 0; i < 255; i++) {
        analogWrite(PIN_M_L2, i);
        delay(10);
    }

    // forward full speed for one second
    delay(1000);

    // ramp down forward
    for (int i = 255; i >= 0; i--) {
        analogWrite(PIN_M_L2, i);
        delay(10);
    }

    // ramp up backward
    digitalWrite(PIN_M_L2, LOW);
    for (int i = 0; i < 255; i++) {
        analogWrite(PIN_M_L1, i);
        delay(10);
    }

    // backward full speed for one second
    delay(1000);

    // ramp down backward
    for (int i = 255; i >= 0; i--) {
        analogWrite(PIN_M_L1, i);
        delay(10);
    }
}

