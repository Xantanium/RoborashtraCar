#include <Arduino.h>
#include <ESP32Servo.h>
#include <Ps3Controller.h>

// #define GET_MAC
/*

    startButton = start,
    select = invertAxis,
    square = ramp,
    left Trigger = gripper--,
    right trigger = gripper++,
    up = arm++,
    down = arm--,

    circle = gripper to 0,
    left = arm to 0
 */

#ifndef GET_MAC
// PINDEFS
#include "car_pins"

#define PWM_FREQUENCY 1000
#define maxPwm 125

// COMPILE FLAGS
// #define DEBUG
#define MATCH
#define ALT_MAC
//

// ----------------- GLOBALS ----------------- //

// MOTION
float valY, valW;
float rSpeed, lSpeed;
int rCap, lCap;

void drive(float argy, float argw);
void getSpeed(float valY, float valW);
void setMotion(); // handles both traction mode and normal mode
//

// CONTROLLER
float stickY, stickW;
bool sFlag = 0;
bool invertAxis = 0;
bool startButton = 0;

bool ramp = 0;

void notify();
//

// SERVO
Servo gripper, arm;

int gripperAngle, armAngle;
int tempAngle = 0;
bool direction, moving;
//

// TIMER
long debugTimer, currentMillis, pidTimer, startTimer, servoTimer;
//

// ------------------ SETUP ------------------ //

void setup()
{
    Serial.begin(115200);
// Initialize the PS3 controller
#undef ALT_MAC
#ifndef ALT_MAC
    Ps3.begin("00:1a:7d:da:71:15");
#else
    Ps3.begin("ac:15:18:d6:45:fc");
#endif
    Ps3.attach(notify);

    pinMode(RIGHT_PWM_PIN_FRONT, OUTPUT);
    pinMode(RIGHT_DIR_FRONT, OUTPUT);

    pinMode(LEFT_PWM_PIN_FRONT, OUTPUT);
    pinMode(LEFT_DIR_FRONT, OUTPUT);

    pinMode(RIGHT_PWM_PIN_BACK, OUTPUT);
    pinMode(RIGHT_DIR_BACK, OUTPUT);

    pinMode(LEFT_PWM_PIN_BACK, OUTPUT);
    pinMode(LEFT_DIR_BACK, OUTPUT);

    ledcSetup(RIGHT_FRONT_CHANNEL, PWM_FREQUENCY, 8);
    ledcSetup(LEFT_FRONT_CHANNEL, PWM_FREQUENCY, 8);
    ledcSetup(RIGHT_BACK_CHANNEL, PWM_FREQUENCY, 8);
    ledcSetup(LEFT_BACK_CHANNEL, PWM_FREQUENCY, 8);

    // Servo
    gripper.attach(GRIPPER_SERVO);

    arm.attach(ARM_SERVO);

    // Inbuilt LED
    pinMode(2, OUTPUT);

    // USE CHANNELS FOR PWM (ledcWrite) NOT PINS
    ledcAttachPin(RIGHT_PWM_PIN_FRONT, RIGHT_FRONT_CHANNEL);
    ledcAttachPin(RIGHT_PWM_PIN_BACK, RIGHT_BACK_CHANNEL);

    ledcAttachPin(LEFT_PWM_PIN_FRONT, LEFT_FRONT_CHANNEL);
    ledcAttachPin(LEFT_PWM_PIN_BACK, LEFT_BACK_CHANNEL);

    digitalWrite(RIGHT_PWM_PIN_BACK, 0);
    digitalWrite(LEFT_PWM_PIN_BACK, 0);
    digitalWrite(RIGHT_PWM_PIN_FRONT, 0);
    digitalWrite(LEFT_PWM_PIN_FRONT, 0);
}

// ------------------ LOOP ------------------ //

void loop()
{
    currentMillis = millis();
#define DEBUG
#ifdef DEBUG
    if (currentMillis - debugTimer > 300) {
        debugTimer = currentMillis;

        // Serial.printf("rCap: %d, lCap: %d\n", rCap, lCap);
        // Serial.print((rSpeed > 0 ? 0 : 1));
        // Serial.print('\t');
        // Serial.println((lSpeed > 0 ? 0 : 1));
        Serial.printf("gripper: %d\tarm: %d\n", gripperAngle, armAngle);
    }
#endif

    if (Ps3.isConnected()) {
        if (startButton) {
            digitalWrite(2, HIGH);

#ifdef SERV0_TEST
            for (int i = 0; i < 180; i++) {
                gripper.write(i);
                delay(10);
            }
            digitalWrite(GRIPPER_SERVO, 1);
            gripper.write(180);
            delay(1000);
            for (int i = 180; i > 0; i--) {
                gripper.write(i);
                delay(10);
                digitalWrite(GRIPPER_SERVO, 0);
            }
            gripper.write(0);
            delay(1000);

            // if (invertAxis) {
            //     if (moving) {
            //         direction ? tempAngle++ : tempAngle--;
            //         constrain(tempAngle, -90, 90);
            //         gripper.write(tempAngle);
            //     } else {
            //         gripper.write(tempAngle);
            //     }
            // } else {

            // }

#endif
#ifdef MOTOR_TEST
            // digitalWrite(LEFT_DIR_FRONT, 1);
            // ledcWrite(LEFT_FRONT_CHANNEL, 255);

            // delay(2000);
            // digitalWrite(LEFT_DIR_FRONT, 1);
            // ledcWrite(LEFT_FRONT_CHANNEL, 255);

            // delay(2000);
            // digitalWrite(RIGHT_DIR, 1);
            // digitalWrite(LEFT_DIR, 1);

            // // digitalWrite(RIGHT_PWM_PIN, 1
            // // drive(100, 0);
            // ledcWrite(RIGHT_CHANNEL, 255);
            // ledcWrite(LEFT_FRONT_CHANNEL, 255);
            // analogWrite(23, stickY);
            // analogWrite(LEFT_PWM_PIN, 250);
            // analogWrite(RIGHT_PWM_PIN, 250);
#endif

#ifdef MATCH
            // constrain(gripperAngle, -90, 90);

            gripper.write(gripperAngle);

            if (((currentMillis - servoTimer) > 5) && (abs(tempAngle - armAngle) > 0)) {

                if (tempAngle == armAngle)
                    arm.write(armAngle);

                tempAngle < armAngle ? tempAngle++ : tempAngle--;
                arm.write(tempAngle);
                servoTimer = currentMillis;
            }

            drive(stickY, stickW);

            getSpeed(valY, valW);
            setMotion();
#endif
        } else {
            digitalWrite(2, LOW);
            // ledcWrite(RIGHT_CHANNEL, 0);
            // ledcWrite(LEFT_FRONT_CHANNEL, 0);
            ledcWrite(RIGHT_BACK_CHANNEL, 0);
            ledcWrite(RIGHT_FRONT_CHANNEL, 0);
            ledcWrite(LEFT_BACK_CHANNEL, 0);
            ledcWrite(LEFT_FRONT_CHANNEL, 0);
        }
    } else {
        digitalWrite(2, LOW);
        ledcWrite(RIGHT_BACK_CHANNEL, 0);
        ledcWrite(RIGHT_FRONT_CHANNEL, 0);
        ledcWrite(LEFT_BACK_CHANNEL, 0);
        ledcWrite(LEFT_FRONT_CHANNEL, 0);
    }
}

// ------------------ FUNCTIONS ------------------ //

void notify()
{
    // StartButton
    if (Ps3.event.button_down.start) {
        startButton = !startButton;
        Serial.println(startButton);
    }

    // Select Button
    if (Ps3.event.button_down.select) {
        invertAxis = !invertAxis;
    }

    // Ramp mode
    if (Ps3.event.button_down.square) {
        ramp = !ramp;
    }

    // Left stick (Y)
    if (abs(Ps3.event.analog_changed.stick.ly) > 1) {
        stickY = Ps3.data.analog.stick.ly;
        if (stickY < 30 && stickY > -30)
            stickY = 0;
    }

    // Right stick (W)
    if (abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 2) {
        stickW = Ps3.data.analog.stick.rx;
        if (stickW < 30 && stickW > -30)
            stickW = 0;
    }

    // Arm Servo
    if (Ps3.event.button_down.up) {
        armAngle = 100;
    } else if (Ps3.event.button_down.left) {
        armAngle = 50;
    } else if (Ps3.event.button_down.down) {
        armAngle = 2;
    }
    // Arm

    // Gripper
    if (Ps3.event.button_down.r2) {
        gripperAngle += 10;
    } else if (Ps3.event.button_down.l2) {
        gripperAngle -= 10;
    } else if (Ps3.event.button_down.circle) {
        gripperAngle = 0;
    }
    // Gripper
}

///////////////////////////////////////////////////////////////////////
void drive(float argy, float argw)
{

    valY = argy / 128.0f;
    valW = argw / 128.0f;
    if (!invertAxis) {
        valW = -1 * valW;
        valY = -1 * valY;
    }
    // getSpeed(valY, valW);
    // setMotion();
}

void getSpeed(float inY, float inW)
{
#define SLOW_W
#ifdef SLOW_W
    inW /= 2;
#endif

    rSpeed = (inY) - (inW);
    lSpeed = (inY) + (inW);
    rCap = round(abs(rSpeed) * maxPwm);
    lCap = round(abs(lSpeed) * maxPwm);
}

void setMotion()
{
    digitalWrite(RIGHT_DIR_FRONT, (rSpeed > 0 ? 0 : 1));
    digitalWrite(RIGHT_DIR_BACK, (rSpeed > 0 ? 0 : 1));

    digitalWrite(LEFT_DIR_FRONT, (lSpeed > 0 ? 0 : 1));
    digitalWrite(LEFT_DIR_BACK, (lSpeed > 0 ? 0 : 1));

    ledcWrite(RIGHT_FRONT_CHANNEL, rCap);
    ledcWrite(LEFT_FRONT_CHANNEL, lCap);

    if (ramp) {
        ledcWrite(RIGHT_BACK_CHANNEL, (int)(rCap / 2));
        ledcWrite(LEFT_BACK_CHANNEL, (int)(lCap / 2));
    } else {
        ledcWrite(RIGHT_BACK_CHANNEL, rCap);
        ledcWrite(LEFT_BACK_CHANNEL, lCap);
    }
}

///////////////////////////////////////////////////////////////////////
#else

#include "BluetoothSerial.h"
#include "WiFi.h"

void setup()
{
    Serial.begin(115200);
    Serial.println("ESP32 Bluetooth MAC Address:");
    Serial.println(WiFi.macAddress()); // Works for most cases
    Serial.println("Alternate BT MAC:");
    Serial.println(WiFi.softAPmacAddress()); // Try this if needed
}

void loop()
{
    // Nothing here, just print once
}

#endif
