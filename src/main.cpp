#include <Arduino.h>
#include <ESP32Servo.h>
#include <Ps3Controller.h>

// PINDEFS
#include "car_pins"

#define PWM_FREQUENCY 5000
#define maxPwm 1023

#define DEBUG
// ----------------- GLOBALS ----------------- //

// MOTION
float valY, valW;
float rSpeed, lSpeed;
int rCap, lCap;

void drive(float argy, float argw);
void getSpeed(float valY, float valW);
void setMotion();
//

// CONTROLLER
float stickY, stickW;
bool sFlag = 0;
bool invertAxis = 0;
bool startButton = 0;

void notify();
//

// SERVO
Servo gripper, arm, halaw;

int gripperAngle, armAngle;
//

// TIMER
long debugTimer, currentMillis, pidTimer, startTimer;
//

// ------------------ SETUP ------------------ //

void setup()
{
    // Initialize the PS3 controller
    Ps3.begin("00:1a:7d:da:71:15");
    Ps3.attach(notify);

    Serial.begin(115200);

    ledcSetup(RIGHT_CHANNEL, PWM_FREQUENCY, 8);
    ledcSetup(LEFT_CHANNEL, PWM_FREQUENCY, 8);

    pinMode(RIGHT_PWM_PIN_FRONT, OUTPUT);
    pinMode(RIGHT_DIR_FRONT, OUTPUT);

    pinMode(LEFT_PWM_PIN_FRONT, OUTPUT);
    pinMode(LEFT_DIR_FRONT, OUTPUT);

    pinMode(RIGHT_PWM_PIN_BACK, OUTPUT);
    pinMode(RIGHT_DIR_BACK, OUTPUT);

    pinMode(LEFT_PWM_PIN_BACK, OUTPUT);
    pinMode(LEFT_DIR_BACK, OUTPUT);

    // Servo
    gripper.attach(19);
    arm.attach(22);
    halaw.attach(16);

    // Inbuilt LED
    pinMode(2, OUTPUT);

    analogWriteFrequency(PWM_FREQUENCY);
    analogWriteResolution(8);

    // USE CHANNELS FOR PWM (ledcWrite) NOT PINS
    ledcAttachPin(RIGHT_PWM_PIN_FRONT, RIGHT_CHANNEL);
    ledcAttachPin(RIGHT_PWM_PIN_BACK, RIGHT_CHANNEL);

    ledcAttachPin(LEFT_PWM_PIN_FRONT, LEFT_CHANNEL);
    ledcAttachPin(LEFT_PWM_PIN_BACK, LEFT_CHANNEL);

    digitalWrite(RIGHT_PWM_PIN_BACK, 0);
    digitalWrite(LEFT_PWM_PIN_BACK, 0);
    digitalWrite(RIGHT_PWM_PIN_FRONT, 0);
    digitalWrite(LEFT_PWM_PIN_FRONT, 0);

    pinMode(19, OUTPUT);
    pinMode(16, OUTPUT);
    pinMode(22, OUTPUT);

    // digitalWrite(RIGHT_DIR, 0);
    // digitalWrite(LEFT_DIR, 0);
}

// ------------------ LOOP ------------------ //

void loop()
{
    currentMillis = millis();

    // #ifdef DEBUG
    if (currentMillis - debugTimer > 1000) {
        debugTimer = currentMillis;
        // Serial.print("Y: ");
        // Serial.print(stickY);
        // Serial.print(" W: ");
        // Serial.print(stickW);
        // Serial.print(" R: ");
        // Serial.print(rSpeed);
        // Serial.print(" L: ");
        // Serial.println(lSpeed);

        Serial.printf("rCap: %d, lCap: %d\n", rCap, lCap);
        Serial.print((rSpeed > 0 ? 0 : 1));
        Serial.print('\t');
        Serial.println((lSpeed > 0 ? 0 : 1));
        Serial.printf("gripper: %d\n", gripperAngle);
    }
    // #endif
    // gripper.write(180);
    // halaw.write(180);
    // arm.write(180);

    digitalWrite(19, 0);
    digitalWrite(22, 0);
    digitalWrite(16, 0);

    delay(1000);

    // gripper.write(0);
    // halaw.write(0);
    // arm.write(0);

    digitalWrite(19, 1);
    digitalWrite(22, 1);
    digitalWrite(16, 1);
    delay(1000);

    if (Ps3.isConnected()) {
        if (startButton) {
            digitalWrite(2, HIGH);

#ifndef SERV0_TEST
            // for (int i = 0; i < 180; i++) {
            //     gripper.write(i);
            //     delay(10);
            // }
            // digitalWrite(GRIPPER_SERVO, 1);
            gripper.write(180);
            delay(1000);
            // for (int i = 180; i > 0; i--) {
            //     gripper.write(i);
            //     delay(10);
            // digitalWrite(GRIPPER_SERVO, 0);
            // }
            gripper.write(0);

            delay(1000);
#endif
            // gripper.write(gripperAngle);
            drive(stickY, stickW);

            getSpeed(valY, valW);
            setMotion();
#ifdef test
            // digitalWrite(LEFT_DIR_FRONT, 1);
            // ledcWrite(LEFT_CHANNEL, 255);

            // delay(2000);
            // digitalWrite(LEFT_DIR_FRONT, 1);
            // ledcWrite(LEFT_CHANNEL, 255);

            // delay(2000);
            // digitalWrite(RIGHT_DIR, 1);
            // digitalWrite(LEFT_DIR, 1);

            // // digitalWrite(RIGHT_PWM_PIN, 1
            // // drive(100, 0);
            // ledcWrite(RIGHT_CHANNEL, 255);
            // ledcWrite(LEFT_CHANNEL, 255);
            // analogWrite(23, stickY);
            // analogWrite(LEFT_PWM_PIN, 250);
            // analogWrite(RIGHT_PWM_PIN, 250);
#endif
        } else {
            digitalWrite(2, LOW);
            // ledcWrite(RIGHT_CHANNEL, 0);
            // ledcWrite(LEFT_CHANNEL, 0);
            digitalWrite(RIGHT_PWM_PIN_BACK, 0);
            digitalWrite(LEFT_PWM_PIN_BACK, 0);
            digitalWrite(RIGHT_PWM_PIN_FRONT, 0);
            digitalWrite(LEFT_PWM_PIN_FRONT, 0);
        }
    } else {
        digitalWrite(RIGHT_PWM_PIN_BACK, 0);
        digitalWrite(LEFT_PWM_PIN_BACK, 0);
        digitalWrite(RIGHT_PWM_PIN_FRONT, 0);
        digitalWrite(LEFT_PWM_PIN_FRONT, 0);
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

    // int tempY, tempY2;

    // JOYSTICK
    if (abs(Ps3.event.analog_changed.stick.ly) > 1) {
        stickY = Ps3.data.analog.stick.ly;
        if (stickY < 30 && stickY > -30)
            stickY = 0;
    }

    // if (abs(Ps3.event.analog_changed.button.l2) > 1) {
    //     tempY = Ps3.event.analog_changed.button.l2;
    //     map(tempY, 0, 255, 0, 128);
    // } else if (abs(Ps3.event.analog_changed.button.r2) > 1) {
    //     tempY2 = Ps3.event.analog_changed.button.r2;
    //     map(tempY2, 0, 255, 0, -128);
    // }

    // Right stick
    if (abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 2) {
        stickW = Ps3.data.analog.stick.rx;
        if (stickW < 30 && stickW > -30)
            stickW = 0;
    }
    // Select Button
    if (Ps3.event.button_down.select) {
        invertAxis = !invertAxis;
        digitalWrite(23, invertAxis);
    }

    // Shoulders for servo
    if (Ps3.event.button_down.l1) {
        gripperAngle = 90;
    }

    if (Ps3.event.button_down.r1) {
        gripperAngle = -90;
    }

    if (Ps3.event.button_down.circle) {
        gripperAngle = 0;
    }

    // stickY = tempY + tempY2;
}

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
    rSpeed = (inY) + (inW);
    lSpeed = (inY) - (inW);

    rCap = round(abs(rSpeed) * maxPwm);
    lCap = round(abs(lSpeed) * maxPwm);
}

void setMotion()
{
    digitalWrite(RIGHT_DIR_FRONT, (rSpeed > 0 ? 0 : 1));
    digitalWrite(RIGHT_DIR_BACK, (rSpeed > 0 ? 0 : 1));

    digitalWrite(LEFT_DIR_FRONT, (lSpeed > 0 ? 0 : 1));
    digitalWrite(LEFT_DIR_BACK, (lSpeed > 0 ? 0 : 1));

    ledcWrite(RIGHT_CHANNEL, rCap);
    ledcWrite(LEFT_CHANNEL, lCap);
}
