#include <Servo.h>

#define PAN_SERVO                          6
#define TILT_SERVO                         7

int TiltServoPosition  = 90;
int PanServoPosition   = 90;

Servo PanServo;
Servo TiltServo;

char cmd = ' ';

void setup() {
    Serial.begin(9600);           // set up Serial library at 9600 bps
    Serial.println("Starting Arduino Mega ------------...................");
}

void camera_up()
{
    if (TiltServoPosition > 0)
    {
        TiltServoPosition -= 10;
        TiltServo.attach(TILT_SERVO);
        TiltServo.write(TiltServoPosition);
        delay(500);
        TiltServo.detach();
    }
}

void camera_down()
{
    if (TiltServoPosition < 180)
    {
        TiltServoPosition += 10;
        TiltServo.attach(TILT_SERVO);
        TiltServo.write(TiltServoPosition);
        delay(500);
        TiltServo.detach();
    }
}

void camera_left()
{
    if (PanServoPosition < 180)
    {
        PanServoPosition += 10;
        PanServo.attach(PAN_SERVO);
        PanServo.write(PanServoPosition);
        delay(500);
        PanServo.detach();
    }
}

void camera_right()
{
    if (PanServoPosition > 0)
    {
        PanServoPosition -= 10;
        PanServo.attach(PAN_SERVO);
        PanServo.write(PanServoPosition);
        delay(500);
        PanServo.detach();
    }
}

void loop() { 
    if (Serial.available())
    {
        cmd = Serial.read();
    }
    else
    {
        cmd = ' ';
    }

    if (cmd == 'U')
    {
        camera_up();
    }
    else if (cmd == 'D')
    {
        camera_down();
    }
    else if (cmd == 'R')
    {
        camera_right();
    }
    else if (cmd == 'L')
    {
        camera_left();
    }
}
