#include <Servo.h>

#define PAN_SERVO                          13
#define TILT_SERVO                         12

int TiltServoPosition  = 90;
int PanServoPosition   = 90;

Servo PanServo;
Servo TiltServo;

char cmd = ' ';
char temp_cmd = ' ';
char *ACCEPTED_INPUTS = "UDLRS";

boolean bTiltServoAttached = false;
boolean bPanServoAttached = false;

int count = 0;

void setup() {
    Serial.begin(9600);           // set up Serial library at 9600 bps
    Serial.println("Starting Arduino Mega ------------...................");
    
    TiltServo.attach(TILT_SERVO);
    PanServo.attach(PAN_SERVO);
    delay(500);
    TiltServo.write(TiltServoPosition);
    PanServo.write(PanServoPosition);
    delay(500);
    TiltServo.detach();
    PanServo.detach();
}

void attach_tilt_servo()
{
    TiltServo.attach(TILT_SERVO);
    bTiltServoAttached = true;
}

void detach_tilt_servo()
{
    TiltServo.detach();
    bTiltServoAttached = false;
}

void attach_pan_servo()
{
    PanServo.attach(PAN_SERVO);
    bPanServoAttached = true;
}

void detach_pan_servo()
{
    PanServo.detach();
    bPanServoAttached = false;
}

void camera_up()
{
    if (TiltServoPosition > 0)
    {
        TiltServoPosition -= 1;
        TiltServo.write(TiltServoPosition);
        delay(50);
    }
}

void camera_down()
{
    if (TiltServoPosition < 180)
    {
        TiltServoPosition += 1;
        TiltServo.write(TiltServoPosition);
        delay(50);
    }
}

void camera_left()
{
    if (PanServoPosition < 180)
    {
        PanServoPosition += 1;
        PanServo.write(PanServoPosition);
        delay(50);
    }
}

void camera_right()
{
    if (PanServoPosition > 0)
    {
        PanServoPosition -= 1;
        PanServo.write(PanServoPosition);
        delay(50);
    }
}

void loop() { 
    count++;
    
    while (Serial.available())
    {
        temp_cmd = Serial.read();
        if (strchr(ACCEPTED_INPUTS, temp_cmd))
        {
            count = 0;
            cmd = temp_cmd;
        }
    }

    if ((cmd == 'U') || (cmd == 'D'))
    {
        if(!bTiltServoAttached)
            attach_tilt_servo();
        if(bPanServoAttached)
            detach_pan_servo();
    }
    else if ((cmd == 'L') || (cmd == 'R'))
    {
        if(!bPanServoAttached)
            attach_pan_servo();
        if(bTiltServoAttached)
            detach_tilt_servo();
    }
    else if ((cmd == 'S') || (count>100))
    {
        if(bTiltServoAttached)
            detach_tilt_servo();
        if(bPanServoAttached)
            detach_pan_servo();
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
