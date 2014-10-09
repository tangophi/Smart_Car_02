#include <NewPing.h>
#include <Wire.h>
#include <UTFT.h>
#include <Servo.h>

// Declare which fonts we will be using
extern uint8_t SmallFont[];

// Set the pins to the correct ones for your development shield
// ------------------------------------------------------------
// Standard Arduino Mega/Due shield            : <display model>,38,39,40,41
// CTE TFT LCD/SD Shield for Arduino Due       : <display model>,25,26,27,28
// Teensy 3.x TFT Test Board                   : <display model>,23,22, 3, 4
// ElecHouse TFT LCD/SD Shield for Arduino Due : <display model>,22,23,31,33
//
// Remember to change the model parameter to suit your display module!
UTFT myGLCD(ITDB32S,38,39,40,41);


#define MAX_DISTANCE 500 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).


#define FRONT_ULTRASONIC_SENSOR_TRIGGER    45
#define FRONT_ULTRASONIC_SENSOR_ECHO       47
#define BACK_ULTRASONIC_SENSOR_TRIGGER     51
#define BACK_ULTRASONIC_SENSOR_ECHO        53

unsigned int safe_distance = 20;

#define FRONT_OBSTACLE_AVOIDANCE_SENSOR    43
#define BACK_OBSTACLE_AVOIDANCE_SENSOR     49

#define FRONT_SERVO                        50
#define BACK_SERVO                         52

#define PAN_SERVO                          48
#define TILT_SERVO                         46

int FrontServoPosition = 0;
int BackServoPosition  = 0;
int TiltServoPosition  = 90;
int PanServoPosition   = 90;

NewPing FrontSonar(FRONT_ULTRASONIC_SENSOR_TRIGGER, FRONT_ULTRASONIC_SENSOR_ECHO, MAX_DISTANCE); 
NewPing BackSonar(BACK_ULTRASONIC_SENSOR_TRIGGER, BACK_ULTRASONIC_SENSOR_ECHO, MAX_DISTANCE); 

Servo FrontServo;
Servo BackServo;
Servo PanServo;
Servo TiltServo;

char cmd = ' ';
char prev_cmd = ' ';
int i=0,j=90,k=180,l=0;
char output_string[256];
char *str;
boolean bObstacle = false, bSonar = false;
unsigned int uS;
unsigned int distance;
int count = 0;

boolean bInForwardMotion = false;
boolean bInForwardLeftMotion = false;
boolean bInForwardRightMotion = false;
boolean bInReverseMotion = false;
boolean bInReverseLeftMotion = false;
boolean bInReverseRightMotion = false;

void setup() {
    Serial.begin(9600);           // set up Serial library at 9600 bps
    Serial3.begin(9600);
    Wire.begin();
    
    Serial.println("Starting Arduino Mega ------------...................");
    
    pinMode(FRONT_ULTRASONIC_SENSOR_TRIGGER, OUTPUT);
    pinMode(FRONT_ULTRASONIC_SENSOR_ECHO,    INPUT);
    pinMode(BACK_ULTRASONIC_SENSOR_TRIGGER,  OUTPUT);
    pinMode(BACK_ULTRASONIC_SENSOR_ECHO,     INPUT);

    pinMode(FRONT_OBSTACLE_AVOIDANCE_SENSOR, INPUT);
    pinMode(BACK_OBSTACLE_AVOIDANCE_SENSOR,  INPUT);

    myGLCD.InitLCD();
    myGLCD.setFont(SmallFont);
    myGLCD.clrScr();
}

void serial_print (char *direction, int secs, int cms, boolean bS, boolean bO)
{
    sprintf(output_string, "Sonar: %s - %d cms - %d us", direction, secs, cms);

    if (bS)
    {
        myGLCD.setColor(0, 255, 0);
    }
    else
    {
        myGLCD.setColor(255, 0, 0);
    }

//    Serial.println(output_string);
    myGLCD.print(output_string, CENTER, 100);

    if (bO)
    {
        myGLCD.setColor(0, 255, 0);
        sprintf(output_string, "IR: No obstacle in %s", direction);
    }
    else
    {
        myGLCD.setColor(255, 0, 0);
        sprintf(output_string, "IR: Obstacle in %s", direction);
    }

//    Serial.println(output_string);
    myGLCD.print(output_string, CENTER, 150);
   
}

boolean check_forward() {
    if (FrontServoPosition != 90)
    {
        FrontServo.attach(FRONT_SERVO);
        FrontServo.write(90);
        delay(500);
        FrontServo.detach();
        FrontServoPosition = 90;
    }

    /*
    ** Check Sonar only if the car is moving
    */   
    if ((prev_cmd == 'F') && bInForwardMotion)
    {
        /* Send ping, get ping time in microseconds (uS) */
        uS = FrontSonar.ping(); 
        distance = uS / US_ROUNDTRIP_CM;
    
        if (distance >= safe_distance)
        {
            bSonar = true;
        }
        else
        { 
            bSonar = false;
        }
    }
    else
    {
        bSonar = true;
    }

    if(digitalRead(FRONT_OBSTACLE_AVOIDANCE_SENSOR))
    {
        /* No obstacle detected */
        bObstacle = true; 
    }
    else
    {
        /* Obstacle detected */
        bObstacle = false;
    }

    myGLCD.clrScr();
//    serial_print("forward", distance, uS, bSonar, bObstacle);

    /*
    ** Return true only if both sonar and IR sensor dont detect
    ** any obstacles.  In case the car was stopped by a previous
    ** check, then only bObstacle matters, because bSonar will be 
    ** set to true above.
    */
    if (bObstacle && bSonar)
    {
        myGLCD.setColor(0, 255, 0);
        myGLCD.fillRect(149, 0, 169, 19);
        return true;
    }
    else
    {
        myGLCD.setColor(255, 0, 0);
        myGLCD.fillRect(149, 0, 169, 19);
        return false;
    }
}

boolean check_forward_left() {
    if (FrontServoPosition != 180)
    {
        FrontServo.attach(FRONT_SERVO);
        FrontServo.write(180);
        delay(500);
        FrontServo.detach();
        FrontServoPosition = 180;
    }

    /*
    ** Check Sonar only if the car is moving
    */   
    if ((prev_cmd == 'G') && bInForwardLeftMotion)
    {
        /* Send ping, get ping time in microseconds (uS) */
        uS = FrontSonar.ping(); 
        distance = uS / US_ROUNDTRIP_CM;
    
        if (distance >= safe_distance)
        {
            bSonar = true;
        }
        else
        { 
            bSonar = false;
        }
    }
    else
    {
        bSonar = true;
    }

    if(digitalRead(FRONT_OBSTACLE_AVOIDANCE_SENSOR))
    {
        /* No obstacle detected */
        bObstacle = true; 
    }
    else
    {
        /* Obstacle detected */
        bObstacle = false;
    }

    myGLCD.clrScr();
//    serial_print("forward_left", distance, uS, bSonar, bObstacle);

    /*
    ** Return true only if both sonar and IR sensor dont detect
    ** any obstacles.  In case the car was stopped by a previous
    ** check, then only bObstacle matters, because bSonar will be 
    ** set to true above.
    */
    if (bObstacle && bSonar)
    {
        myGLCD.setColor(0, 255, 0);
        myGLCD.fillRect(0, 0, 19, 19);
        return true;
    }
    else
    {
        myGLCD.setColor(255, 0, 0);
        myGLCD.fillRect(0, 0, 19, 19);
        return false;
    }
}

boolean check_forward_right() {
    if (FrontServoPosition != 30)
    {
        FrontServo.attach(FRONT_SERVO);
        FrontServo.write(30);
        delay(500);
        FrontServo.detach();
        FrontServoPosition = 30;
    }

    /*
    ** Check Sonar only if the car is moving
    */   
    if ((prev_cmd == 'I') && bInForwardRightMotion)
    {
        /* Send ping, get ping time in microseconds (uS) */
        uS = FrontSonar.ping(); 
        distance = uS / US_ROUNDTRIP_CM;
    
        if (distance >= safe_distance)
        {
            bSonar = true;
        }
        else
        { 
            bSonar = false;
        }
    }
    else
    {
        bSonar = true;
    }

    if(digitalRead(FRONT_OBSTACLE_AVOIDANCE_SENSOR))
    {
        /* No obstacle detected */
        bObstacle = true; 
    }
    else
    {
        /* Obstacle detected */
        bObstacle = false;
    }

    myGLCD.clrScr();
//    serial_print("forward_right", distance, uS, bSonar, bObstacle);

    /*
    ** Return true only if both sonar and IR sensor dont detect
    ** any obstacles.  In case the car was stopped by a previous
    ** check, then only bObstacle matters, because bSonar will be 
    ** set to true above.
    */
    if (bObstacle && bSonar)
    {
        myGLCD.setColor(0, 255, 0);
        myGLCD.fillRect(299, 0, 319, 19);
        return true;
    }
    else
    {
        myGLCD.setColor(255, 0, 0);
        myGLCD.fillRect(299, 0, 319, 19);
        return false;
    }
}

boolean check_reverse() {
    if (BackServoPosition != 90)
    {
        BackServo.attach(BACK_SERVO);
        BackServo.write(90);
        delay(500);
        BackServo.detach();
        BackServoPosition = 90;
    }

    /*
    ** Check Sonar only if the car is moving
    */   
    if ((prev_cmd == 'B') && bInReverseMotion)
    {
        /* Send ping, get ping time in microseconds (uS) */
        uS = BackSonar.ping(); 
        distance = uS / US_ROUNDTRIP_CM;
    
        if (distance >= safe_distance)
        {
            bSonar = true;
        }
        else
        { 
            bSonar = false;
        }
    }
    else
    {
        bSonar = true;
    }

    if(digitalRead(BACK_OBSTACLE_AVOIDANCE_SENSOR))
    {
        /* No obstacle detected */
        bObstacle = true; 
    }
    else
    {
        /* Obstacle detected */
        bObstacle = false;
    }

    myGLCD.clrScr();
//    serial_print("reverse", distance, uS, bSonar, bObstacle);

    /*
    ** Return true only if both sonar and IR sensor dont detect
    ** any obstacles.  In case the car was stopped by a previous
    ** check, then only bObstacle matters, because bSonar will be 
    ** set to true above.
    */
    if (bObstacle && bSonar)
    {
        myGLCD.setColor(0, 255, 0);
        myGLCD.fillRect(149, 219, 169, 239);
        return true;
    }
    else
    {
        myGLCD.setColor(255, 0, 0);
        myGLCD.fillRect(149, 219, 169, 239);
        return false;
    }
}

boolean check_reverse_left() {
    if (BackServoPosition != 20)
    {
        BackServo.attach(BACK_SERVO);
        BackServo.write(20);
        delay(500);
        BackServo.detach();
        BackServoPosition = 20;
    }

    /*
    ** Check Sonar only if the car is moving
    */   
    if ((prev_cmd == 'H') && bInReverseLeftMotion)
    {
        /* Send ping, get ping time in microseconds (uS) */
        uS = BackSonar.ping(); 
        distance = uS / US_ROUNDTRIP_CM;
    
        if (distance >= safe_distance)
        {
            bSonar = true;
        }
        else
        { 
            bSonar = false;
        }
    }
    else
    {
        bSonar = true;
    }

    if(digitalRead(BACK_OBSTACLE_AVOIDANCE_SENSOR))
    {
        /* No obstacle detected */
        bObstacle = true; 
    }
    else
    {
        /* Obstacle detected */
        bObstacle = false;
    }

    myGLCD.clrScr();
//    serial_print("reverse_left", distance, uS, bSonar, bObstacle);

    /*
    ** Return true only if both sonar and IR sensor dont detect
    ** any obstacles.  In case the car was stopped by a previous
    ** check, then only bObstacle matters, because bSonar will be 
    ** set to true above.
    */
    if (bObstacle && bSonar)
    {
        myGLCD.setColor(0, 255, 0);
        myGLCD.fillRect(0, 219, 19, 239);
        return true;
    }
    else
    {
        myGLCD.setColor(255, 0, 0);
        myGLCD.fillRect(0, 219, 19, 239);
        return false;
    }
}

boolean check_reverse_right() {
    if (BackServoPosition != 180)
    {
        BackServo.attach(BACK_SERVO);
        BackServo.write(180);
        delay(500);
        BackServo.detach();
        BackServoPosition = 180;
    }

    /*
    ** Check Sonar only if the car is moving
    */   
    if ((prev_cmd == 'J') && bInReverseRightMotion)
    {
        /* Send ping, get ping time in microseconds (uS) */
        uS = BackSonar.ping(); 
        distance = uS / US_ROUNDTRIP_CM;
    
        if (distance >= safe_distance)
        {
            bSonar = true;
        }
        else
        { 
            bSonar = false;
        }
    }
    else
    {
        bSonar = true;
    }

    if(digitalRead(BACK_OBSTACLE_AVOIDANCE_SENSOR))
    {
        /* No obstacle detected */
        bObstacle = true; 
    }
    else
    {
        /* Obstacle detected */
        bObstacle = false;
    }

    myGLCD.clrScr();
//    serial_print("reverse_right", distance, uS, bSonar, bObstacle);

    /*
    ** Return true only if both sonar and IR sensor dont detect
    ** any obstacles.  In case the car was stopped by a previous
    ** check, then only bObstacle matters, because bSonar will be 
    ** set to true above.
    */
    if (bObstacle && bSonar)
    {
        myGLCD.setColor(0, 255, 0);
        myGLCD.fillRect(299, 219, 319, 239);
        return true;
    }
    else
    {
        myGLCD.setColor(255, 0, 0);
        myGLCD.fillRect(299, 219, 319, 239);
        return false;
    }
}

void wsend(char c)
{
    Wire.beginTransmission(9);
    Wire.write(c);
    Wire.endTransmission();
}

void stop()
{
    wsend('S');
}

void forward()
{
    if (check_forward())
    {
        bInForwardMotion = true;
        wsend('F');
    }
    else
    {
        if (bInForwardMotion)
        {
            stop();
        }
        bInForwardMotion = false;
    }
}

void forward_left()
{
    if (check_forward_left())
    {
        bInForwardLeftMotion = true;
        wsend('G');
    }
    else
    {
        if (bInForwardLeftMotion)
        {
            stop();
        }
        bInForwardRightMotion = false;
    }
}

void forward_right()
{
    if (check_forward_right())
    {
        bInForwardRightMotion = true;
        wsend('I');
    }
    else
    {
        if (bInForwardRightMotion)
        {
            stop();
        }
        bInForwardRightMotion = false;
    }
}

void reverse()
{
    if (check_reverse())
    {
        bInReverseMotion = true;
        wsend('B');
    }
    else
    {
        if (bInReverseMotion)
        {
            stop();
        }
        bInReverseMotion = false;
    }
}

void reverse_left()
{
    if (check_reverse_left())
    {
        bInReverseLeftMotion = true;
        wsend('H');
    }
    else
    {
        if (bInReverseLeftMotion)
        {
            stop();
        }
        bInReverseRightMotion = false;
    }
}

void reverse_right()
{
    if (check_reverse_right())
    {
        bInReverseRightMotion = true;
        wsend('J');
    }
    else
    {
        if (bInReverseRightMotion)
        {
            stop();
        }
        bInReverseRightMotion = false;
    }
}

void camera_up()
{
    if (TiltServoPosition > 40)
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
    count ++;
    
    /* 
    ** Clear LCD screen if no activity for some time.  Otherwise
    ** there will be a "screen burn in at the place on the screen
    ** of the last car movement.
    */
    if (count > 10000)
    {
        count = 0;
        myGLCD.clrScr();
    }
        
    if (Serial3.available())
    {
        cmd = Serial3.read();
    }
    else if (Serial.available())
    {
        cmd = Serial.read();
    }
    else
    {
        cmd = ' ';
    }

    if (prev_cmd == 'F') {
        if (!check_forward())
        {
            stop();
        }
    } else if (prev_cmd == 'B') {
        if (!check_reverse())
        {
            stop();
        }
    } else if (prev_cmd == 'G') {
        if (!check_forward_left())
        {
            stop();
        }
    } else if (prev_cmd == 'I') {
        if (!check_forward_right())
        {
            stop();
        }
    } else if (prev_cmd == 'H') {
        if (!check_reverse_left())
        {
            stop();
        }
    } else if (prev_cmd == 'J') {
        if (!check_reverse_right())
        {
            stop();
        }
    }
    
    if (cmd == 'F')
    {
        forward();
        prev_cmd = cmd;
        count = 0;
    }
    else if (cmd == 'B')
    {
        reverse();
        prev_cmd = cmd;
        count = 0;
    }
    else if (cmd == 'G')
    {
        forward_left();
        prev_cmd = cmd;
        count = 0;
    }
    else if (cmd == 'I')
    {
        forward_right();
        prev_cmd = cmd;
        count = 0;
    }
    else if (cmd == 'H')
    {
        reverse_left();
        prev_cmd = cmd;
        count = 0;
    }
    else if (cmd == 'J')
    {
        reverse_right();
        prev_cmd = cmd;
        count = 0;
    }
    else if (cmd == 'S')
    {
        stop();
        prev_cmd = cmd;            
        count = 0;
    }
    else if (cmd == 'U')
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
