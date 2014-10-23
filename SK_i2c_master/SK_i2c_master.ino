#include <NewPing.h>
#include <Wire.h>
#include <UTFT.h>
#include <Servo.h>
#include "FastLED.h"
#include "TimerOne.h"
#include "TimerThree.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// How many leds in your strip?
#define NUM_LEDS 44
#define LED_DATA_PIN 13
// Define the array of leds
CRGB leds[NUM_LEDS];

// Declare which fonts we will be using
extern uint8_t SmallFont[];
extern uint8_t BigFont[];
extern uint8_t SevenSegNumFont[];

// Set the pins to the correct ones for your development shield
// ------------------------------------------------------------
// Standard Arduino Mega/Due shield            : <display model>,38,39,40,41
// CTE TFT LCD/SD Shield for Arduino Due       : <display model>,25,26,27,28
// Teensy 3.x TFT Test Board                   : <display model>,23,22, 3, 4
// ElecHouse TFT LCD/SD Shield for Arduino Due : <display model>,22,23,31,33
//
// Remember to change the model parameter to suit your display module!
UTFT myGLCD(ITDB32S,38,39,40,41);


#define PIN_BUZZER       11
#define PIN_TILT_SWITCHPIN_TILT_SWITCH  12

#define PIN_TEMPERATURE       1  // Analog
#define PIN_BATTERY_VOLTAGE   7  // Analog


float resistor1 = 993;
float resistor2 = 324.3;

#define MAX_DISTANCE 500 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).


#define FRONT_ULTRASONIC_SENSOR_TRIGGER    45
#define FRONT_ULTRASONIC_SENSOR_ECHO       47
#define BACK_ULTRASONIC_SENSOR_TRIGGER     51
#define BACK_ULTRASONIC_SENSOR_ECHO        53

unsigned int safe_distance = 30;

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

volatile int servoPos = 0;
int ForwardServoPos[] = { 60, 90, 120, 90 };
int ForwardLeftServoPos[] = { 180, 150, 120, 150 };
int ForwardRightServoPos[] = { 20, 50, 80, 50 };
int BackwardServoPos[] = { 60, 90, 120, 90 };
int BackwardLeftServoPos[] = { 20, 50, 80, 50 };
int BackwardRightServoPos[] = { 180, 150, 120, 150 };
volatile boolean bFrontSensorServoAttached = false;
volatile boolean bBackSensorServoAttached = false;

NewPing FrontSonar(FRONT_ULTRASONIC_SENSOR_TRIGGER, FRONT_ULTRASONIC_SENSOR_ECHO, MAX_DISTANCE); 
NewPing BackSonar(BACK_ULTRASONIC_SENSOR_TRIGGER, BACK_ULTRASONIC_SENSOR_ECHO, MAX_DISTANCE); 

Servo FrontServo;
Servo BackServo;
Servo PanServo;
Servo TiltServo;

char *ACCEPTED_INPUTS = "FBGIHJSDURL";
volatile char cmd = ' ';
volatile char temp_cmd = ' ';
char prev_cmd = ' ';
volatile char led_cmd = 'A';
int i=0,j=90,k=180,l=0;
char output_string[256];
char debug_strings[10][256] = {' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};
char null_string[256] = "                                                                                                                             ";
char *str;
boolean bObstacle = false, bSonar = false;
unsigned int uS;
unsigned int distance;
volatile int count = 0;

volatile boolean bForwardCheck = false;
volatile boolean bForwardLeftCheck = false;
volatile boolean bForwardRightCheck = false;
volatile boolean bReverseCheck = false;
volatile boolean bReverseLeftCheck = false;
volatile boolean bReverseRightCheck = false;

volatile boolean bForwardCheckNow = false;
volatile boolean bForwardLeftCheckNow = false;
volatile boolean bForwardRightCheckNow = false;
volatile boolean bReverseCheckNow = false;
volatile boolean bReverseLeftCheckNow = false;
volatile boolean bReverseRightCheckNow = false;

volatile boolean bForwardCheckLast = false;
volatile boolean bForwardLeftCheckLast = false;
volatile boolean bForwardRightCheckLast = false;
volatile boolean bReverseCheckLast = false;
volatile boolean bReverseLeftCheckLast = false;
volatile boolean bReverseRightCheckLast = false;

volatile int ledPos = 0;
int LeftLEDs[] = { 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1000, 1000 };
int RightLEDs[] = { 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 1000, 1000 };
int FrontLEDs[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 1000, 1000 };
int BackLEDs[] = { 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 1000, 1000 };

CRGB::HTMLColorCode Color1 = CRGB::Black;
CRGB::HTMLColorCode Color2 = CRGB::Navy;
CRGB::HTMLColorCode Color3 = CRGB::Black;
CRGB::HTMLColorCode Color4 = CRGB::IndianRed;
CRGB::HTMLColorCode Color5 = CRGB::LightSkyBlue;
CRGB::HTMLColorCode Color6 = CRGB::IndianRed;

volatile int led_color1 = 0;
volatile int led_color2 = 70;
volatile int led_counter1 = 0;
volatile int led_counter2 = 0;
int idle_start = 100;
int idle_end = 200;

int htmlcolorcode[] = {
0xF0F8FF,
0x9966CC,
0xFAEBD7,
0x00FFFF,
0x7FFFD4,
0xF0FFFF,
0xF5F5DC,
0xFFE4C4,
0x000000,
0xFFEBCD,
0x0000FF,
0x8A2BE2,
0xA52A2A,
0xDEB887,
0x5F9EA0,
0x7FFF00,
0xD2691E,
0xFF7F50,
0x6495ED,
0xFFF8DC,
0xDC143C,
0x00FFFF,
0x00008B,
0x008B8B,
0xB8860B,
0xA9A9A9,
0x006400,
0xBDB76B,
0x8B008B,
0x556B2F,
0xFF8C00,
0x9932CC,
0x8B0000,
0xE9967A,
0x8FBC8F,
0x483D8B,
0x2F4F4F,
0x00CED1,
0x9400D3,
0xFF1493,
0x00BFFF,
0x696969,
0x1E90FF,
0xB22222,
0xFFFAF0,
0x228B22,
0xFF00FF,
0xDCDCDC,
0xF8F8FF,
0xFFD700,
0xDAA520,
0x808080,
0x008000,
0xADFF2F,
0xF0FFF0,
0xFF69B4,
0xCD5C5C,
0x4B0082,
0xFFFFF0,
0xF0E68C,
0xE6E6FA,
0xFFF0F5,
0x7CFC00,
0xFFFACD,
0xADD8E6,
0xF08080,
0xE0FFFF,
0xFAFAD2,
0x90EE90,
0xD3D3D3,
0xFFB6C1,
0xFFA07A,
0x20B2AA,
0x87CEFA,
0x778899,
0xB0C4DE,
0xFFFFE0,
0x00FF00,
0x32CD32,
0xFAF0E6,
0xFF00FF,
0x800000,
0x66CDAA,
0x0000CD,
0xBA55D3,
0x9370DB,
0x3CB371,
0x7B68EE,
0x00FA9A,
0x48D1CC,
0xC71585,
0x191970,
0xF5FFFA,
0xFFE4E1,
0xFFE4B5,
0xFFDEAD,
0x000080,
0xFDF5E6,
0x808000,
0x6B8E23,
0xFFA500,
0xFF4500,
0xDA70D6,
0xEEE8AA,
0x98FB98,
0xAFEEEE,
0xDB7093,
0xFFEFD5,
0xFFDAB9,
0xCD853F,
0xFFC0CB,
0xCC5533,
0xDDA0DD,
0xB0E0E6,
0x800080,
0xFF0000,
0xBC8F8F,
0x4169E1,
0x8B4513,
0xFA8072,
0xF4A460,
0x2E8B57,
0xFFF5EE,
0xA0522D,
0xC0C0C0,
0x87CEEB,
0x6A5ACD,
0x708090,
0xFFFAFA,
0x00FF7F,
0x4682B4,
0xD2B48C,
0x008080,
0xD8BFD8,
0xFF6347,
0x40E0D0,
0xEE82EE,
0xF5DEB3,
0xFFFFFF,
0xF5F5F5,
0xFFFF00,
0x9ACD32
    } ;

volatile boolean DEBUG = false;

void serial_print (char *direction, int secs, int cms, boolean bS, boolean bO)
{
    if (DEBUG)
    {
        myGLCD.setFont(SmallFont);

        sprintf(output_string, "Sonar: %s - %d cms - %d us", direction, secs, cms);

        if (bS)
        {
            myGLCD.setColor(0, 255, 0);
        }
        else
        {
            myGLCD.setColor(255, 0, 0);
        }

        Serial.println(output_string);
        myGLCD.print(output_string, CENTER, 60);

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

        Serial.println(output_string);
        myGLCD.print(output_string, CENTER, 70);
    }
}

void serial_println (char *mesg, char c)
{
    if (DEBUG) {
        sprintf(output_string, "%s : %c", mesg, c);
        Serial.println(output_string);
        strcpy(debug_strings[0], debug_strings[1]);
        strcpy(debug_strings[1], debug_strings[2]);
        strcpy(debug_strings[2], debug_strings[3]);
        strcpy(debug_strings[3], debug_strings[4]);
        strcpy(debug_strings[4], debug_strings[5]);
        strcpy(debug_strings[5], debug_strings[6]);
        strcpy(debug_strings[6], debug_strings[7]);
        strcpy(debug_strings[7], debug_strings[8]);
        strcpy(debug_strings[8], debug_strings[9]);
        strcpy(debug_strings[9], output_string);
        
        myGLCD.setColor(255, 255, 255);
        myGLCD.setFont(SmallFont);
        myGLCD.print(debug_strings[0], LEFT, 80);
        myGLCD.print(debug_strings[1], LEFT, 90);
        myGLCD.print(debug_strings[2], LEFT, 100);
        myGLCD.print(debug_strings[3], LEFT, 110);
        myGLCD.print(debug_strings[4], LEFT, 120);
        myGLCD.print(debug_strings[5], LEFT, 130);
        myGLCD.print(debug_strings[6], LEFT, 140);
        myGLCD.print(debug_strings[7], LEFT, 150);
        myGLCD.print(debug_strings[8], LEFT, 160);
        myGLCD.print(debug_strings[9], LEFT, 170);
    }
}

void setup() {
    sensor_t sensor;
    char lcd_str[256];
    float tempC = 0, battery_voltage = 0;
    int voltagePinReading = 0;

    Serial.begin(57600);           // USB connection to Raspberry Pi
    Serial3.begin(9600);          // HC-05 Bluetooth module
    Wire.begin();
    
    Serial.println("Starting Arduino Mega ...................");

    NewPing::timer_ms(100, ledStrip);
//    NewPing::timer_ms(100, checkSensors);

//    analogReference(INTERNAL1V1);

//    Timer3.initialize(100000);
//    Timer3.attachInterrupt(ledStrip);

//    Timer3.initialize(10000);
//    Timer3.attachInterrupt(getInput);
    
    pinMode(FRONT_ULTRASONIC_SENSOR_TRIGGER, OUTPUT);
    pinMode(FRONT_ULTRASONIC_SENSOR_ECHO,    INPUT);
    pinMode(BACK_ULTRASONIC_SENSOR_TRIGGER,  OUTPUT);
    pinMode(BACK_ULTRASONIC_SENSOR_ECHO,     INPUT);

    pinMode(FRONT_OBSTACLE_AVOIDANCE_SENSOR, INPUT);
    pinMode(BACK_OBSTACLE_AVOIDANCE_SENSOR,  INPUT);

    myGLCD.InitLCD();
    myGLCD.setFont(SmallFont);
    myGLCD.clrScr();
    
    FastLED.addLeds<NEOPIXEL, LED_DATA_PIN, RGB>(leds, NUM_LEDS);
 
    /* Initialise the sensor */
    if(!mag.begin())
    {
        /* There was a problem detecting the HMC5883 ... check your connections */
        Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    }    


                
    Serial.print  ("Mag Sensor:   "); Serial.println(sensor.name);
    Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
    Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
    Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT"); 

    // Get a new sensor event
    sensors_event_t event; 
    mag.getEvent(&event);
 
    // Display the results (magnetic vector values are in micro-Tesla (uT))
    Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

    // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    float heading = atan2(event.magnetic.y, event.magnetic.x);
  
    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
    // Bangalore is : 1* 33' W, which is 0.1745
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
    float declinationAngle = 0.1745;
    heading += declinationAngle;
  
    // Correct for when signs are reversed.
    if(heading < 0)
        heading += 2*PI;
    
    // Check for wrap due to addition of declination.
    if(heading > 2*PI)
        heading -= 2*PI;
   
    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180/M_PI; 
  
    Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  
  
    analogReference(INTERNAL1V1);
    tempC = analogRead(PIN_TEMPERATURE)/9.31;
    Serial.print("Temperature: "); Serial.println(tempC);

    sprintf(lcd_str, "Temperature: %.1f", tempC);
    myGLCD.print(lcd_str, LEFT, 130);
    sprintf(lcd_str, "Heading: %.1f", headingDegrees);
    myGLCD.print(lcd_str, LEFT, 160);

    analogReference(DEFAULT);
    
    voltagePinReading = analogRead(PIN_BATTERY_VOLTAGE);
    battery_voltage = ((float)voltagePinReading) * (5.0 /1024.0) * (resistor1 + resistor2) / resistor2;

    Serial.print("voltagePinReading: "); Serial.println(voltagePinReading);
    
    Serial.print("Battery voltage: "); Serial.println(battery_voltage);
    sprintf(lcd_str, "Battery voltage: %.1f", battery_voltage);
    myGLCD.print(lcd_str, LEFT, 180);
    
    delay(2000);

}

void ledStrip(void)
{
    int p;
    int color1,color2;

    count++;
    
    if (cmd == 'F' ) {
        if (bForwardCheck) {
            color1 = Color1;
            color2 = Color2;
        } else {
            color1 = Color3;
            color2 = Color4;
        }
      
        for (p=0;p<22;p=p+4) {
            leds[LeftLEDs[p]]    = (ledPos == 0)?color2:color1;
            leds[LeftLEDs[p+1]]  = (ledPos == 1)?color2:color1;
            leds[LeftLEDs[p+2]]  = (ledPos == 2)?color2:color1;
            leds[LeftLEDs[p+3]]  = (ledPos == 3)?color2:color1;
            leds[RightLEDs[p]]   = (ledPos == 0)?color2:color1;
            leds[RightLEDs[p+1]] = (ledPos == 1)?color2:color1;
            leds[RightLEDs[p+2]] = (ledPos == 2)?color2:color1;
            leds[RightLEDs[p+3]] = (ledPos == 3)?color2:color1;
        }
    } else if (cmd == 'B' ) {
        if (bReverseCheck) {
            color1 = Color1;
            color2 = Color2;
        } else {
            color1 = Color3;
            color2 = Color4;
        }
      
        for (p=21;p>0;p=p-4) {
            leds[LeftLEDs[p]]    = (ledPos == 0)?color2:color1;
            leds[LeftLEDs[p-1]]  = (ledPos == 1)?color2:color1;
            leds[LeftLEDs[p-2]]  = (ledPos == 2)?color2:color1;
            leds[LeftLEDs[p-3]]  = (ledPos == 3)?color2:color1;
            leds[RightLEDs[p]]   = (ledPos == 0)?color2:color1;
            leds[RightLEDs[p-1]] = (ledPos == 1)?color2:color1;
            leds[RightLEDs[p-2]] = (ledPos == 2)?color2:color1;
            leds[RightLEDs[p-3]] = (ledPos == 3)?color2:color1;
        }
    } else if (cmd == 'G' ) {
        if (bForwardLeftCheck) {
            color1 = Color1;
            color2 = Color2;
        } else {
            color1 = Color3;
            color2 = Color4;
        }
      
        for (p=21;p>0;p=p-4) {
            leds[FrontLEDs[p]]  =  (ledPos == 0)?color2:color1;
            leds[FrontLEDs[p-1]] = (ledPos == 1)?color2:color1;
            leds[FrontLEDs[p-2]] = (ledPos == 2)?color2:color1;
            leds[FrontLEDs[p-3]] = (ledPos == 3)?color2:color1;
            leds[BackLEDs[p]]  =  Color1;
            leds[BackLEDs[p-1]] = Color1;
            leds[BackLEDs[p-2]] = Color1;
            leds[BackLEDs[p-3]] = Color1;
        }
    } else if (cmd == 'I' ) {
        if (bForwardRightCheck) {
            color1 = Color1;
            color2 = Color2;
        } else {
            color1 = Color3;
            color2 = Color4;
        }
      
        for (p=0;p<22;p=p+4) {
            leds[FrontLEDs[p]]   = (ledPos == 0)?color2:color1;
            leds[FrontLEDs[p+1]] = (ledPos == 1)?color2:color1;
            leds[FrontLEDs[p+2]] = (ledPos == 2)?color2:color1;
            leds[FrontLEDs[p+3]] = (ledPos == 3)?color2:color1;
            leds[BackLEDs[p]]    = Color1;
            leds[BackLEDs[p+1]]  = Color1;
            leds[BackLEDs[p+2]]  = Color1;
            leds[BackLEDs[p+3]]  = Color1;
        }
    }
    else if (cmd == 'J' )
    {
        if (bReverseRightCheck) {
            color1 = Color1;
            color2 = Color2;
        } else {
            color1 = Color3;
            color2 = Color4;
        }
      
        for (p=21;p>0;p=p-4) {
                leds[BackLEDs[p]]    = (ledPos == 0)?color2:color1;
                leds[BackLEDs[p-1]]  = (ledPos == 1)?color2:color1;
                leds[BackLEDs[p-2]]  = (ledPos == 2)?color2:color1;
                leds[BackLEDs[p-3]]  = (ledPos == 3)?color2:color1;
                leds[FrontLEDs[p]]   = Color1;
                leds[FrontLEDs[p-1]] = Color1;
                leds[FrontLEDs[p-2]] = Color1;
                leds[FrontLEDs[p-3]] = Color1;
        }
    }
    else if (cmd == 'H' )
    {
        if (bReverseRightCheck) {
            color1 = Color1;
            color2 = Color2;
        } else {
            color1 = Color3;
            color2 = Color4;
        }
     
        for (p=0;p<22;p=p+4) {
            leds[BackLEDs[p]]    = (ledPos == 0)?color2:color1;
            leds[BackLEDs[p+1]]  = (ledPos == 1)?color2:color1;
            leds[BackLEDs[p+2]]  = (ledPos == 2)?color2:color1;
            leds[BackLEDs[p+3]]  = (ledPos == 3)?color2:color1;
            leds[FrontLEDs[p]]   = Color1;
            leds[FrontLEDs[p+1]] = Color1;
            leds[FrontLEDs[p+2]] = Color1;
            leds[FrontLEDs[p+3]] = Color1;
        }
    }  
    else if (led_cmd == 'Z' )
    {
        myGLCD.clrScr();
        for (p=0;p<44;p++) {
            leds[p]   = CRGB::Black;
        }
        led_cmd = ' ';
    }
    else if (led_cmd == 'A' )
    {
        char lcd_str[256];
        myGLCD.setFont(BigFont);
        sprintf(lcd_str, "Hi there...");
        myGLCD.print(lcd_str, CENTER, 100);
        
        color1 = Color5;
        color2 = Color6;
      
        for (p=0;p<44;p++) {
             leds[p] = ((p % 4) == ledPos)?color2:color1;
         }
    }
    else 
    {
        /*
        ** light up the LEDs if the car has been idle for more than
        ** idle_start and let it lit up till idle_end.
        */
        led_counter1++;
        if (led_counter1 > idle_start)
        {
            char lcd_str[256];
            float tempC;
            myGLCD.setFont(BigFont);

            color1 = htmlcolorcode[led_color1];
            color2 = htmlcolorcode[led_color2];
//            color1 = Color5;
//            color2 = Color6;

            if (led_counter1 > idle_end) {
//                led_color1 = random(0,142);
//                led_color2 = random(0,142);
                led_cmd = 'Z';
                led_counter1 = 0;
            }
            else if ((led_counter1 + 30) > idle_end) {
                myGLCD.setColor(0,255, 0);
//                sprintf(lcd_str, "ZZZzzz... %d %d %d %d", color1, led_color1, color2, led_color2);
                sprintf(lcd_str, "ZZZzzz...");
                myGLCD.print(lcd_str, CENTER, 200);
            }
            else {
             
                myGLCD.setColor(255, 0, 0);
                sprintf(lcd_str, "On standby... %d %d", led_color1, led_color2);
                myGLCD.print(lcd_str, CENTER, 100);
            }
                        
            
            led_counter2++;
            if (led_counter2 > 25) {
                led_color1 = random(0,142);
                led_color2 = random(0,142);
/*                if (led_color1 > 143)
                    led_color1=0;
                if (led_color2 > 143)
                    led_color2=0;
*/
                led_counter2 = 0;
            }
      
            for (p=0;p<44;p++) {
                leds[p]   = ((p % 4) == ledPos)?color2:color1;
            }
        }
    }
    
    FastLED.show();
    
    ledPos++;
    if (ledPos == 4)
        ledPos = 0;
}


void attach_front_sensor_servo()
{
    FrontServo.attach(FRONT_SERVO);
    bFrontSensorServoAttached = true;
}

void attach_back_sensor_servo()
{
    BackServo.attach(BACK_SERVO);
    bBackSensorServoAttached = true;
}

void detach_front_sensor_servo()
{
    FrontServo.detach();
    bFrontSensorServoAttached = false;
}

void detach_back_sensor_servo()
{
    BackServo.detach();
    bBackSensorServoAttached = false;
}

void checkSensors(void)
{
    if (cmd == 'F')
    {
        check_forward();
    }
    else if (cmd == 'B')
    {
        check_reverse();
    }
    else if (cmd == 'G')
    {
        check_forward_left();
    }
    else if (cmd == 'I')
    {
        check_forward_right();
    }
    else if (cmd == 'H')
    {
        check_reverse_left();
    }
    else if (cmd == 'J')
    {
        check_reverse_right();
    }

    servoPos++;
    if (servoPos == 4)
        servoPos = 0;
}


boolean check_forward() 
{
    FrontServo.write(ForwardServoPos[servoPos]);
    delay(50);

    if(digitalRead(FRONT_OBSTACLE_AVOIDANCE_SENSOR))  {
        bObstacle = true; 
    } else {
        bObstacle = false;
    }

    uS = FrontSonar.ping(); 
    distance = uS / US_ROUNDTRIP_CM;
    
    /* Sonar sometimes gives 0 cms as distance, ignore it */
//    if ((distance == 0 ) || (distance >= safe_distance))  {
    if (distance >= safe_distance)  {
        bSonar = true;
    } else { 
        bSonar = false;
    }

    myGLCD.clrScr();
    serial_print("forward", distance, uS, bSonar, bObstacle);

    bForwardCheckLast = bForwardCheckNow;
    if (bObstacle && bSonar) {
        myGLCD.setColor(0, 255, 0);
        bForwardCheckNow = true;
        
        if (bForwardCheckLast)
            bForwardCheck = true;
    } else {
        myGLCD.setColor(255, 0, 0);
        bForwardCheckNow = false;

        if (!bForwardCheckLast)
            bForwardCheck = false;
    }
    myGLCD.fillRect(129, 0, 189, 59);
}

boolean check_forward_left() 
{
    FrontServo.write(ForwardLeftServoPos[servoPos]);
    delay(50);

    if(digitalRead(FRONT_OBSTACLE_AVOIDANCE_SENSOR))  {
        bObstacle = true; 
    } else {
        bObstacle = false;
    }

    uS = FrontSonar.ping(); 
    distance = uS / US_ROUNDTRIP_CM;
    
    /* Sonar sometimes gives 0 cms as distance, ignore it */
//    if ((distance == 0 ) || (distance >= safe_distance))  {
    if (distance >= safe_distance)  {
        bSonar = true;
    } else { 
        bSonar = false;
    }

    myGLCD.clrScr();
    serial_print("forward_left", distance, uS, bSonar, bObstacle);

    bForwardLeftCheckLast = bForwardLeftCheckNow;
    if (bObstacle && bSonar) {
        myGLCD.setColor(0, 255, 0);
        bForwardLeftCheckNow = true;
        
        if (bForwardLeftCheckLast)
            bForwardLeftCheck = true;
    } else {
        myGLCD.setColor(255, 0, 0);
        bForwardLeftCheckNow = false;

        if (!bForwardLeftCheckLast)
            bForwardLeftCheck = false;
    }
    myGLCD.fillRect(0, 0, 59, 59);
}

boolean check_forward_right() 
{
    FrontServo.write(ForwardRightServoPos[servoPos]);
    delay(50);

    if(digitalRead(FRONT_OBSTACLE_AVOIDANCE_SENSOR))  {
        bObstacle = true; 
    } else {
        bObstacle = false;
    }

    uS = FrontSonar.ping(); 
    distance = uS / US_ROUNDTRIP_CM;
    
    /* Sonar sometimes gives 0 cms as distance, ignore it */
//    if ((distance == 0 ) || (distance >= safe_distance))  {
    if (distance >= safe_distance)  {
        bSonar = true;
    } else { 
        bSonar = false;
    }

    myGLCD.clrScr();
    serial_print("forward_right", distance, uS, bSonar, bObstacle);

    bForwardRightCheckLast = bForwardRightCheckNow;
    if (bObstacle && bSonar) {
        myGLCD.setColor(0, 255, 0);
        bForwardRightCheckNow = true;
        
        if (bForwardRightCheckLast)
            bForwardRightCheck = true;
    } else {
        myGLCD.setColor(255, 0, 0);
        bForwardRightCheckNow = false;
        
        if (!bForwardRightCheckLast)
            bForwardRightCheck = false;
    }
    myGLCD.fillRect(259, 0, 319, 59);
}

boolean check_reverse() 
{
    BackServo.write(BackwardServoPos[servoPos]);
    delay(50);

    if(digitalRead(BACK_OBSTACLE_AVOIDANCE_SENSOR))  {
        bObstacle = true; 
    } else {
        bObstacle = false;
    }

    uS = BackSonar.ping(); 
    distance = uS / US_ROUNDTRIP_CM;
    
    /* Sonar sometimes gives 0 cms as distance, ignore it */
//    if ((distance == 0 ) || (distance >= safe_distance))  {
    if (distance >= safe_distance)  {
        bSonar = true;
    } else { 
        bSonar = false;
    }

    myGLCD.clrScr();
    serial_print("reverse", distance, uS, bSonar, bObstacle);

    bReverseCheckLast = bReverseCheckNow;
    if (bObstacle && bSonar) {
        myGLCD.setColor(0, 255, 0);
        bReverseCheckNow = true;
        
        if (bReverseCheckLast)
            bReverseCheck = true;
    } else {
        myGLCD.setColor(255, 0, 0);
        bReverseCheckNow = false;
        
        if (!bReverseCheckLast)
            bReverseCheck = false;
    }
    myGLCD.fillRect(129, 179, 189, 239);
}

boolean check_reverse_left() 
{
    BackServo.write(BackwardLeftServoPos[servoPos]);
    delay(50);

    if(digitalRead(BACK_OBSTACLE_AVOIDANCE_SENSOR))  {
        bObstacle = true; 
    } else {
        bObstacle = false;
    }

    uS = BackSonar.ping(); 
    distance = uS / US_ROUNDTRIP_CM;
    
    /* Sonar sometimes gives 0 cms as distance, ignore it */
//    if ((distance == 0 ) || (distance >= safe_distance))  {
    if (distance >= safe_distance)  {
        bSonar = true;
    } else { 
        bSonar = false;
    }

    myGLCD.clrScr();
    serial_print("reverse_left", distance, uS, bSonar, bObstacle);

    bReverseLeftCheckLast = bReverseLeftCheckNow;
    if (bObstacle && bSonar) {
        myGLCD.setColor(0, 255, 0);
        bReverseLeftCheckNow = true;
        
        if (bReverseLeftCheckLast)
            bReverseLeftCheck = true;
    } else {
        myGLCD.setColor(255, 0, 0);
        bReverseLeftCheckNow = false;
        
        if (!bReverseLeftCheckLast)
            bReverseLeftCheck = false;
    }
    myGLCD.fillRect(0, 179, 59, 239);
}

boolean check_reverse_right() 
{
    BackServo.write(BackwardRightServoPos[servoPos]);
    delay(50);

    if(digitalRead(BACK_OBSTACLE_AVOIDANCE_SENSOR))  {
        bObstacle = true; 
    } else {
        bObstacle = false;
    }

    uS = BackSonar.ping(); 
    distance = uS / US_ROUNDTRIP_CM;
    
    /* Sonar sometimes gives 0 cms as distance, ignore it */
//    if ((distance == 0 ) || (distance >= safe_distance))  {
    if (distance >= safe_distance)  {
        bSonar = true;
    } else { 
        bSonar = false;
    }

    myGLCD.clrScr();
    serial_print("reverse_right", distance, uS, bSonar, bObstacle);

    bReverseRightCheckLast = bReverseRightCheckNow;
    if (bObstacle && bSonar) {
        myGLCD.setColor(0, 255, 0);
        bReverseRightCheckNow = true;
        
        if (bReverseRightCheckLast)
            bReverseRightCheck = true;
    } else {
        myGLCD.setColor(255, 0, 0);
        bReverseRightCheckNow = false;
        
        if (!bReverseRightCheckLast)
            bReverseRightCheck = false;
    }
    myGLCD.fillRect(259, 179, 319, 239);
}

void wsend(char c)
{
    Wire.beginTransmission(9);
    Wire.write(c);
    serial_println("Wire write : ",c);
    Wire.endTransmission();
}

void stop()
{
    wsend('S');
}

void forward()
{
    if (bForwardCheck) {
        wsend('F');
    } else {
        stop();
    }
}

void forward_left()
{
    if (bForwardLeftCheck) {
        wsend('G');
    } else {
        stop();
    }
}


void forward_right()
{
    if (bForwardRightCheck) {
        wsend('I');
    } else {
        stop();
    }
}

void reverse()
{
    if (bReverseCheck) {
        wsend('B');
    } else {
        stop();
    }
}

void reverse_left()
{
    if (bReverseLeftCheck) {
        wsend('H');
    } else {
        stop();
    }
}


void reverse_right()
{
    if (bReverseRightCheck) {
        wsend('J');
    } else {
        stop();
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

void getInput(void)
{
    int serial_count = 0;
    
    if (Serial3.available() || Serial.available())
    {
        led_counter1 = 0;

        // Dont try to process each and every command.  Otherwise
        // the arduino may be busy for a long time after the user
        // has released a command button.
        while (Serial3.available() && serial_count < 10)
        {
            serial_count++;
            temp_cmd = Serial3.read();
            if (strchr(ACCEPTED_INPUTS, temp_cmd))
            {
                Serial.print("btemp_cmd: ");Serial.print(temp_cmd);Serial.print(" ");Serial.println(count);
                cmd = temp_cmd;
                count = 0;
                Serial.print("bcmd: ");Serial.print(cmd);Serial.print(" ");Serial.println(count);
            }
            else
            {
                Serial.print("binv temp_cmd: ");Serial.print(temp_cmd);Serial.print(" ");Serial.println(count);
            }
        }
    
        while (Serial.available() && serial_count < 10)
        {
            serial_count++;
            temp_cmd = Serial.read();
            if (strchr(ACCEPTED_INPUTS, temp_cmd))
            {
                Serial.print("temp_cmd: ");Serial.print(temp_cmd);Serial.print(" ");Serial.println(count);
                cmd = temp_cmd;
                count = 0;
                Serial.print("cmd: ");Serial.print(cmd);Serial.print(" ");Serial.println(count);
            }
            else
            {
                Serial.print("inv temp_cmd: ");Serial.print(temp_cmd);Serial.print(" ");Serial.println(count);
            }
        }
    
    }
}

void loop() { 
    char temp_str[256];
    int serial_count = 0;

    /* 
    ** Clear LCD screen if no activity for some time.  Otherwise
    ** there will be a "screen burn in at the place on the screen
    ** of the last car movement.
    */
    if (count > 100)
    {
        if (bFrontSensorServoAttached)
            detach_front_sensor_servo();
        if (bBackSensorServoAttached)
            detach_back_sensor_servo();

        if (count < 200) {
            led_cmd = 'Z';
        }
        else if ((count > 300) && (count < 350)) {
            led_cmd = ' ';
            led_counter1 = 500;
        }    
    }
      
    getInput();

    if ((cmd == 'F') || (cmd == 'G') || (cmd == 'I')) {
        if (!bFrontSensorServoAttached)
            attach_front_sensor_servo();
        if (bBackSensorServoAttached)
            detach_back_sensor_servo();
    }
    else if ((cmd == 'B') || (cmd == 'H') || (cmd == 'J')) {
        if (!bBackSensorServoAttached)
            attach_back_sensor_servo();
        if (bFrontSensorServoAttached)
            detach_front_sensor_servo();
    }
    
    checkSensors();

   
    if (cmd == 'F')
    {
        forward();
    }
    else if (cmd == 'B')
    {
        reverse();
    }
    else if (cmd == 'G')
    {
        forward_left();
    }
    else if (cmd == 'I')
    {
        forward_right();
    }
    else if (cmd == 'H')
    {
        reverse_left();
    }
    else if (cmd == 'J')
    {
        reverse_right();
    }
    else if (cmd == 'S')
    {
        stop();
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
