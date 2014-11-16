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
#define PIN_BATTERY_VOLTAGE   6  // Analog
#define PIN_MAINS_VOLTAGE     7  // Analog

#define PIN_RELAY_1     44
#define PIN_RELAY_2     42
#define PIN_RELAY_3     8
#define PIN_RELAY_4     9

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
volatile boolean bPanSensorServoAttached = false;
volatile boolean bTiltSensorServoAttached = false;

NewPing FrontSonar(FRONT_ULTRASONIC_SENSOR_TRIGGER, FRONT_ULTRASONIC_SENSOR_ECHO, MAX_DISTANCE); 
NewPing BackSonar(BACK_ULTRASONIC_SENSOR_TRIGGER, BACK_ULTRASONIC_SENSOR_ECHO, MAX_DISTANCE); 

Servo FrontServo;
Servo BackServo;
Servo PanServo;
Servo TiltServo;

char *ACCEPTED_INPUTS = "FBGIHJSDURLWwVvXx123456789";
volatile char cmd = 'A';
volatile char temp_cmd = ' ';
char prev_cmd = ' ';
int i=0,j=90,k=180,l=0;
char output_string[256];
char debug_strings[10][256] = {' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};
char null_string[256] = "                                                                                                                             ";
char str[256];
char lcd_str[256];

float tempC = 0, battery_voltage = 0, mains_voltage = 0;

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

volatile boolean bDockingMode = false;
volatile boolean bPowerFromBattery = true;
volatile boolean bPowerFromMains = true;

volatile boolean bFrontLights = false;
volatile boolean bBackLights = false;

volatile boolean bGetSensorReadings = true;

volatile int ledPos = 0;
int LeftLEDs[] = { 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1000, 1000 };
int RightLEDs[] = { 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 1000, 1000 };
int FrontLEDs[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 1000, 1000 };
int BackLEDs[] = { 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 1000, 1000 };

int FrontLightLEDs[] = { 7, 8, 9, 10, 11, 12, 13, 14 };
int BackLightLEDs[] = { 29, 30, 31, 32, 33, 34, 35, 36 };

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
volatile int idle_start = 300;
volatile int idle_end   = idle_start + 10;

int serial_count = 0;

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
        
        myGLCD.setFont(BigFont);
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
        myGLCD.setFont(BigFont);
    }
}

void getSensorReadings()
{
    int analogPinReading = 0;
    sensor_t sensor;
    sensors_event_t event; 
  
    Serial.print  ("Mag Sensor:   "); Serial.println(sensor.name);
    Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
    Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
    Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT"); 

    // Get a new sensor event
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
  
    analogPinReading = analogRead(PIN_TEMPERATURE);
    tempC = ((float)analogPinReading)*0.48828125;

    analogPinReading = analogRead(PIN_BATTERY_VOLTAGE);
    battery_voltage = ((float)analogPinReading) * (5.0 /1024.0) * (resistor1 + resistor2) / resistor2;

    analogPinReading = analogRead(PIN_MAINS_VOLTAGE);
    mains_voltage = ((float)analogPinReading) * (5.0 /1024.0) * (resistor1 + resistor2) / resistor2;

    Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
    Serial.print("Temperature: "); Serial.println(tempC);
    Serial.print("Battery voltage: "); Serial.println(battery_voltage);
    Serial.print("Mains voltage: "); Serial.println(mains_voltage);

    dtostrf(tempC, 5, 2, str);           sprintf(lcd_str, "Temperature: %sC", str);
    myGLCD.print(lcd_str, LEFT, 100);
    dtostrf(headingDegrees, 6, 2, str);  sprintf(lcd_str, "Heading: %s\%", str);
    myGLCD.print(lcd_str, LEFT, 120);
    dtostrf(battery_voltage, 5, 2, str); sprintf(lcd_str, "Battery: %s v", str);
    myGLCD.print(lcd_str, LEFT, 140);
    dtostrf(mains_voltage, 5, 2, str);   sprintf(lcd_str, "Mains  : %s v", str);
    myGLCD.print(lcd_str, LEFT, 160);
}

void initializeServos()
{
    attach_front_sensor_servo();
    attach_back_sensor_servo();
    attach_pan_sensor_servo();
    attach_tilt_sensor_servo();
  
    FrontServo.write(90);
    BackServo.write(90);
    PanServo.write(90);
    TiltServo.write(90);
    delay(1000);
    
    detach_front_sensor_servo();
    detach_back_sensor_servo();
    detach_pan_sensor_servo();
    detach_tilt_sensor_servo();
}

void setup() {
    Serial.begin(57600);           // USB connection to Raspberry Pi
    Serial3.begin(9600);           // HC-05 Bluetooth module
    Wire.begin();

    // Initialise the sensor
    if(!mag.begin())
    {
        /* There was a problem detecting the HMC5883 ... check your connections */
        Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    }    
    
    Serial.println("Starting Arduino Mega ...................");

    analogReference(DEFAULT);

    pinMode(FRONT_ULTRASONIC_SENSOR_TRIGGER, OUTPUT);
    pinMode(FRONT_ULTRASONIC_SENSOR_ECHO,    INPUT);
    pinMode(BACK_ULTRASONIC_SENSOR_TRIGGER,  OUTPUT);
    pinMode(BACK_ULTRASONIC_SENSOR_ECHO,     INPUT);
    pinMode(FRONT_OBSTACLE_AVOIDANCE_SENSOR, INPUT);
    pinMode(BACK_OBSTACLE_AVOIDANCE_SENSOR,  INPUT);

    pinMode(PIN_RELAY_1,                     OUTPUT);
    pinMode(PIN_RELAY_2,                     OUTPUT);
    pinMode(PIN_RELAY_3,                     OUTPUT);
    pinMode(PIN_RELAY_4,                     OUTPUT);

    digitalWrite(PIN_RELAY_1, HIGH);
    digitalWrite(PIN_RELAY_2, HIGH);
    digitalWrite(PIN_RELAY_3, HIGH);
    digitalWrite(PIN_RELAY_4, HIGH);
    
    myGLCD.InitLCD();
    myGLCD.setFont(BigFont);
    myGLCD.clrScr();        
    myGLCD.print("Initializing..", CENTER, 60);
    
    FastLED.addLeds<NEOPIXEL, LED_DATA_PIN, RGB>(leds, NUM_LEDS);

//    NewPing::timer_ms(300, ledStrip);
//    NewPing::timer_ms(100, checkSensors);

//    Timer3.initialize(100000);
//    Timer3.attachInterrupt(ledStrip);

//    Timer3.initialize(10000);
//    Timer3.attachInterrupt(getInput);


    getSensorReadings();
    initializeServos();
    
    if (mains_voltage > 10.0)
    {
        powerFromMains();
    }
    else
    {
        powerFromBattery();
    }
    
    delay(1000);
    count = 0;
    cmd = 'Z';
    led_counter1 = 0;
}

void powerFromMains()
{
    digitalWrite(PIN_RELAY_1, HIGH);
    digitalWrite(PIN_RELAY_2, HIGH);
    digitalWrite(PIN_RELAY_3, HIGH);
    digitalWrite(PIN_RELAY_4, LOW);
    
    bPowerFromMains = true;
    bPowerFromBattery = false;

    idle_start = 50;
    idle_end   = idle_start + 50;
}

void powerFromBattery()
{
    digitalWrite(PIN_RELAY_1, LOW);
    digitalWrite(PIN_RELAY_2, HIGH);
    digitalWrite(PIN_RELAY_3, HIGH);
    digitalWrite(PIN_RELAY_4, HIGH);

    bPowerFromMains = false;
    bPowerFromBattery = true;

    idle_start = 300;
    idle_end   = idle_start + 10;
}

void powerFromBothMainsAndBattery()
{
    digitalWrite(PIN_RELAY_1, HIGH);
    digitalWrite(PIN_RELAY_2, HIGH);
    digitalWrite(PIN_RELAY_3, HIGH);
    digitalWrite(PIN_RELAY_4, HIGH);

    bPowerFromMains = true;
    bPowerFromBattery = true;
    
}

void showPowerStatus()
{
    if (bPowerFromMains)
    {
        myGLCD.setColor(255, 0, 0);
        myGLCD.fillRect(0, 0, 159, 50);
        myGLCD.setColor(0, 0, 0);
        myGLCD.print("Mains: On", LEFT, 20);
    }
    else
    {
        myGLCD.setColor(0, 0, 0);
        myGLCD.fillRect(0, 0, 159, 50);
        myGLCD.setColor(255, 255, 255);
        myGLCD.print("Mains: Off", LEFT, 20);
    }

    if (bPowerFromBattery)
    {
        myGLCD.setColor(255, 0, 0);
        myGLCD.fillRect(160, 0, 319, 50);
        myGLCD.setColor(0, 0, 0);
        myGLCD.print("Batt: On", RIGHT, 20);
    }
    else
    {
        myGLCD.setColor(0, 0, 0);
        myGLCD.fillRect(160, 0, 319, 50);
        myGLCD.setColor(255, 255, 255);
        myGLCD.print("Batt: Off", RIGHT, 20);
    }
}

void ledStrip(void)
{
    int p;
    int color1,color2;

//    count++;
    
    switch(cmd)
    {
        case 'F':
        if (bForwardCheck) {
            color1 = Color1;
            color2 = Color2;
        } else {
            color1 = Color3;
            color2 = Color4;
        }
      
        for (p=0;p<22;p=p+8) {
            leds[LeftLEDs[p]]    = (ledPos == 0)?color2:color1;
            leds[LeftLEDs[p+1]]  = (ledPos == 1)?color2:color1;
            leds[LeftLEDs[p+2]]  = (ledPos == 2)?color2:color1;
            leds[LeftLEDs[p+3]]  = (ledPos == 3)?color2:color1;
            leds[RightLEDs[p]]   = (ledPos == 0)?color2:color1;
            leds[RightLEDs[p+1]] = (ledPos == 1)?color2:color1;
            leds[RightLEDs[p+2]] = (ledPos == 2)?color2:color1;
            leds[RightLEDs[p+3]] = (ledPos == 3)?color2:color1;
        }
        break;
        
        case 'B':
        if (bReverseCheck) {
            color1 = Color1;
            color2 = Color2;
        } else {
            color1 = Color3;
            color2 = Color4;
        }
      
        for (p=21;p>0;p=p-8) {
            leds[LeftLEDs[p]]    = (ledPos == 0)?color2:color1;
            leds[LeftLEDs[p-1]]  = (ledPos == 1)?color2:color1;
            leds[LeftLEDs[p-2]]  = (ledPos == 2)?color2:color1;
            leds[LeftLEDs[p-3]]  = (ledPos == 3)?color2:color1;
            leds[RightLEDs[p]]   = (ledPos == 0)?color2:color1;
            leds[RightLEDs[p-1]] = (ledPos == 1)?color2:color1;
            leds[RightLEDs[p-2]] = (ledPos == 2)?color2:color1;
            leds[RightLEDs[p-3]] = (ledPos == 3)?color2:color1;
        }
        break;
        
        case 'G':
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
        break;
        
        case 'I':
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
        break;
        
        case 'J':
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
        break;
        
        case 'H':
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
        break;
        
        case 'A':
        
        color1 = Color5;
        color2 = Color6;
      
        for (p=0;p<44;p++) {
             leds[p] = ((p % 4) == ledPos)?color2:color1;
        }
        break;

        case 'Z':
        for (p=0;p<44;p++) {
            leds[p]   = CRGB::Black;
        }
        cmd = ' ';
        break;
         
        default:
        /*
        ** light up the LEDs if the car has been idle for more than
        ** idle_start and let it lit up till idle_end.
        */
        led_counter1++;
        if (led_counter1 >= idle_start)
        {
            color1 = htmlcolorcode[led_color1];
            color2 = htmlcolorcode[led_color2];
            

            if (led_counter1 == idle_start)
            {
                bGetSensorReadings = true;
            }

            if (bPowerFromMains && !bPowerFromBattery)
            {
                for (p=0;p<44;p++) {
                    leds[p]   = ((p % 4) == ledPos)?color2:color1;
                }
            }
            else
            {
                for (p=0;p<44;p++) {
                    leds[p]   = CRGB::Black;
                }
            }

            if (led_counter1 > idle_end) {
                myGLCD.clrScr();
                for (p=0;p<44;p++) {
                    leds[p]   = CRGB::Black;
                }
                led_counter1 = 0;
                if (bFrontLights)
                    bFrontLights = false;
                if (bBackLights)
                    bBackLights = false;
            }

            led_counter2++;
        }
    }
    
    if (bFrontLights)
    {
        for (i=0;i<8;i++)
        {
            leds[FrontLightLEDs[i]] = CRGB::White;
        }
    }
    
    if (bBackLights)
    {
        for (i=0;i<8;i++)
        {
            leds[BackLightLEDs[i]] = CRGB::White;
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
    switch (cmd)
    {
        case 'F':
            check_forward();
            break;
        case 'B':
            check_reverse();
            break;
        case 'G':
            check_forward_left();
            break;
        case 'I':
            check_forward_right();
            break;
        case 'H':
            check_reverse_left();
            break;
        case 'J':
            check_reverse_right();
            break;
    }

    servoPos++;
    if (servoPos == 4)
        servoPos = 0;
}


boolean check_forward() 
{
    attach_front_sensor_servo();
    FrontServo.write(ForwardServoPos[servoPos]);
    delay(50);
    detach_front_sensor_servo();

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
    attach_front_sensor_servo();
    FrontServo.write(ForwardLeftServoPos[servoPos]);
    delay(50);
    detach_front_sensor_servo();

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
    attach_front_sensor_servo();
    FrontServo.write(ForwardRightServoPos[servoPos]);
    delay(50);
    detach_front_sensor_servo();

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
    attach_back_sensor_servo();
    BackServo.write(BackwardServoPos[servoPos]);
    delay(50);
    detach_back_sensor_servo();

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
    attach_back_sensor_servo();
    BackServo.write(BackwardLeftServoPos[servoPos]);
    delay(50);
    detach_back_sensor_servo();

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
    attach_back_sensor_servo();
    BackServo.write(BackwardRightServoPos[servoPos]);
    delay(50);
    detach_back_sensor_servo();

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
/*    if (bFrontSensorServoAttached)
        detach_front_sensor_servo();
    if (bBackSensorServoAttached)
        detach_back_sensor_servo();
*/    if (bPanSensorServoAttached)
        detach_pan_sensor_servo();
    if (bTiltSensorServoAttached)
        detach_tilt_sensor_servo();
}

void forward()
{
    if (bForwardCheck  || bDockingMode) {
        wsend('F');
    } else {
        stop();
    }
}

void forward_left()
{
    if (bForwardLeftCheck || bDockingMode) {
        wsend('G');
    } else {
        stop();
    }
}

void forward_right()
{
    if (bForwardRightCheck || bDockingMode) {
        wsend('I');
    } else {
        stop();
    }
}

void reverse()
{
    if (bReverseCheck || bDockingMode) {
        wsend('B');
    } else {
        stop();
    }
}

void reverse_left()
{
    if (bReverseLeftCheck || bDockingMode) {
        wsend('H');
    } else {
        stop();
    }
}

void reverse_right()
{
    if (bReverseRightCheck || bDockingMode) {
        wsend('J');
    } else {
        stop();
    }
}

void attach_pan_sensor_servo()
{
    PanServo.attach(PAN_SERVO);
    bPanSensorServoAttached = true;
}

void attach_tilt_sensor_servo()
{
    TiltServo.attach(TILT_SERVO);
    bTiltSensorServoAttached = true;
}

void detach_pan_sensor_servo()
{
    PanServo.detach();
    bPanSensorServoAttached = false;
}

void detach_tilt_sensor_servo()
{
    TiltServo.detach();
    bTiltSensorServoAttached = false;
}

void camera_up()
{
    if (TiltServoPosition > 40)
    {
        TiltServoPosition -= 1;
        TiltServo.write(TiltServoPosition);
        delay(100);
    }
}

void camera_down()
{
    if (TiltServoPosition < 180)
    {
        TiltServoPosition += 1;
        TiltServo.write(TiltServoPosition);
        delay(100);
    }
}

void camera_left()
{
    if (PanServoPosition < 180)
    {
        PanServoPosition += 1;
        PanServo.write(PanServoPosition);
        delay(100);
    }
}

void camera_right()
{
    if (PanServoPosition > 0)
    {
        PanServoPosition -= 1;
        PanServo.write(PanServoPosition);
        delay(100);
    }
}

void getInput(void)
{
    serial_count = 0;
    
//    if (Serial3.available() || Serial.available())
//    {
        // Dont try to process each and every command.  Otherwise
        // the arduino may be busy for a long time after the user
        // has released a command button.
        while (Serial3.available() && serial_count < 50)
        {
            serial_count++;
            temp_cmd = Serial3.read();
            if (strchr(ACCEPTED_INPUTS, temp_cmd))
            {
                cmd = temp_cmd;
                led_counter1 = 0;
                count = 0;
//                Serial.print("bcmd: ");Serial.print(cmd);Serial.print(" ");Serial.println(count);
            }
        }

        serial_count = 0;
    
        while (Serial.available() && serial_count < 50)
        {
            serial_count++;
            temp_cmd = Serial.read();
            if (strchr(ACCEPTED_INPUTS, temp_cmd))
            {
                cmd = temp_cmd;
                led_counter1 = 0;
                count = 0;
//                Serial.print("cmd: ");Serial.print(cmd);Serial.print(" ");Serial.println(count);
            }
        }
//    }
}

void loop() { 
    if (bGetSensorReadings)
    {
        getSensorReadings();
        showPowerStatus();
        bGetSensorReadings = false;
    }

    count++;
    if (count > 10000)
    {
/*        if (bFrontSensorServoAttached)
            detach_front_sensor_servo();
        if (bBackSensorServoAttached)
            detach_back_sensor_servo();
*/        if (bPanSensorServoAttached)
            detach_pan_sensor_servo();
        if (bTiltSensorServoAttached)
            detach_tilt_sensor_servo();

//        cmd = 'S';  
        count = 0;
        bGetSensorReadings = false;
        myGLCD.clrScr();
    }
      
    getInput();

//Serial.print(':');
//Serial.print(cmd);

/*    if (cmd == ' ')
    {
        if (led_counter2 > 25) {
            led_color1 = random(0,142);
            led_color2 = random(0,142);
            led_counter2 = 0;
        }
        if (led_counter1 > idle_end) {
            myGLCD.clrScr();
        }        

        return;
    }
*/    
    if ((cmd == 'F') || (cmd == 'G') || (cmd == 'I')) {
/*        if (!bFrontSensorServoAttached)
            attach_front_sensor_servo();
        if (bBackSensorServoAttached)
            detach_back_sensor_servo();
*/        if (bPanSensorServoAttached)
            detach_pan_sensor_servo();
        if (bTiltSensorServoAttached)
            detach_tilt_sensor_servo();
        bGetSensorReadings = true;
        checkSensors();
    }
    else if ((cmd == 'B') || (cmd == 'H') || (cmd == 'J')) {
/*        if (bFrontSensorServoAttached)
            detach_front_sensor_servo();
        if (!bBackSensorServoAttached)
            attach_back_sensor_servo();
*/        if (bPanSensorServoAttached)
            detach_pan_sensor_servo();
        if (bTiltSensorServoAttached)
            detach_tilt_sensor_servo();
        bGetSensorReadings = true;
        checkSensors();
    }
    else if ((cmd == 'U') || (cmd == 'D')) {
/*        if (bFrontSensorServoAttached)
            detach_front_sensor_servo();
        if (bBackSensorServoAttached)
            detach_back_sensor_servo();
*/        if (bPanSensorServoAttached)
            detach_pan_sensor_servo();
        if (!bTiltSensorServoAttached)
            attach_tilt_sensor_servo();
    }
    else if ((cmd == 'L') || (cmd == 'R')) {
/*        if (bFrontSensorServoAttached)
            detach_front_sensor_servo();
        if (bBackSensorServoAttached)
            detach_back_sensor_servo();
*/        if (!bPanSensorServoAttached)
            attach_pan_sensor_servo();
        if (bTiltSensorServoAttached)
            detach_tilt_sensor_servo();
    }

// Serial.println(cmd);    
    switch (cmd)
    {
        case 'F': 
            forward();
            break;
        case 'B':
            reverse();
            break;
        case 'G': 
            forward_left(); 
            break;
        case 'I':
            forward_right();
            break;
        case 'H':
            reverse_left();
            break;
        case 'J':
            reverse_right();
            break;
        case 'S':
            stop();
            bGetSensorReadings = false;
            cmd = ' ';
            break;
        case 'U':
            camera_up();
            break;
        case 'D':
            camera_down();
            break;
        case 'R':
            camera_right();
            break;
        case 'L':
            camera_left();
            break;
        case '1':
            wsend('1');
            cmd = ' ';
            break;
        case '2':
            wsend('2');
            cmd = ' ';
            break;
        case '3':
            wsend('3');
            cmd = ' ';
            break;
        case '4':
            wsend('4');
            cmd = ' ';
            break;
        case '5':
            wsend('5');
            cmd = ' ';
            break;
        case '6':
            wsend('6');
            cmd = ' ';
            break;
        case '7':
            wsend('7');
            cmd = ' ';
            break;
        case '8':
            wsend('8');
            cmd = ' ';
            break;
        case '9':
            wsend('9');
            cmd = ' ';
            break;
        case 'V':
            powerFromBothMainsAndBattery();
            delay(50);
            powerFromBattery();
            cmd = ' ';
            break;
        case 'v':
            powerFromBothMainsAndBattery();
            delay(50);
            powerFromMains();
            cmd = ' ';
            break;
        case 'X':
            bDockingMode = true;
            cmd = ' ';
            break;
        case 'x':
            bDockingMode = false;
            cmd = ' ';
            break;
        case 'W':
            bFrontLights = true;
            cmd = ' ';
            break;
        case 'w':
            bFrontLights = false;
            cmd = ' ';
            break;
    }
}
