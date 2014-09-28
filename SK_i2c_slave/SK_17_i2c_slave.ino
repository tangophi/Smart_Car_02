#include <AFMotor.h>
#include <Wire.h>

AF_DCMotor RBmotor(1, MOTOR12_64KHZ);   // right back wheel
AF_DCMotor RFmotor(2, MOTOR12_64KHZ);   // right front wheel
AF_DCMotor LBmotor(4, MOTOR34_64KHZ);   // left back wheel
AF_DCMotor LFmotor(3, MOTOR34_64KHZ);   // left front wheel

int min_speed = 0;
int max_speed = 100;
int max_speed2 = 250;
int move_duration = 1;
char cmd = ' ';
char prev_cmd = ' ';
int iteration_delay = 0;
int loop_count = 0;
int reset_delay = 10;
long int count = 0;

void setup() {
    Serial.begin(9600);           // set up Serial library at 9600 bps

    Wire.begin(9);
    Wire.onReceive(receiveEvent);
    
    // turn on motor
    RBmotor.setSpeed(200);
    RFmotor.setSpeed(200);
    LBmotor.setSpeed(200);
    LFmotor.setSpeed(200);

    RBmotor.run(RELEASE);
    RFmotor.run(RELEASE);
    LBmotor.run(RELEASE);
    LFmotor.run(RELEASE);

}

void receiveEvent(int howMany)
{
    cmd = Wire.read();
}

void serial_print (char *direction, int secs, int cms)
{
    char output_string[256];
    sprintf(output_string, "%-13s - %3d cms - %5d microseconds", direction, secs, cms);
    Serial.println(output_string);
}

void stop()
{
    RBmotor.setSpeed(0);  
    RFmotor.setSpeed(0);  
    LBmotor.setSpeed(0);  
    LFmotor.setSpeed(0);  
    
    RBmotor.run(RELEASE);
    RFmotor.run(RELEASE);
    LBmotor.run(RELEASE);
    LFmotor.run(RELEASE);
}

void stop2()
{
    delay(20);
    cmd = ' ';
    stop();
}

void forward(){
    int i = 0;
    RBmotor.run(FORWARD);
    RFmotor.run(FORWARD);
    LBmotor.run(FORWARD);
    LFmotor.run(FORWARD);

    for (i=0; i<move_duration; i++) {
        RBmotor.setSpeed(max_speed);  
        RFmotor.setSpeed(max_speed);  
        LBmotor.setSpeed(max_speed);  
        LFmotor.setSpeed(max_speed);  
        delay(iteration_delay);
    }
}

void reverse(){
    int i = 0;
    RBmotor.run(BACKWARD);
    RFmotor.run(BACKWARD);
    LBmotor.run(BACKWARD);
    LFmotor.run(BACKWARD);

    for (i=0; i<move_duration; i++) {
        RBmotor.setSpeed(max_speed);  
        RFmotor.setSpeed(max_speed);  
        LBmotor.setSpeed(max_speed);  
        LFmotor.setSpeed(max_speed);  
        delay(iteration_delay);
    }
}

void forward_left(){
    int i = 0;
    RBmotor.run(FORWARD);
    RFmotor.run(FORWARD);
    LBmotor.run(FORWARD);
    LFmotor.run(FORWARD);

    for (i=0; i<move_duration; i++) {
        RBmotor.setSpeed(max_speed2);  
        RFmotor.setSpeed(max_speed2);  
        LBmotor.setSpeed(min_speed);  
        LFmotor.setSpeed(min_speed);  
        delay(iteration_delay);
    }
}

void forward_right(){
    int i = 0;
    RBmotor.run(FORWARD);
    RFmotor.run(FORWARD);
    LBmotor.run(FORWARD);
    LFmotor.run(FORWARD);

    for (i=0; i<move_duration; i++) {
        RBmotor.setSpeed(min_speed);  
        RFmotor.setSpeed(min_speed);  
        LBmotor.setSpeed(max_speed2);  
        LFmotor.setSpeed(max_speed2);  
        delay(iteration_delay);
    }
}

void reverse_left(){
    int i = 0;
    RBmotor.run(BACKWARD);
    RFmotor.run(BACKWARD);
    LBmotor.run(BACKWARD);
    LFmotor.run(BACKWARD);

    for (i=0; i<move_duration; i++) {
        RBmotor.setSpeed(max_speed2);  
        RFmotor.setSpeed(max_speed2);  
        LBmotor.setSpeed(min_speed);  
        LFmotor.setSpeed(min_speed);  
        delay(iteration_delay);
    }
}

void reverse_right(){
    int i = 0;
    RBmotor.run(BACKWARD);
    RFmotor.run(BACKWARD);
    LBmotor.run(BACKWARD);
    LFmotor.run(BACKWARD);

    for (i=0; i<move_duration; i++) {
        RBmotor.setSpeed(min_speed);  
        RFmotor.setSpeed(min_speed);  
        LBmotor.setSpeed(max_speed2);  
        LFmotor.setSpeed(max_speed2);  
        delay(iteration_delay);
    }
}

void loop() { 
    count++;
    
    
    
    if ((count < 300) && (prev_cmd != ' '))
    {
        if (prev_cmd == 'F') {
            forward();
        } else if (prev_cmd == 'B') {
            reverse();
        } else if (prev_cmd == 'G') {
            forward_left();
            if (count > 50) 
                prev_cmd = ' ';
        } else if (prev_cmd == 'I') {
            forward_right();
            if (count > 50) 
                prev_cmd = ' ';
        } else if (prev_cmd == 'H') {
            reverse_left();
            if (count > 50) 
                prev_cmd = ' ';
        } else if (prev_cmd == 'J') {
            reverse_right();
            if (count > 50) 
                prev_cmd = ' ';
        }      
    }
    else
    {
        stop();
        count = 0;
        prev_cmd = ' ';
    }
    
    if (cmd == 'F') {
        forward();
        prev_cmd = cmd;
        cmd = ' ';
        count = 0;
    } else if (cmd == 'B') {
        reverse();
        prev_cmd = cmd;
        cmd = ' ';
        count = 0;
    } else if (cmd == 'G') {
        forward_left();
        prev_cmd = cmd;
        cmd = ' ';
        count = 0;
    } else if (cmd == 'I') {
        forward_right();
        prev_cmd = cmd;
        cmd = ' ';
        count = 0;
    } else if (cmd == 'H') {
        reverse_left();
        prev_cmd = cmd;
        cmd = ' ';
        count = 0;
    } else if (cmd == 'J') {
        reverse_right();
        prev_cmd = cmd;
        cmd = ' ';
        count = 0;
    } else if (cmd == 'S') {
        stop();
        prev_cmd = ' ';
        cmd = ' ';
        count = 0;
    }
}
