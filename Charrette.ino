/*
 * Thibaut Waechter
 * GE2
 * 
 * Grégoire Pêche
 * GE4
 */

/****************************************************************************************
 * CHARRETTE PROJECT
 ****************************************************************************************/

// Include the servo library
#include <Servo.h>

/****************************************************************************************
 * ENUMERATION
 ****************************************************************************************/

// Declaration of possible directions
enum
{
    FORWARD,
    BACKWARDS,
    RIGHT,
    LEFT,
    STOP
};

/****************************************************************************************/

// Declaration of possible modes
enum
{
    LINE_R,
    LINE_L,
    SOLO,
    DUO
};


/****************************************************************************************
 * PINS
 ****************************************************************************************/

// PWM pins (servo)

const int R_servo = 10;
const int L_servo = 11;

/****************************************************************************************/

// Analog pins (optical sensor)

const int L_sensor = A3;
const int R_sensor = A4;

/****************************************************************************************/

// Digital pins (contact sensor)

const int SELECT_sensor = 8;
const int START_sensor = 9;

/****************************************************************************************/

// Digital pins (LED)

const int START_led = 4;
const int LINE_led = 7;
const int SOLO_led = 2;
const int DUO_led = 3;

/****************************************************************************************/

// Digital pins (distance sensor)

const int trigger = 12;
const int echo = 13;

/****************************************************************************************
 * VARIABLES
 ****************************************************************************************/

// Declaration of the servo

Servo L;
Servo R;

/****************************************************************************************/

// Maximum speed
int max_speed = 100;

// Start speed
const int start_speed = 90;

// Rise time between each speed
const int min_delay = 12;

// Delay during which the robot ssees nothing
const int blind_time = 200;

/****************************************************************************************/

// Current direction
int dir = STOP;

// Beginning of the action
int start = 0;

// Indicates the operating mode
int count = 0;

// Operating mode of the robot
int action_mode = LINE_L;

/****************************************************************************************/

// Indicate if the robot has find a target
bool lock = false;

// Indicate if the robot is turning to the right
bool turn_right = false;

// Indicate if the robot is in the finale state
bool finale = false;

// Indicate if it is the beginning of a figth
bool beginning = true;

/****************************************************************************************/

// Threshold of the optical sensor
const int optical_threshold = 400;

/****************************************************************************************
 * DECLARATION FUNCTIONS
 ****************************************************************************************/

/// @brief Stop the robot.
void stop_bot();

/// @brief The robot turn to the left.
void left();

/// @brief The robot turn to the right.
void right();

/// @brief The robot move backwards.
void backwards();

/// @brief The robot move forward.
void forward();

/****************************************************************************************/

/// @brief Check whitch direction to take for the LINE mode right sensor.
int check_direction_right();

/// @brief Check whitch direction to take for the LINE mode left sensor.
int check_direction_left();

/// @brief The robot works in LINE mode with the right sensor.
void line_action_right();

/// @brief The robot works in LINE mode with the left sensor.
void line_action_left();

/// @brief Returns the distance to a near object.
int distance();

/// @brief Check whitch direction to take in orther to not fall.
int check_edge();

/// @brief The robot works in SOLO mode.
void solo_action();

/// @brief The robot works in DUO mode.
void duo_action();

/****************************************************************************************
 * SETUP
 ****************************************************************************************/

void setup()
{
    // Rate of communication
    Serial.begin(9600);

    // Driver of the servo
    pinMode(R_servo, OUTPUT);
    pinMode(L_servo, OUTPUT);

    // Optical sensor
    pinMode(L_sensor, INPUT);
    pinMode(R_sensor, INPUT);

    // Contact sensor
    pinMode(SELECT_sensor, INPUT);
    pinMode(START_sensor, INPUT);

    // Distance sensor
    pinMode(trigger, OUTPUT);
    pinMode(echo, INPUT);

    // Display
    pinMode(START_led, OUTPUT);
    pinMode(LINE_led, OUTPUT);
    pinMode(SOLO_led, OUTPUT);
    pinMode(DUO_led, OUTPUT);

    // Servo
    R.attach(R_servo);
    L.attach(L_servo);

    // Make sure that the bot is not going anywhere
    stop_bot();

    // Check for update until the user presses the start button
    while(digitalRead(START_sensor))
    {
        update_mode();
    }

    // Wait for 1 secondes
    delay(1000);
    
    // Beginnig
    start = millis();
    
    // Switch on the strat LED
    digitalWrite(START_led, HIGH);
}

/****************************************************************************************
 * LOOP
 ****************************************************************************************/

void loop()
{
    //Serial.println("R : " + static_cast<String>(analogRead(R_sensor)) + " L : " + static_cast<String>(analogRead(L_sensor)));
    //Serial.println("Distance : " + static_cast<String>(distance()));

    if(digitalRead(START_sensor))
    {
        switch(action_mode)
        {
            case LINE_R:                
                line_action_right();
                break;

            case LINE_L:                
                line_action_left();
                break;
    
            case SOLO:
                solo_action();
                break;
    
            case DUO:
                duo_action();
                break;
    
            default:
                break;
        }
    }
    else
    {
        // Make sure that the bot is not going anywhere
        stop_bot();

        // Wait for 1 secondes
        delay(1000);
      
        // Check for update until the user presses the start button
        while(digitalRead(START_sensor))
        {
            update_mode();
        }

        // Wait for 1 secondes
        delay(1000);
    
        // Beginnig
        start = millis();
    
        // Switch on the strat LED
        digitalWrite(START_led, HIGH);
    }
    
}

/****************************************************************************************
 * GENERAL PURPOSE FUNCTIONS
 ****************************************************************************************/

void update_mode()
{
    // Increment count to select different mode with one sensor
    if(!digitalRead(SELECT_sensor))
    {
        if(count < 3)
        {
            count++;
            delay(300);
        }
        else
        {
            count = 0;
            delay(300);
        }
    }
    
  
    // Determinate the mode based on count
    switch(count)
    {
        case 0:
            action_mode = LINE_R;
  
            digitalWrite(START_led, LOW);
            digitalWrite(LINE_led, HIGH);
            digitalWrite(SOLO_led, HIGH);
            digitalWrite(DUO_led, LOW);
            break;

        case 1:
            action_mode = LINE_L;
  
            digitalWrite(START_led, LOW);
            digitalWrite(LINE_led, HIGH);
            digitalWrite(SOLO_led, LOW);
            digitalWrite(DUO_led, HIGH);
            break;

        case 2:
            action_mode = SOLO;
  
            digitalWrite(START_led, LOW);
            digitalWrite(LINE_led, LOW);
            digitalWrite(SOLO_led, HIGH);
            digitalWrite(DUO_led, LOW);
            break;

        case 3:
            action_mode = DUO;
  
            digitalWrite(START_led, LOW);
            digitalWrite(LINE_led, LOW);
            digitalWrite(SOLO_led, LOW);
            digitalWrite(DUO_led, HIGH);
            break;

        default:
            break;
    }
}

/****************************************************************************************
 * MOVEMENT FUNCTIONS
 ****************************************************************************************/

void stop_bot()
{
    R.detach();
    L.detach();

    // Wait a bit
    delay(1);

    // Middle position
    R.write(90);
    L.write(90);

    R.attach(R_servo);
    L.attach(L_servo);

    dir = STOP;
}

/****************************************************************************************/

void left()
{
    for(int i = start_speed ; i <= max_speed ; i++)
    {
        // To turn faster the opposing wheel is also turning
        R.write(180 - i);
        delay(1);
        
        L.write(180 - i);
        delay(min_delay);
    }

    dir = LEFT;
}


void left_bis()
{
    for(int i = start_speed ; i <= max_speed ; i++)
    {
        L.write(i);
        delay(min_delay);
    }

    dir = LEFT;
}

/****************************************************************************************/

void right()
{
    for(int i = start_speed ; i <= max_speed ; i++)
    {
        // To turn faster the opposing wheel is also turning
        R.write(i);
        delay(1);
        
        L.write(i);
        delay(min_delay);
    }

    dir = RIGHT;
}


void right_bis()
{
    for(int i = start_speed ; i <= max_speed ; i++)
    {
        R.write(180 - i);
        delay(1);
    }

    dir = RIGHT;
}

/****************************************************************************************/

void backwards()
{
    for(int i = start_speed ; i <= max_speed ; i++)
    {
        R.write(i);
        delay(1);
        
        L.write(180 - i);
        delay(min_delay);
    }

    dir = BACKWARDS;
}


/****************************************************************************************/

void forward()
{
    for(int i = start_speed ; i <= max_speed ; i++)
    {
        R.write(180 - i);
        delay(1);
        
        L.write(i);
        delay(min_delay);
    }

    dir = FORWARD;
}

/****************************************************************************************
 * ACTION FUNCTIONS
 ****************************************************************************************/

int check_direction_right()
{
    // if the R optical sensor sees white
    if(analogRead(R_sensor) < optical_threshold)
    {
        // turn to the right with one wheel
        return RIGHT;
    }
    else
    {
        // turn to the left with one wheel
        return LEFT;
    }
}


int check_direction_left()
{
    // if the L optical sensor sees white
    if(analogRead(L_sensor) < optical_threshold)
    {
        // turn to the left with one wheel
        return LEFT;
    }
    else
    {
        // turn to the right with one wheel
        return RIGHT;
    }
}

/****************************************************************************************/

void line_action_right()
{
    if(millis() - start > 60000)
    {
        stop_bot();
    }
    else
    {
        switch(check_direction_right())
        {    
            case RIGHT:
                if(dir != RIGHT)
                {
                    stop_bot();
                    right_bis();
                    delay(100);
                }
                break;
    
            case LEFT:
                if(dir != LEFT)
                {
                    stop_bot();
                    left_bis();
                    delay(400);
                }
                break;
    
            default:
                stop_bot();
                break;
        }
    }
}


void line_action_left()
{
    if(millis() - start > 60000)
    {
        stop_bot();
    }
    else
    {
        switch(check_direction_left())
        {    
            case RIGHT:
                if(dir != RIGHT)
                {
                    stop_bot();
                    right_bis();
                    delay(600);
                }
                break;
    
            case LEFT:
                if(dir != LEFT)
                {
                    stop_bot();
                    left_bis();
                    delay(100);
                }
                break;
    
            default:
                stop_bot();
                break;
        }
    }
}

/****************************************************************************************/

int distance()
{
    // Clear the trigger
    digitalWrite(trigger, LOW);

    // Wait a bit
    delay(2);

    // Trigger
    digitalWrite(trigger, HIGH);

    // For 10ms
    delayMicroseconds(10);

    // Stop
    digitalWrite(trigger, LOW);

    // Compute the distance
    int dist = pulseIn(echo, HIGH) * 0.01723;

    // Return it
    return dist;
}

/****************************************************************************************/

int check_edge()
{  
    if(analogRead(R_sensor) > optical_threshold)
    {
        if(analogRead(L_sensor) > optical_threshold)
        {
            return FORWARD;
        }
        else
        {
            return RIGHT;
        }
    }
    else
    {
        if(analogRead(L_sensor) > optical_threshold)
        {
            return LEFT;
        }
        else
        {
            return BACKWARDS;
        }
    }
}

/****************************************************************************************/

void solo_action()
{
    if(millis() - start > 60000)
    {
        stop_bot();
    }
    else
    {
        // Check if we are near the edge with the optical sensor
        switch(check_edge())
        {
            case FORWARD:
    
                // Turn in cercle while the distance is greater than 40
                if((distance() > 40) && !lock)
                {
                    max_speed = 92;
                    
                    if(!turn_right)
                    {
                        if(dir != LEFT)
                        {
                            stop_bot();
                            left();
                        }
                    }
                    else
                    {
                        if(dir != RIGHT)
                        {
                            stop_bot();
                            right();
                        }
                    }
                }
                else
                {
                    // When close to the cup the robot enter into the lock mode
                    if(distance() == 5)
                    {
                        // Lock mode activated
                        lock = true;
                    }
    
                    // Goes out of the lock mode if the cup is out of the arena aka the robot sees white.
                    if((analogRead(R_sensor) < optical_threshold) || 
                       (analogRead(L_sensor) < optical_threshold))
                    {
                        // Lock mode deactivated
                        lock = false;
                    }

                    max_speed = 120;
    
                    // When we have detected something move towards it
                    if(dir != FORWARD)
                    {
                        stop_bot();
                        forward();
                    }
                }
                
                break;

            case RIGHT:

                max_speed = 120;
                    
                stop_bot();
                backwards();
                delay(2000);
                
                turn_right = true;
                break;

            case LEFT:

                max_speed = 120;
                    
                stop_bot();
                backwards();
                delay(2000);

                turn_right = false;
                break;
    
            case BACKWARDS:

                max_speed = 120;
                    
                stop_bot();
                backwards();
                delay(2000);
                break;
        
            default:
                stop_bot();
                break;
        }
    }   
}

/****************************************************************************************/

void duo_action()
{
    max_speed = 120;

    if(millis() - start > 120000)
    {
        stop_bot();
    }
    else
    {
        if(millis() - start < 118000)
        {
            // Check if we are near the edge with the optical sensor
            switch(check_edge())
            {
                case FORWARD:

                    // Wait for 2.8 secondes at the beginning
                    if(beginning)
                    {
                        stop_bot();
                        delay(2800);

                        beginning = false;
                    }

                    if(dir != FORWARD)
                    {
                        stop_bot();
                        forward();
                        delay(blind_time);
                    }
                    
                    break;
            
                case RIGHT:
                    if(dir != RIGHT)
                    {
                        stop_bot();
                        right();
                        delay(blind_time);
                    }
                    break;
            
                case LEFT:
                    if(dir != LEFT)
                    {
                        stop_bot();
                        left();
                        delay(blind_time);
                    }
                    break;
        
                case BACKWARDS:
                    if(dir != BACKWARDS)
                    {
                        stop_bot();
                        backwards();
                        delay(2000);
                    }
                    break;
            
                default:
                    stop_bot();
                    break;
            }
        }
        else
        {
            if(!finale)
            {
                stop_bot();
                right();
                delay(300);
    
                stop_bot();
                forward();
                delay(1500);

                stop_bot();
            }

            finale = true;
        }
    }
}
