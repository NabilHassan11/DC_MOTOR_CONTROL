///////////////////// Author : Nabil Hassan ///////////////////////////

/* reference 1: 
Modern Control Engineering
Fifth Edition
Katsuhiko Ogata*/

/* reference 2: https://youtu.be/QSIksPKndEs?si=3yDz2XbFMGmFbnLq */
#include <math.h>

// PID Position Controller Parameters
float kp_angle = 0.4;
float ki_angle = 0.0 ;
float kd_angle = 0.02;

// PID Speed Controller Parameters
float kp_speed = 0.3;
float ki_speed = 0.8 ;
float kd_speed = 0.0003;

//low pass filter const
float alpha = 0.1;

const float Pi = 3.1416;

// Integral Anti-Windup back-calculation tracking time constant 

float T_tracking = 0.2;

//time variable 
// unsigned long t ;
unsigned long t_prev = 0;

// PWM and Direction pins
const byte PWMPin = 5;
const byte DirPin1 = 6;
const byte DirPin2 = 7;


// Interrupt Pins
const byte interruptPinA = 2;
const byte interruptPinB = 3;

// Encoder Count
long EncoderCount_prev = 0;
volatile long EncoderCount= 0;
unsigned long count_mod = 0;

// cycle counts
volatile unsigned long count = 0;
unsigned long count_prev = 0;


// Motor Variables
float Theta_curr,Theta_curr_deg,Theta_Desired,Omega_curr, RPM_curr, RPM_Desired, RPM_curr_filtered, RPM_prev_filtered,Theta_curr_filtered,Theta_prev_filtered ;
float Theta_prev = 0;
float RPM_max = 230;
float RPM_min = -230;
float dt;
float dt_angle;
const float Ts = 0.01;
const float Ts_angle = 0.05;

//Controller saturation parameters
float Vmax = 12;
float Vmin = -12;


// Control input U to the system
float V = 0.0;

// error variables
float e, e_prev = 0, inte, inte_prev = 0,e_angle, e_angle_prev,inte_angle, inte_angle_prev = 0;


/* ISR_Encoder Functions */

void ISR_EncoderA(){
  bool PinB = digitalRead(interruptPinB);
  bool PinA = digitalRead(interruptPinA);
  if(PinB == LOW){
    if(PinA == HIGH){
      EncoderCount++;
    }
    else{
      EncoderCount--;
    }
  }
  else{
    if(PinA == HIGH){
      EncoderCount--;
    }
    else{
      EncoderCount++;
    }
  }
}


void ISR_EncoderB(){
  bool PinB = digitalRead(interruptPinA);
  bool PinA = digitalRead(interruptPinB);
  if(PinA == LOW){
    if(PinB == HIGH){
      EncoderCount--;
    }
    else{
      EncoderCount++;
    }
  }
  else{
    if(PinB == HIGH){
      EncoderCount++;
    }
    else{
      EncoderCount--;
    }
  }
}


////////////////////////// Cycle counter repeats every 10ms ///////////////////////////////
ISR(TIMER1_COMPA_vect){
  count++;
}

///////////////////////Motor Driver Function//////////////////////////////

void WriteDriverVoltage(float V, float Vmax) {
  int PWMval = int(255 * abs(V) / Vmax); // this ensures that the output average voltage stays between 0V and 12V

  // this is saturation limit if the value of PWM exceeds 255 it caps it at 255
  if (PWMval > 255) {
    PWMval = 255;
  }

  // if V is greater than zero it spins motor CW or CCW (depends on the wire connection)
  if (V > 0) {
    digitalWrite(DirPin1, HIGH);
    digitalWrite(DirPin2, LOW);
  }

  // this is the opposite of the condition above it spins the motor in the other direction
  else if (V < 0) {
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, HIGH);
  }

  // base case for safety it unplugs the motor if V = 0
  else {
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, LOW);
  }

  // write the output signal on the motor PWM pin
  analogWrite(PWMPin, PWMval);
}

void ReadReference()
{
    if (Serial.available())
    {
        Theta_Desired = Serial.parseFloat();

        while (Serial.available())
        {
            Serial.read();
        }
    }
}

void setup(){

  Serial.begin(115200);
  pinMode(interruptPinA,INPUT_PULLUP);
  pinMode(interruptPinB,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR_EncoderB, CHANGE);

  cli(); // disables global interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1A = 19999; // prescalar = 8 , this means the PID loop runs at 100Hz or 10ms
  TCNT1 = 0;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11);
  TIMSK1 |= (1 << OCIE1A);
  sei(); // enables global interrupts
  RPM_Desired = 0;
  Theta_Desired = 0;
  dt = Ts;
  dt_angle = Ts_angle;

}

void loop(){


  ReadReference(); // gets the reference angle
 
  unsigned long localCount;
  static int positionDivider = 0;

  noInterrupts();
  localCount = count;
  interrupts();


  if(localCount > count_prev){
    // t = millis(); // gets the current time

    long encoder;

    noInterrupts();
    encoder = EncoderCount;
    interrupts();


    //////////////////////////////////////// angle Controller ////////////////////////////////////////////////////
    count_mod = ((encoder % 1496) + 1496) % 1496; // this ensures theta stays between 0 --> 360 deg
    // Theta_curr = (count_mod * 2 * Pi) / 1496.0; // this is in rad

    Theta_curr_deg = count_mod * 360.0 / 1496.0;

    // wrap angle between 0 and 360
    float theta_desired_deg = fmod(Theta_Desired, 360.0f);

    if (theta_desired_deg < 0)
    {
        theta_desired_deg += 360.0f;
    }


    // dt = (t-t_prev)*0.001;// 10ms or f = 100Hz
    
    

    long deltaCount = encoder - EncoderCount_prev; // gets the count difference

    Omega_curr = (deltaCount*2*Pi)/(1496*dt);
    RPM_curr = deltaCount * 60.0 / (1496.0 * dt);

    // Low pass filter calculation
    RPM_curr_filtered = alpha* RPM_curr + (1-alpha)*RPM_prev_filtered;
    // Theta_curr_filtered = Theta_curr_deg;

    

    //////////////////////////////////////// Position Controller ////////////////////////////////////////////////////
    //Desired speed calculation
    if(++positionDivider >= 5)
    {
        positionDivider = 0;
        // e_angle = Theta_Desired - Theta_curr_deg;
        e_angle = theta_desired_deg - Theta_curr_deg; //angle error calculation

        // shortest-path error
        if(e_angle > 180)
            e_angle -= 360;

        if(e_angle < -180)
            e_angle += 360;

        
        // Anti wind-up back calculation for integral part of angle controller
        float RPM_Desired_unsat = kp_angle*e_angle + ki_angle*inte_angle_prev + kd_angle*(e_angle-e_angle_prev)/dt_angle;
        float RPM_Desired_sat = constrain(RPM_Desired_unsat,-230,230);

        // integral angle error calculation
        inte_angle = inte_angle_prev + (dt_angle*(e_angle + e_angle_prev)/2) +(dt_angle/T_tracking)*(RPM_Desired_sat  - RPM_Desired_unsat);

        RPM_Desired = kp_angle*e_angle + ki_angle*inte_angle + kd_angle*(e_angle-e_angle_prev)/dt_angle;
        // Saturation limit Condition
        RPM_Desired = constrain(RPM_Desired, RPM_min, RPM_max);
    }
    
    //////////////////////////////////////// Speed Controller ////////////////////////////////////////////////////
    e = RPM_Desired - RPM_curr_filtered; // Speed Error Calculation

    float V_unsat = kp_speed*e + ki_speed*inte_prev + kd_speed*(e-e_prev)/dt;

    float V_sat = constrain(V_unsat,Vmin,Vmax);
    
    inte = inte_prev + ((dt) * (e + e_prev)/2) +(dt/T_tracking)*(V_sat - V_unsat); // Integral speed Error Calculation
    
    V = kp_speed*e + inte*ki_speed + (kd_speed * (e - e_prev)/(dt)); // Control input calculation

    
    // Saturation limit Condition
    V = constrain(V, Vmin, Vmax);

    

    // Apply desired Voltage to the motor 
    WriteDriverVoltage(V, Vmax);

    //Printing Outputs
    Serial.print( Theta_Desired);
    Serial.print(" \t");
    Serial.print(Theta_curr_deg);
    Serial.print(" \t");
    // Serial.print(V); Serial.print("\t  ");
    Serial.print(e_angle);
    Serial.print(" \t");
    Serial.print(V);
    Serial.println(" ");
    // Serial.print(dt); Serial.println("\t ");
    // Serial.print(int(255 * abs(V) / Vmax));
    // Serial.println("\t  ");


    //updating variables

    // Theta_prev = Theta_curr;
    count_prev = localCount;
    // t_prev = t;
    inte_prev = inte;
    e_prev = e;
    EncoderCount_prev = encoder;
    RPM_prev_filtered = RPM_curr_filtered;
    e_angle_prev = e_angle;
    inte_angle_prev = inte_angle;
    // Theta_prev_filtered = Theta_curr_filtered;


  }
}

