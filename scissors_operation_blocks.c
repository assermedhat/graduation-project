

#include <QTRSensors.h>
#include <PID_v1.h>
#include <SPI.h>
#include <MFRC522.h>

#include <Ultrasonic.h>
Ultrasonic ultrasonic(42, 44);
float distance;


int pos[2];
int prev[2];
int targ[3];
int dir[2];
int path[2];

//box flag
bool box_flag = 0;   // 0 then there is no box, 1 there is box

bool NEW_CARD;
bool done_flag=0; //flags when the robot has done pick up or delivery from/to shelf
bool at_targ=0;   //flags when in target position 

const int trigPin = 42; // Pin connected to Trig pin of the ultrasonic sensor
const int echoPin = 44; // Pin connected to Echo pin of the ultrasonic sensor



//#define MAX 255
const int expectedBytes = 1;
byte receivedBytes[expectedBytes];
String receivedMessage = "";
#define MAX 255
#define MIN 0
#define SVar 0.3
#define spin_delay 1200
#define flag_p 0

#define RST_PIN     8          // Reset pin
#define SS_PIN      9          // Slave select pin
MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance


//limit swtches
int open_S = 39;
int open_S_S  ;

int closed_S = 35;
int closed_S_S ;

int middle_S = 37;
int middle_S_S ;

//gripper limit switch;
int open_G = 31 ;
int open_G_S ;

int closed_G = 33 ;
int closed_G_S  ; 

//Right motor drvier
int LPWM = 7;
int RPWM = 6;
int L_EN = 40;     //40
int R_EN = 38;     //38

//Left motor driver
int LPWM2 = 2;
int RPWM2 = 3;
int L_EN2 = 30;    //32
int R_EN2 = 32;    //30

//Scissors motor driver
int LPWMS = 4;
int RPWMS = 5;
int L_ENS = 34;
int R_ENS = 36;

//Gripper motor driver
int MotorGripper1= 26;
int MotorGripper2= 28;
//int MotorGripperS = 26;

//pump

int pump = 24;


void forward();
void back();
void turnR();
void turnL();
void spinR();
void spinL();
void stop();
void scissorsup();
void scissorsdown();
void scissorsoff();
void gripperon();
void gripperoff();
void gripperback();
void pumpON();
void pumpOFF();
void RFID();
void IR();
int calibration();
void NAV();
void database_comm();
void nos_fat7a_box();
void nos_fat7a_no_box();
void fat7a_box();
void fat7a_no_box();
void cvarduino();
float getDistance();
void fun_scissors_up();
void fun_scissors_down();
void fun_gripper_ext();
void fun_gripper_ret();
void pump_holding();







double Setpoint = 2500;
double Input, Output;

double Kp = 0.017;
double Kd = 0.011;  
double Ki = 0;

//first time flag
bool first_time_cv=1;
//first time database flag
bool first_time_db=1;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

int motorPow = 0;
void setup() {

  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(40, OUTPUT);
  pinMode(38, OUTPUT);


  //Left
  pinMode(3, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(30, OUTPUT);
 
  //Scissors
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(36, OUTPUT);
  pinMode(34, OUTPUT);
  
  
  //Gripper
  pinMode(26, OUTPUT);
  pinMode(28, OUTPUT);
  //pinMode(26, OUTPUT);
  //scissors ls
  pinMode(35, INPUT);
  pinMode(37, INPUT);
  pinMode(39, INPUT);

  //gripper limit switch

  pinMode(33, INPUT);
  pinMode(31, INPUT);

  //pump
  
  pinMode(24,OUTPUT);


  digitalWrite(R_EN, HIGH);  
  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN2, HIGH);  
  digitalWrite(L_EN2, HIGH);
  digitalWrite(R_ENS, HIGH);  
  digitalWrite(L_ENS, HIGH);


  pinMode(open_S, INPUT_PULLUP);
  pinMode(closed_S, INPUT_PULLUP);
  pinMode(middle_S, INPUT_PULLUP);
  pinMode(open_G, INPUT_PULLUP);
  pinMode(closed_G, INPUT_PULLUP);
  pumpON();


  

}


void loop() {
  
 fun_scissors_half_up();
 fun_gripper_ext();
 pump_holding();
 fun_gripper_ret();
 fun_scissors_down();



}


void turnR()
 {
  
  analogWrite(RPWM, MIN);
  analogWrite(LPWM,MIN);

  analogWrite(RPWM2, MAX);
  analogWrite(LPWM2,MIN);

  analogWrite(RPWMS, MIN);
  analogWrite(LPWMS,MIN);

  digitalWrite(MotorGripper1, LOW);
  digitalWrite(MotorGripper2, LOW);
  
   digitalWrite(pump,LOW);
  
}

void turnL()
 {
    analogWrite(RPWM, MAX);
    analogWrite(LPWM,MIN);

    analogWrite(RPWM2, MIN);
    analogWrite(LPWM2,MIN);

    analogWrite(RPWMS, MIN);
    analogWrite(LPWMS,MIN);

    digitalWrite(MotorGripper1, LOW);
    digitalWrite(MotorGripper2, LOW);

     digitalWrite(pump,LOW);

}


void back()
 {
    analogWrite(RPWM, MIN);
    analogWrite(LPWM, MAX);

    analogWrite(RPWM2, MIN);
    analogWrite(LPWM2, MAX);

    analogWrite(RPWMS, MIN);
    analogWrite(LPWMS,MIN);

    digitalWrite(MotorGripper1, LOW);
    digitalWrite(MotorGripper2, LOW);
    
     digitalWrite(pump,LOW);

}

void forward()
 {
  
  analogWrite(RPWM,0.5* MAX);
  analogWrite(LPWM,0.5*MIN);

  analogWrite(RPWM2,0.5* MAX);
  analogWrite(LPWM2,MIN);

  analogWrite(RPWMS, MIN);
  analogWrite(LPWMS,MIN);

  digitalWrite(MotorGripper1, LOW);
  digitalWrite(MotorGripper2, LOW);

  digitalWrite(pump,LOW);
}


void stop()
 {

  analogWrite(RPWM, MIN);
  analogWrite(LPWM,MIN);
  analogWrite(RPWM2, MIN);
  analogWrite(LPWM2,MIN);

  analogWrite(RPWMS, MIN);
  analogWrite(LPWMS,MIN);


  digitalWrite(MotorGripper1, LOW);
  digitalWrite(MotorGripper2, LOW);

  digitalWrite(pump,LOW);
}


void scissorsup()
{
  analogWrite(RPWM, MIN);
  analogWrite(LPWM,MIN);
  analogWrite(RPWM2, MIN);
  analogWrite(LPWM2,MIN);

  analogWrite(RPWMS, MAX);
  analogWrite(LPWMS,MIN);


  digitalWrite(MotorGripper1, LOW);
  digitalWrite(MotorGripper2, LOW);

 digitalWrite(pump,LOW);

}
void scissorsdown()
{
   analogWrite(RPWM, MIN);
  analogWrite(LPWM,MIN);
  analogWrite(RPWM2, MIN);
  analogWrite(LPWM2,MIN);

  analogWrite(RPWMS, MIN);
  analogWrite(LPWMS,MAX);


  digitalWrite(MotorGripper1, LOW);
  digitalWrite(MotorGripper2, LOW);

   digitalWrite(pump,LOW);
}
void scissorsoff()
{
  analogWrite(RPWM, MIN);
  analogWrite(LPWM,MIN);
  analogWrite(RPWM2, MIN);
  analogWrite(LPWM2,MIN);

  analogWrite(RPWMS, MIN);
  analogWrite(LPWMS,MIN);


  digitalWrite(MotorGripper1, LOW);
  digitalWrite(MotorGripper2, LOW);

 digitalWrite(pump,LOW);

}
void gripperon()
{
   analogWrite(RPWM, MIN);
  analogWrite(LPWM,MIN);
  analogWrite(RPWM2, MIN);
  analogWrite(LPWM2,MIN);

  analogWrite(RPWMS, MIN);
  analogWrite(LPWMS,MIN);
  
  
  
  digitalWrite(MotorGripper1, LOW);
  digitalWrite(MotorGripper2, HIGH);
  //analogWrite(MotorGripperS, MAX);

 digitalWrite(pump,LOW);


}
void gripperoff()
{
   analogWrite(RPWM, MIN);
  analogWrite(LPWM, MIN);
  analogWrite(RPWM2, MIN);
  analogWrite(LPWM2, MIN);

  analogWrite(RPWMS, MIN);
  analogWrite(LPWMS, MIN);
  
  
  //gripper

  digitalWrite(MotorGripper1, LOW);
  digitalWrite(MotorGripper2, LOW);
  //analogWrite(MotorScissorsS, MAX);

  digitalWrite(pump,LOW);


}
void gripperback()
{
   analogWrite(RPWM, MIN);
  analogWrite(LPWM, MIN);
  analogWrite(RPWM2, MIN);
  analogWrite(LPWM2, MIN);

  analogWrite(RPWMS, MIN);
  analogWrite(LPWMS, MIN);
  
  
  //gripper

  digitalWrite(MotorGripper1, HIGH);
  digitalWrite(MotorGripper2, LOW);
  //analogWrite(MotorScissorsS, MAX);

  digitalWrite(pump,LOW);


}
void spinR()
 {
  analogWrite(RPWM, MIN);
  analogWrite(LPWM, MAX);

  analogWrite(RPWM2, MAX);
  analogWrite(LPWM2,MIN);

  analogWrite(RPWMS, MIN);
  analogWrite(LPWMS,MIN);

  digitalWrite(MotorGripper1, LOW);
  digitalWrite(MotorGripper2, LOW);

  digitalWrite(pump,LOW);

}

void spinL()
 {
  analogWrite(RPWM, MAX);
  analogWrite(LPWM, MIN);

  analogWrite(RPWM2, MIN);
  analogWrite(LPWM2, MAX);

  analogWrite(RPWMS, MIN);
  analogWrite(LPWMS,MIN);

  digitalWrite(MotorGripper1, LOW);
  digitalWrite(MotorGripper2, LOW);

   digitalWrite(pump,LOW);
}

void pumpON()
{
  analogWrite(RPWM, MIN);
  analogWrite(LPWM, MIN);

  analogWrite(RPWM2, MIN);
  analogWrite(LPWM2, MIN);

  analogWrite(RPWMS, MIN);
  analogWrite(LPWMS,MIN);

  digitalWrite(MotorGripper1, LOW);
  digitalWrite(MotorGripper2, LOW);

  digitalWrite(pump,HIGH);


}
void pumpOFF()
{
  analogWrite(RPWM, MIN);
  analogWrite(LPWM, MIN);

  analogWrite(RPWM2, MIN);
  analogWrite(LPWM2, MIN);

  analogWrite(RPWMS, MIN);
  analogWrite(LPWMS,MIN);

  digitalWrite(MotorGripper1, LOW);
  digitalWrite(MotorGripper2, LOW);

  digitalWrite(pump,LOW);


}

  



// void nos_fat7a_box()
// {
  
//   middle_S_S = digitalRead(middle_S);
//   open_S_S = digitalRead(open_S);
//   closed_S_S = digitalRead(closed_S);

//   open_G_S = digitalRead(open_G);
//   closed_G_S = digitalRead(closed_G);


//   scissorsup();


//   while (middle_S_S == HIGH) {
//     middle_S_S = digitalRead(middle_S);
//   }


//   scissorsoff();
//   delay(1000);

//   gripperon();

//   while (open_G_S == HIGH) {
//     open_G_S = digitalRead(open_G);
//   }

//   gripperoff();
//   delay(1000); 

//   pumpON();
//   delay(3000);
//   pumpOFF();
//   delay(1000);

//   gripperback();

//   while (closed_G_S == HIGH) {
//     closed_G_S = digitalRead(closed_G);
//   }

//   gripperoff();
//   delay(1000); 

  
//   scissorsdown();

//   while (closed_S_S == HIGH) {
//   closed_S_S = digitalRead(closed_S);
//   }

  
//   scissorsoff();
//   box_flag=1;
// }


// void fat7a_box()
// {

//   // middle_S_S = 1;
//   // open_S_S = 1;
//   // closed_S_S = 1;

//   // open_G_S = 1;
//   // closed_G_S = 1;

//   middle_S_S = digitalRead(middle_S);
//   open_S_S = digitalRead(open_S);
//   closed_S_S = digitalRead(closed_S);

//   open_G_S = digitalRead(open_G);
//   closed_G_S = digitalRead(closed_G);


//   scissorsup();


//   while (open_S_S == HIGH ) {
//     open_S_S = digitalRead(open_S);
//   }


//   scissorsoff();

//   delay(1000);

//   gripperon();

//   while (open_G_S == HIGH) {
//     open_G_S = digitalRead(open_G);
//   }

//   gripperoff();
//   delay(1000); 

//   pumpON();
//   delay(3000);

//   pumpOFF();
//   delay(1000);

//   gripperback();
  

//   while (closed_G_S == HIGH) {
//     closed_G_S = digitalRead(closed_G);
//   }

//   gripperoff();
//   delay(1000); 

  
//   scissorsdown();

//   while (closed_S_S == HIGH) {
//   closed_S_S = digitalRead(closed_S);
//   }

  
//   scissorsoff();
//   box_flag=1;

// }

// void nos_fat7a_no_box()
// {
  
//   middle_S_S = digitalRead(middle_S);
//   open_S_S = digitalRead(open_S);
//   closed_S_S = digitalRead(closed_S);

//   open_G_S = digitalRead(open_G);
//   closed_G_S = digitalRead(closed_G);


//   scissorsup();


//   while (middle_S_S == HIGH) {
//     middle_S_S = digitalRead(middle_S);
//   }


//   scissorsoff();
//   delay(1000);

//   gripperon();

//   while (open_G_S == HIGH) {
//     open_G_S = digitalRead(open_G);
//   }

//   gripperoff();
//   delay(1000); 

//   gripperback();

//   while (closed_G_S == HIGH) {
//     closed_G_S = digitalRead(closed_G);
//   }

//   gripperoff();
//   delay(1000); 

  
//   scissorsdown();

//   while (closed_S_S == HIGH) {
//   closed_S_S = digitalRead(closed_S);
//   }

  
//   scissorsoff();
//   box_flag=0;
// }


// void fat7a_no_box()
// {
//   // middle_S_S = digitalRead(middle_S);
//   open_S_S = digitalRead(open_S);
//   closed_S_S = digitalRead(closed_S);

//   open_G_S = digitalRead(open_G);
//   closed_G_S = digitalRead(closed_G);


//   scissorsup();


//   while (open_S_S == HIGH) {
//     open_S_S = digitalRead(open_S);
//   }


//   scissorsoff();
//   delay(1000);

//   gripperon();

//   while (open_G_S == HIGH) {
//     open_G_S = digitalRead(open_G);
//   }

//   gripperoff();

//   delay(1000);

//   gripperback();

//   while (closed_G_S == HIGH) {
//     closed_G_S = digitalRead(closed_G);
//   }

//   gripperoff();
//   delay(1000); 

  
//   scissorsdown();

//   while (closed_S_S == HIGH) {
//   closed_S_S = digitalRead(closed_S);
//   }

  
//   scissorsoff();
//   box_flag=0;

// }



void fun_scissors_up()
{
  middle_S_S = digitalRead(middle_S);
  open_S_S = digitalRead(open_S);
  closed_S_S = digitalRead(closed_S);

  while(open_S_S == 1)
  {
    open_S_S = digitalRead(open_S);
    scissorsup();
  }

  scissorsoff();

}

void fun_scissors_half_up()
{
  middle_S_S = digitalRead(middle_S);
  open_S_S = digitalRead(open_S);
  closed_S_S = digitalRead(closed_S);

  while(middle_S_S == 1)
  {
    middle_S_S = digitalRead(middle_S);
    scissorsup();
  }

  scissorsoff();

}



void fun_scissors_down()
{
  middle_S_S = digitalRead(middle_S);
  open_S_S = digitalRead(open_S);
  closed_S_S = digitalRead(closed_S);

  while(closed_S_S == 1)
  {
    scissorsdown();
    closed_S_S = digitalRead(closed_S);

  }

  scissorsoff();

}

void fun_gripper_ext()
{
  open_G_S = digitalRead(open_G);
  closed_G_S = digitalRead(closed_G);

  while(open_G_S == 1)
  {
    open_G_S = digitalRead(open_G);
    gripperon();
  }

  gripperoff();

}

void fun_gripper_ret()
{
  open_G_S = digitalRead(open_G);
  closed_G_S = digitalRead(closed_G);

  while(closed_G_S == 1)
  {
   closed_G_S = digitalRead(closed_G);
   gripperback();
  }

  gripperoff();

}

void pump_holding()
{
  pumpON();
  _delay_ms(3000);
  pumpOFF();
  _delay_ms(1000);

}

