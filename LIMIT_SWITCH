#define MAX 255
#define MIN 0
int fin_flag=0;
unsigned long previousMillis = 0;
const long interval = 1000;
//Scissors limit switch
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


void scissorsup();
void scissoroff();
void scissorsdown();
void gripperON();
void gripperoff();
void gripperback();
void pumpON();
void pumpOFF();
void nos_fat7a_box();
void nos_fat7a_no_box();
void fat7a_box();
void fat7a_no_box();



void setup() {

  Serial.begin(9600);
  // put your setup code here, to run once:
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

  //pump
  
  pinMode(24,OUTPUT);


  //scissors limit switch

  pinMode(35, INPUT);
  pinMode(37, INPUT);
  pinMode(39, INPUT);

  //gripper limit switch

  pinMode(29, INPUT);
  pinMode(31, INPUT);


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

  fat7a_box();
  
   

}





void nos_fat7a_box()
{
  
  middle_S_S = digitalRead(middle_S);
  open_S_S = digitalRead(open_S);
  closed_S_S = digitalRead(closed_S);

  open_G_S = digitalRead(open_G);
  closed_G_S = digitalRead(closed_G);


  scissorsup();


  while (middle_S_S == HIGH) {
    middle_S_S = digitalRead(middle_S);
  }


  scissorsoff();
  delay(1000);

  gripperON();

  while (open_G_S == HIGH) {
    open_G_S = digitalRead(open_G);
  }

  gripperoff();
  delay(1000); 

  pumpON();
  delay(3000);
  pumpOFF();
  delay(1000);

  gripperback();

  while (closed_G_S == HIGH) {
    closed_G_S = digitalRead(closed_G);
  }

  gripperoff();
  delay(1000); 

  
  scissorsdown();

  while (closed_S_S == HIGH) {
  closed_S_S = digitalRead(closed_S);
  }

  
  scissorsoff();
}


void fat7a_box()
{
  middle_S_S = digitalRead(middle_S);
  open_S_S = digitalRead(open_S);
  closed_S_S = digitalRead(closed_S);

  open_G_S = digitalRead(open_G);
  closed_G_S = digitalRead(closed_G);


  scissorsup();


  while (open_S_S == HIGH ) {
    open_S_S = digitalRead(open_S);
  }


  scissorsoff();
  delay(1000);

  gripperON();

  while (open_G_S == HIGH) {
    open_G_S = digitalRead(open_G);
  }

  gripperoff();
  delay(1000); 

  pumpON();
  delay(3000);
  pumpOFF();
  delay(1000);

  gripperback();

  while (closed_G_S == HIGH) {
    closed_G_S = digitalRead(closed_G);
  }

  gripperoff();
  delay(1000); 

  
  scissorsdown();

  while (closed_S_S == HIGH) {
  closed_S_S = digitalRead(closed_S);
  }

  
  scissorsoff();

}

void nos_fat7a_no_box()
{
  
  middle_S_S = digitalRead(middle_S);
  open_S_S = digitalRead(open_S);
  closed_S_S = digitalRead(closed_S);

  open_G_S = digitalRead(open_G);
  closed_G_S = digitalRead(closed_G);


  scissorsup();


  while (middle_S_S == HIGH) {
    middle_S_S = digitalRead(middle_S);
  }


  scissorsoff();
  delay(1000);

  gripperON();

  while (open_G_S == HIGH) {
    open_G_S = digitalRead(open_G);
  }

  gripperoff();
  delay(1000); 

  gripperback();

  while (closed_G_S == HIGH) {
    closed_G_S = digitalRead(closed_G);
  }

  gripperoff();
  delay(1000); 

  
  scissorsdown();

  while (closed_S_S == HIGH) {
  closed_S_S = digitalRead(closed_S);
  }

  
  scissorsoff();
}


void fat7a_no_box()
{
  middle_S_S = digitalRead(middle_S);
  open_S_S = digitalRead(open_S);
  closed_S_S = digitalRead(closed_S);

  open_G_S = digitalRead(open_G);
  closed_G_S = digitalRead(closed_G);


  scissorsup();


  while (open_S_S == HIGH) {
    open_S_S = digitalRead(open_S);
  }


  scissorsoff();
  delay(1000);

  gripperON();

  while (open_G_S == HIGH) {
    open_G_S = digitalRead(open_G);
  }

  gripperoff();

  delay(1000);

  gripperback();

  while (closed_G_S == HIGH) {
    closed_G_S = digitalRead(closed_G);
  }

  gripperoff();
  delay(1000); 

  
  scissorsdown();

  while (closed_S_S == HIGH) {
  closed_S_S = digitalRead(closed_S);
  }

  
  scissorsoff();

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

void gripperON()
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
  analogWrite(LPWM,MIN);
  analogWrite(RPWM2, MIN);
  analogWrite(LPWM2,MIN);

  analogWrite(RPWMS, MIN);
  analogWrite(LPWMS,MIN);
  
  
  
  digitalWrite(MotorGripper1, LOW);
  digitalWrite(MotorGripper2, LOW);
  //analogWrite(MotorGripperS, MAX);

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


  fin_flag=1;
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

  fin_flag=0;
}
