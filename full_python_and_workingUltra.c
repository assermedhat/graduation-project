

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
int done_flag=0; //flags when the robot has done pick up or delivery from/to shelf
int at_target=0;   //flags when in target position 

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
void scissoroff();
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
void fun_scissors_up();
void fun_scissors_half_up();
void fun_scissors_down();
void fun_gripper_ext();
void fun_gripper_ret();
void pump_holding();
void cvarduino();
float getDistance();
void hold_full();
void hold_half();
void leave_full();
void leave_half();
void Out_of_Shelves();  


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


  if(calibration()==1){
    // Serial.println("calibrated");
  }

  
  //RFID start
  
  SPI.begin();                  // Init SPI bus
  mfrc522.PCD_Init();           // Init MFRC522
  // Serial.println("RFID reader initialized.");
  //end

  // targ[]={4,2,0};
  // targ[0]=4;
  // targ[1]=2;
  // targ[3]=0;
  // pos[]={4,1};
  pos[0]=2;
  pos[1]=1;
  
  targ[0]=3;
  targ[1]=2;

  distance = 6.0;


}


void loop() {

  Serial.println("done_flag= ");
  Serial.println(done_flag);
  Serial.println("at_target= ");
  Serial.println(at_target);

  if((pos[0]==2)&&(pos[1]==1) && (box_flag==0)) {
          if(first_time_cv==1){
            Serial.write('y');
            first_time_cv=0;
            first_time_db=1;
          }
          while(distance>=5.0){
            distance = getDistance();
            RFID();
            byte x= Serial.read();
            if(x=='r'){
              turnR();
            }
            else if(x=='l'){
              turnL();

            }
            else if(x=='c'){
              forward();
            }
          }
          
          hold_half();
          
    }


  // else if((pos[0]==2)&&(pos[1]==1) && (box_flag==1)){
  //   if(first_time_db==1){
  //     Serial.write('d');
  //     first_time_db=0;
  //     first_time_cv=1;

  //   }
  //     database_comm();
  // }
  
  // Serial.println("target: ");
  // Serial.println(targ[0]);
  // Serial.println("zzzzzzzzzzzzzzzzzzz");
  // Serial.println(targ[1]);
  
   
  IR();
  //implementing the RFID
  RFID();

  //implementing the navigation
  NAV();

  if ((done_flag == 1) && (at_target == 1))   //moving out of place after pickup or delivery
  {
    Out_of_Shelves();  
  }


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
    analogWrite(LPWM, 0.5*MAX);

    analogWrite(RPWM2, MIN);
    analogWrite(LPWM2, 0.5*MAX);

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
void RFID()
{
  if ( ! mfrc522.PICC_IsNewCardPresent() || ! mfrc522.PICC_ReadCardSerial() ) {
  delay(50);
  return;
  }
  prev[0]=pos[0];
  prev[1]=pos[1];

  NEW_CARD=1;

  // Show UID on serial monitor
  // Serial.print("UID tag :");
  String content= "";
  byte letter;
  for (byte i = 0; i < mfrc522.uid.size; i++) {
      // Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
      // Serial.print(mfrc522.uid.uidByte[i], HEX);
      content.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
      content.concat(String(mfrc522.uid.uidByte[i], HEX));
  }
  // Serial.println();
  // Serial.print("Message : ");
  content.toUpperCase();


  if (content.substring(1) == "5D 97 58 89") { // Change to match your UID
    pos[0]=1;
    pos[1]=2;
    
    // Serial.println("pos = ");
    // Serial.print(pos[0]);
    // Serial.print(",");
    // Serial.print(pos[1]);
    // Serial.println();


    // Add your actions here for authorized access
  }
  else if (content.substring(1) == "4D 12 77 89"){
    pos[0]=2;
    pos[1]=1;

    // Serial.println("pos = ");
    // Serial.print(pos[0]);
    // Serial.print(",");
    // Serial.print(pos[1]);
    // Serial.println();
  }

  else if (content.substring(1) == "4D FC 54 89"){
    pos[0]=2;
    pos[1]=2;

    // Serial.println("pos = ");
    // Serial.print(pos[0]);
    // Serial.print(",");
    // Serial.print(pos[1]);
    // Serial.println();
  }

  else if (content.substring(1) == "DD F4 79 89"){
    pos[0]=2;
    pos[1]=3;

    // Serial.println("pos = ");
    // Serial.print(pos[0]);
    // Serial.print(",");
    // Serial.print(pos[1]);
    // Serial.println();
  }

  else if (content.substring(1) == "6D 2E 7D 89"){
    pos[0]=3;
    pos[1]=1;

    // Serial.println("pos = ");
    // Serial.print(pos[0]);
    // Serial.print(",");
    // Serial.print(pos[1]);
    // Serial.println();
  }

  else if (content.substring(1) == "6D 75 75 89"){
    pos[0]=3;
    pos[1]=2;

    // Serial.println("pos = ");
    // Serial.print(pos[0]);
    // Serial.print(",");
    // Serial.print(pos[1]);
    // Serial.println();
  }

  else if (content.substring(1) == "1D CA 76 89"){
    pos[0]=3;
    pos[1]=3;

    // Serial.println("pos = ");
    // Serial.print(pos[0]);
    // Serial.print(",");
    // Serial.print(pos[1]);
    // Serial.println();
  }

  else if (content.substring(1) == "3D BF 79 89"){
    pos[0]=4;
    pos[1]=1;

    // Serial.println("pos = ");
    // Serial.print(pos[0]);
    // Serial.print(",");
    // Serial.print(pos[1]);
    // Serial.println();
  }

  else if (content.substring(1) == "DD 5E 59 89"){
    pos[0]=4;
    pos[1]=2;

    // Serial.println("pos = ");
    // Serial.print(pos[0]);
    // Serial.print(",");
    // Serial.print(pos[1]);
    // Serial.println();
  }

  else if (content.substring(1) == "ED F5 57 89"){
    pos[0]=4;
    pos[1]=3;

    // Serial.println("pos = ");
    // Serial.print(pos[0]);
    // Serial.print(",");
    // Serial.print(pos[1]);
    // Serial.println();
  }

  else if (content.substring(1) == "90 A4 EA 32"){
    pos[0]=5;
    pos[1]=2;

    // Serial.println("pos = ");
    // Serial.print(pos[0]);
    // Serial.print(",");
    // Serial.print(pos[1]);
    // Serial.println();
  }

  dir[0]=pos[0]-prev[0];
  dir[1]=pos[1]-prev[1];

  path[0]=targ[0]-pos[0];
  path[1]=targ[1]-pos[1];



  // delay(500);

}

void IR()
{  

  // Read the sensor values and print the first sensor value for debugging
    qtr.read(sensorValues);
    // Serial.println(sensorValues[0]);

    // Read the line position using the sensors
    Input = qtr.readLineBlack(sensorValues);
    // Serial.println(Input);

    // Compute the PID output
    myPID.Compute();
    // Serial.println(Output);

    // Check if all sensors read a low value indicating no line detected
    bool noLineDetected = true;
    for (int i = 0; i < SensorCount; i++) {
        if (sensorValues[i] > 200) { // Adjust the threshold value as needed
            noLineDetected = false;
            break;
        }
    }

    // If no line is detected, move forward
    if (noLineDetected) {
      
        // Serial.println("No line detected, moving forward.");
        forward();
    }
    // Otherwise, adjust the direction based on the PID output
    else {
        // Go straight if the output is within a small range around zero
        if (Output < 4 && Output > -4) {
            forward();
        }
        // Turn right if the output is greater than 4
        else if (Output > 4) {
            turnR();
        }
        // Turn left if the output is less than -4
        else if (Output < -4) {
            turnL();
        }
    }      
    
}
  
int calibration(){

     // configure the sensors of IR
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(5);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 27, 49, 47, 45, 43, 41 }, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
  qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  //for (uint8_t i = 0; i < SensorCount; i++) {
    // Serial.print(qtr.calibrationOn.minimum[i]);
    // Serial.print(' ');
  //}
  // Serial.println();

  //print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++) {
    // Serial.print(qtr.calibrationOn.maximum[i]);
    // // Serial.print(' ');
  }
  // Serial.println();
  // Serial.println();
  delay(1000);
  //END of IR configuration
  return 1;
}

void Out_of_Shelves()
{
   Serial.println("inn moving outtttt");

      back();
      _delay_ms(spin_delay);

      if (path[0] > 0)    // rotate left 90 degree
      {
        spinL();
        _delay_ms(spin_delay);

      }

      else if (path[0] < 0)      //rotate right 90 degree
      {
        spinR();
        _delay_ms(spin_delay);
      }

      else if (path[0] == 0)   //for cases where target is on the same x-line as position
      {
        if (pos[0]==2 || pos[0]==3)
        {
          spinR();
        _delay_ms(spin_delay);
        }

        else if (pos[0] == 4)
        {
          spinL();
        _delay_ms(spin_delay);
        }


      }

    
      forward();
      _delay_ms(spin_delay);

      at_target=0;


}

void NAV(){
  // Serial.println("NAV1");  

  if (NEW_CARD==0 ) {
    delay(50);
    return;
  }

  

  // if (pos[0]%4 == 1)   //at nodes looking in X-direction
  // {
  //   if ((dir[0]>0 && path[1]>0) || (dir[0]<0 && path[1]<0))
  //   {
  //     spinL();
  //     _delay_ms(500);
  //     forward();
  //     _delay_ms(500);
  //     Serial.println("148");
  //   }

  //   else if ((dir[0]>0 && path[1]<0) || (dir[0]<0 && path[1]>0))
  //   {
  //     spinR();
  //     _delay_ms(500);
  //     forward();
  //     _delay_ms(500);
  //     Serial.println("157");
  //   }
  // }

  // if (pos[0]%4 == 1)   //at nodes looking in Y-direction and target NOT at y=2
  // {
  //   if ((dir[1]>0 && path[1]>0) || (dir[1]<0 && path[1]<0))
  //   {
  //     forward();
  //     _delay_ms(500);
  //     Serial.println("167");
  //   }
  // }

  // if (pos[0] == 1)   //at node 1 looking in Y-direction and target at y=2
  // {
  //   if ((dir[1]>0 && path[1]==0)) //facing +ve Y 
  //   {
  //     spinR();
  //     _delay_ms(500);
  //     forward();
  //     _delay_ms(500);
  //     Serial.println("179");
  //   }
    
  //   if ((dir[1]<0 && path[1]==0)) //facing -ve Y
  //   {
  //     soinL();
  //     _delay_ms(500);
  //     forward();
  //     _delay_ms(500);
  //     Serial.println("188");
  //   }
  // }

  // if (pos[0] == 5)   //at node 5 looking in Y-direction and target at y=2
  // {
  //   if ((dir[1]>0 && path[1]==0)) //facing +ve Y
  //   {
  //     spinL();
  //     _delay_ms(500);
  //     forward();
  //     _delay_ms(500);
  //     Serial.println("200");
  //   }

  //   else if ((dir[1]<0 && path[1]==0)) //facing -ve Y
  //   {
  //     spinR();
  //     _delay_ms(500);
  //     forward();
  //     _delay_ms(500);
  //     Serial.println("209");
  //   }
  // }

  if ((pos[0]%4==1) && (dir[0]==1 && dir[1]==1))
  {
      if (targ[1]==2)
      {
        spinL();
        _delay_ms(spin_delay);
        forward();
        _delay_ms(spin_delay);
      }

      if (targ[1]!=2)
      {
        forward();
        _delay_ms(spin_delay);
      }
  }

  if ((pos[0]%4==1) && (dir[0]==1 && dir[1]==-1))
  {
    if (targ[1]==2)
    {
      spinR();
      _delay_ms(spin_delay);
      forward();
      _delay_ms(spin_delay);
    }

    if (targ[1]!=2)
    {
      forward();
      _delay_ms(spin_delay);      
    } 

  }

  if ((pos[0]%4==1) && (dir[0]==-1 && dir[1]==1))
  {
    if (targ[1]==2)
    {
      spinR();
      _delay_ms(spin_delay);
      forward();
      _delay_ms(spin_delay);
    }

    if (targ[1]!=2)
    {
      forward();
      _delay_ms(spin_delay);      
    }     
  }

  if ((pos[0]%4==1) && (dir[0]==-1 && dir[1]==-1))
  {
      if (targ[1]==2)
      {
        spinL();
        _delay_ms(spin_delay);
        forward();
        _delay_ms(spin_delay);
      }

      if (targ[1]!=2)
      {
        forward();
        _delay_ms(spin_delay);
      }    
  }


  if ((path[0] == 0) && (path[1] == 0))  // rotating to face shelves at target point
  {
    if (dir[0] > 0)
    {
      spinR();
      Serial.println("at target");
      _delay_ms(spin_delay);
      forward();
      _delay_ms(spin_delay);
      leave_half();
      delay(spin_delay);
      at_target=1;
    }

    else if (dir[0] < 0)
    {
      spinL();
      Serial.println("at target");
      _delay_ms(spin_delay);
      forward();
      _delay_ms(spin_delay);
      leave_half();
      delay(spin_delay);
      at_target=1;
    }

  }
  NEW_CARD=0;
  
}
void database_comm(){
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    int xpos = data.substring(0, data.indexOf(',')).toInt();
    data.remove(0, data.indexOf(',') + 1);
    int ypos = data.substring(0, data.indexOf(',')).toInt();
    data.remove(0, data.indexOf(',') + 1);
    int zpos = data.toInt();
    
 
    targ[0]=xpos;
    targ[1]=ypos;
    targ[2]=zpos;
  }
}

float getDistance() {
  // Clear the trigPin by setting it LOW
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Trigger the sensor by setting the trigPin HIGH for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin, pulseIn() returns the duration (time) of the pulse in microseconds
  float duration = pulseIn(echoPin, HIGH);

  // Calculate the distance
  // Speed of sound at sea level = 343 m/s = 0.0343 cm/us
  // Distance = (duration / 2) * speed of sound in cm/us
  float distance = (duration / 2) * 0.0343;

  return distance;
}



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

void hold_full()
{
  fun_scissors_up();
  fun_gripper_ext();
  pump_holding();
  fun_gripper_ret();
  fun_scissors_down();

  box_flag=1;
          
}


void hold_half()
{
  fun_scissors_half_up();
  _delay_ms(50);
  fun_gripper_ext();
  _delay_ms(50);
  pump_holding();
  _delay_ms(50);
  fun_gripper_ret();
  _delay_ms(50);
  fun_scissors_down();
  _delay_ms(50);

  box_flag=1;
          
}


void leave_full()
{
  fun_scissors_up();
  fun_gripper_ext();
  fun_gripper_ret();
  fun_scissors_down();

  box_flag=0;
  done_flag=1;
          
}

void leave_half()
{
  fun_scissors_half_up();
  fun_gripper_ext();
  fun_gripper_ret();
  fun_scissors_down();

  box_flag=0;
  done_flag=1;

          
}
