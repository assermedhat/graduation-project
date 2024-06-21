#include <QTRSensors.h>
#include <PID_v1.h>
// #include <L298NX2.h>
#include <SPI.h>
#include <MFRC522.h>

//#define MAX 255
float getDistance();
const int expectedBytes = 1;
byte receivedBytes[expectedBytes];
String receivedMessage = "";
#define MAX 255
#define MIN 0
#define SVar 0.3

#define RST_PIN     8          // Reset pin
#define SS_PIN      9          // Slave select pin
MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance


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
int pump1 = 22;
int pump2 = 24;


void forward();
void back();
void turnR();
void turnL();
void spin();
void stop();
void scissorsup();
void scissorsdown();
void gripperON();
void gripperOFF();
void pumpON();
void RFID();
bool IR();
int calibration();

double Setpoint = 2500;
double Input, Output;

double Kp = 0.017;
double Kd = 0.011;  
double Ki = 0;

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

  //pump
  pinMode(22,OUTPUT);
  pinMode(24,OUTPUT);


  digitalWrite(R_EN, HIGH);  
  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN2, HIGH);  
  digitalWrite(L_EN2, HIGH);
  digitalWrite(R_ENS, HIGH);  
  digitalWrite(L_ENS, HIGH);





  if(calibration()==1){
    Serial.println("calibrated");
  }

  
  //RFID start
  
  SPI.begin();                  // Init SPI bus
  mfrc522.PCD_Init();           // Init MFRC522
  Serial.println("RFID reader initialized.");
  //end
}


void loop() {
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
  
  digitalWrite(pump1 ,LOW);
  digitalWrite(pump2 ,LOW);
  
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

    digitalWrite(pump1 ,LOW);
    digitalWrite(pump2 ,LOW);

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
    
    digitalWrite(pump1 ,LOW);
    digitalWrite(pump2 ,LOW);

}

void forward()
 {
  
  analogWrite(RPWM, MAX);
  analogWrite(LPWM,MIN);

  analogWrite(RPWM2, MAX);
  analogWrite(LPWM2,MIN);

  analogWrite(RPWMS, MIN);
  analogWrite(LPWMS,MIN);

  digitalWrite(MotorGripper1, LOW);
  digitalWrite(MotorGripper2, LOW);

  digitalWrite(pump1 ,LOW);
  digitalWrite(pump2 ,LOW);
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

  digitalWrite(pump1 ,LOW);
  digitalWrite(pump2 ,LOW);
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

  digitalWrite(pump1 ,LOW);
  digitalWrite(pump2 ,LOW);

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

  digitalWrite(pump1 ,LOW);
  digitalWrite(pump2 ,LOW);
}

void gripperON()
{
   analogWrite(RPWM, MIN);
  analogWrite(LPWM,MIN);
  analogWrite(RPWM2, MIN);
  analogWrite(LPWM2,MIN);

  analogWrite(RPWMS, MIN);
  analogWrite(LPWMS,MIN);
  
  
  
  digitalWrite(MotorGripper1, HIGH);
  digitalWrite(MotorGripper2, LOW);
  //analogWrite(MotorGripperS, MAX);


  digitalWrite(pump1 ,LOW);
  digitalWrite(pump2 ,LOW);


}
void gripperOFF()
{
   analogWrite(RPWM, MIN);
  analogWrite(LPWM, MIN);
  analogWrite(RPWM2, MIN);
  analogWrite(LPWM2, MIN);

  analogWrite(RPWMS, MIN);
  analogWrite(LPWMS, MIN);
  
  
  //gripper

  digitalWrite(MotorGripper1, LOW);
  digitalWrite(MotorGripper2, HIGH);
  //analogWrite(MotorScissorsS, MAX);

  digitalWrite(pump1 ,LOW);
  digitalWrite(pump2 ,LOW);


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

  digitalWrite(pump1 ,LOW);
  digitalWrite(pump2 ,LOW);

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

  digitalWrite(pump1 ,LOW);
  digitalWrite(pump2 ,LOW);
}

void pumpON()
{
  analogWrite(RPWM, MAX);
  analogWrite(LPWM, MIN);

  analogWrite(RPWM2, MIN);
  analogWrite(LPWM2, MAX);

  analogWrite(RPWMS, MIN);
  analogWrite(LPWMS,MIN);

  digitalWrite(MotorGripper1, LOW);
  digitalWrite(MotorGripper2, LOW);

  digitalWrite(pump1 ,HIGH);
  digitalWrite(pump2 ,LOW);


}
void RFID()
{
   // Look for new cards
  if ( ! mfrc522.PICC_IsNewCardPresent() || ! mfrc522.PICC_ReadCardSerial() ) {
    delay(50);
    return;
  }

  // Show UID on serial monitor
  Serial.print("UID tag :");
  String content= "";
  byte letter;
  for (byte i = 0; i < mfrc522.uid.size; i++) {
     Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
     Serial.print(mfrc522.uid.uidByte[i], HEX);
     content.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
     content.concat(String(mfrc522.uid.uidByte[i], HEX));
  }
  Serial.println();
  Serial.print("Message : ");
  content.toUpperCase();
  if (content.substring(1) == "4A 97 A2 1E") { // Change to match your UID
    Serial.println("Authorized access");
    // Add your actions here for authorized access
  }
  else {
    Serial.println("Access denied");
  }

  delay(500);

}

bool IR()
{ Serial.println(sensorValues[0]);
        Input = qtr.readLineBlack(sensorValues);
        Serial.println(Input);

        //getDistance();
        //delay(10);
        myPID.Compute();
        //getDistance();  
        Serial.println(Output);
        

        //straight
        if ((Output < 4) & (Output > -4)) {


        forward();

        }

        //LEFT
        else if (Output > 4) {
          
          turnR();
          // motorBD.setSpeed((195) + abs(Output));
          // motorBD.forward();
          // motorAC.setSpeed(30 + abs(Output));
          // motorAC.backward();

        }

        //turn RIGHT
        else if (Output < -4) {

          turnL();
          // motorAC.setSpeed((195) + abs(Output));
          // motorAC.forward();
          // motorBD.setSpeed(30 + abs(Output));
          // motorBD.backward();
        }
        return 1;
        }
  
int calibration(){

     // configure the sensors of IR
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(5);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 53, 49, 47, 45, 43, 41 }, SensorCount);
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
  Serial.println();

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
// float getDistance() {
//   digitalWrite(trig, LOW);
//   delayMicroseconds(5);
//   digitalWrite(trig, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(trig, LOW);
//   t = pulseIn(echo, HIGH);
//   distance = t / 57;
//   Serial.println("us");
//   Serial.println(distance);
//   return distance;
// }
