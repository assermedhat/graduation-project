// int pos[2];
// int prev[2];
// int targ[3]; //firebase x,y,z



int pos[]={5,2};
int  prev[]={4,1};
int  targ[]={4,2,0};

int dir[2]; //position-previous
int path[2]; //target-position
int distance;       //ultrasonic distance

bool done_flag=0; //flags when the robot has done pick up or delivery from/to shelf
bool at_targ=0;   //flags when in target position 

void RFID();
void NAV();
void FORWARD();
void BACKWARD();
void SPIN_L();
void SPIN_R();




void setup() {
  Serial.begin(9600);

  dir[0]= pos[0]-prev[0];
  dir[1]= pos[1]-prev[1];

  path[0]=targ[0]-pos[0];
  path[1]=targ[1]-pos[1];

  Serial.print(dir[0]);
  Serial.print(",");
  Serial.println(dir[1]);

  Serial.print(path[0]);
  Serial.print(",");
  Serial.println(path[1]);
}

void loop() {
  NAV();

}



void RFID()
{

  
  //  // Look for new cards
  // if ( ! mfrc522.PICC_IsNewCardPresent() || ! mfrc522.PICC_ReadCardSerial() ) {
  //   delay(50);
  //   return;
  // }

  // // Show UID on serial monitor
  // // Serial.print("UID tag :");
  // String content= "";
  // byte letter;
  // for (byte i = 0; i < mfrc522.uid.size; i++) {
  //   //  Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
  //   //  Serial.print(mfrc522.uid.uidByte[i], HEX);
  //    content.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
  //    content.concat(String(mfrc522.uid.uidByte[i], HEX));
  // }
  // // Serial.println();
  // // Serial.print("Message : ");
  // content.toUpperCase();
  // if (content.substring(1) == "4A 97 A2 1E") { // Change to match your UID
  //   Serial.println("Authorized access");
  //   // Add your actions here for authorized access
  // }
  // else {
  //   Serial.println("Access denied");
  // }

  // delay(500);

}


void NAV()
{


  if ((done_flag == 1) && (at_targ == 1))   //moving out of place after pickup or delivery
  {
    BACKWARD();
    _delay_ms(1000);
  Serial.println("95");

    if (path[0] > 0)    // rotate left 90 degree
    {
      SPIN_L();
      _delay_ms(500);
      Serial.println("101");

    }

    else if (path[0] < 0)      //rotate right 90 degree
    {
      SPIN_R();
      _delay_ms(500);
      Serial.println("109");
    }

    else if (path[0] == 0)   //for cases where target is on the same x-line as position
    {
      if (pos[0]==2 || pos[0]==3)
      {
        SPIN_R();
      _delay_ms(500);
      Serial.println("118");
      }

      else if (pos[0] == 4)
      {
        SPIN_L();
      _delay_ms(500);
      Serial.println("124");
      }


    }

  
  FORWARD();
  _delay_ms(500);
  Serial.println("134");

  at_targ=0;
  }


  // if (pos[0]%4 == 1)   //at nodes looking in X-direction
  // {
  //   if ((dir[0]>0 && path[1]>0) || (dir[0]<0 && path[1]<0))
  //   {
  //     SPIN_L();
  //     _delay_ms(500);
  //     FORWARD();
  //     _delay_ms(500);
  //     Serial.println("148");
  //   }

  //   else if ((dir[0]>0 && path[1]<0) || (dir[0]<0 && path[1]>0))
  //   {
  //     SPIN_R();
  //     _delay_ms(500);
  //     FORWARD();
  //     _delay_ms(500);
  //     Serial.println("157");
  //   }
  // }

  // if (pos[0]%4 == 1)   //at nodes looking in Y-direction and target NOT at y=2
  // {
  //   if ((dir[1]>0 && path[1]>0) || (dir[1]<0 && path[1]<0))
  //   {
  //     FORWARD();
  //     _delay_ms(500);
  //     Serial.println("167");
  //   }
  // }

  // if (pos[0] == 1)   //at node 1 looking in Y-direction and target at y=2
  // {
  //   if ((dir[1]>0 && path[1]==0)) //facing +ve Y 
  //   {
  //     SPIN_R();
  //     _delay_ms(500);
  //     FORWARD();
  //     _delay_ms(500);
  //     Serial.println("179");
  //   }
    
  //   if ((dir[1]<0 && path[1]==0)) //facing -ve Y
  //   {
  //     SPIN_L();
  //     _delay_ms(500);
  //     FORWARD();
  //     _delay_ms(500);
  //     Serial.println("188");
  //   }
  // }

  // if (pos[0] == 5)   //at node 5 looking in Y-direction and target at y=2
  // {
  //   if ((dir[1]>0 && path[1]==0)) //facing +ve Y
  //   {
  //     SPIN_L();
  //     _delay_ms(500);
  //     FORWARD();
  //     _delay_ms(500);
  //     Serial.println("200");
  //   }

  //   else if ((dir[1]<0 && path[1]==0)) //facing -ve Y
  //   {
  //     SPIN_R();
  //     _delay_ms(500);
  //     FORWARD();
  //     _delay_ms(500);
  //     Serial.println("209");
  //   }
  // }

  if ((pos[0]%4==1) && (dir[0]==1 && dir[1]==1))
  {
      if (targ[1]==2)
      {
        SPIN_L();
        _delay_ms(500);
        FORWARD();
        _delay_ms(500);
      }

      if (targ[1]!=2)
      {
        FORWARD();
        _delay_ms(500);
      }
  }

  if ((pos[0]%4==1) && (dir[0]==1 && dir[1]==-1))
  {
    if (targ[1]==2)
    {
      SPIN_R();
      _delay_ms(500);
      FORWARD();
      _delay_ms(500);
    }

    if (targ[1]!=2)
    {
      FORWARD();
      _delay_ms(500);      
    } 

  }

  if ((pos[0]%4==1) && (dir[0]==-1 && dir[1]==1))
  {
    if (targ[1]==2)
    {
      SPIN_R();
      _delay_ms(500);
      FORWARD();
      _delay_ms(500);
    }

    if (targ[1]!=2)
    {
      FORWARD();
      _delay_ms(500);      
    }     
  }

  if ((pos[0]%4==1) && (dir[0]==-1 && dir[1]==-1))
  {
      if (targ[1]==2)
      {
        SPIN_L();
        _delay_ms(500);
        FORWARD();
        _delay_ms(500);
      }

      if (targ[1]!=2)
      {
        FORWARD();
        _delay_ms(500);
      }    
  }


  if ((path[0] == 0) && (path[1] == 0))  // rotating to face shelves at target point
  {
    if (dir[0] > 0)
    {
      SPIN_R();
      _delay_ms(500);
      FORWARD();
      _delay_ms(100);
      //while(distance < 7);
      //STOP();
      at_targ=1;
      Serial.println("225");
    }

    else if (dir[0] < 0)
    {
      SPIN_L();
      _delay_ms(500);
      FORWARD();
      _delay_ms(100);
      //while(distance < 7);
      //STOP();
      at_targ=1;
      Serial.println("237");
    }

  }


}


void FORWARD()
{
  Serial.println("forward");

}
void BACKWARD()
{
  Serial.println("Backward");

}
void SPIN_L()
{
  Serial.println("spin left");

}
void SPIN_R()
{
  Serial.println("spin right");

}

