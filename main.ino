#include "TimerOne.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
   
#include <SPI.h>  
#include <Pixy.h>

// This is the main Pixy object 
Pixy pixy;

// Use pins 11 and 12 to communicate with DFPlayer Mini
SoftwareSerial mySoftwareSerial(12, 11); // RX, TX

//SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);

// Create the Player object
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

const byte MOTOR1 = 2; //Motor 1 interrupt pin (INT 0)
const byte MOTOR2 = 3; //Motor 2 interrupt pin (INT 1)

volatile int counter1=0;  //COUNTER RUOTA SX
volatile int counter2=0; //COUNTER RUOTA DX

const float wheeldiameter=66.10;
int stby=4;
//Motor A SINISTRO
int enA = 10;
int in1 = 9;
int in2 = 8;

//Motor B DESTRO
int enB = 5;
int in3 = 7;
int in4 = 6;

int max=13000;//block min dimension to activate action
int points;
static unsigned long timer;

//Interrupt service routines

void ISR_count1(){
  counter1++;
}
void ISR_count2(){
  counter2++;
}
/*

void TimerOne(){
  Timer1.detachInterrupt(); //FERMA IL TIMER (Serve per poter fare i print)
  Serial.print("Motor Speed 1: ");
  float rotation1 = (counter1/20.00)*60.00;
  serial.print(rotation1);
  serial.print(" RPM - ");
  counter1= 0;
  Serial.print("Motor Speed 2: ");
  float rotation1 = (counter2/20.00)*60.00;
  Serial.print(rotatoin2)
  srial.printl(" RPM");
  counter2 =0;
  Timer1.attachInterrupt(ISR_timerone);
}
 */
 void aggiusta(){
  if(counter1>counter2+5){
    Serial.println("Aggiusta ruota dx");
    while(counter1>counter2){
          analogWrite(enB,250);
    }
          analogWrite(enB,150);

  }
    if(counter2>counter1+7){
          Serial.println("Aggiusta ruota sx");
    while(counter2>counter1){
          analogWrite(enA,200);
    }
          analogWrite(enA,100);

  }
 }
void moveForward(int steps, int mspeed){
  counter1=0;
  counter2=0;

  //SX
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  //DX
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
 
  

  while(steps>counter1 && steps>counter2){
    if(steps>counter1){
       analogWrite(enA,mspeed);
    }else{
      analogWrite(enA,0);
    }
    if(steps>counter2){
      analogWrite(enB,mspeed);
    }else{
      analogWrite(enB,0);
    }
    Serial.print("GIRI RUOTA DX:");
    Serial.print(counter2);
    Serial.print("              GIRI RUOTA SX:");
    Serial.println(counter1);
    Serial.print("GIRI richiesti:");
    Serial.println(steps);
    
  }
  
  analogWrite(enA,0);
  analogWrite(enB,0);
  counter1=0;
  counter2=0;
}

void turnLeft(int steps, int mspeed){
  counter1=0;
  counter2=0;
    Serial.println("SVOLTA A SINISTRA");

  //SX
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  Serial.println("motore 1 acceso");
  //DX
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
  Serial.println("motore 2 acceso");

  while(steps>counter2){
    if(steps>counter2){
      analogWrite(enA,mspeed);
    /*}else{
      analogWrite(enA,0);
    }
  if(steps>counter2){
      analogWrite(enB,mspeed);
    }else{
      analogWrite(enB,0);
    }*/
  }
  }
  analogWrite(enA,0);
  analogWrite(enB,0);
  counter1=0;
  counter2=0;
}

void turnRight(int steps, int mspeed){
  counter1=0;
  counter2=0;
   Serial.println("SVOLTA A DESTRA");

  //SX
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);

  //DX
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);


  
  while( steps>counter1){
    /*if(steps>counter1){
      analogWrite(enA,mspeed);

    }else{
      analogWrite(enA,0);
    }*/
    if(steps>counter1){
      analogWrite(enB,mspeed);

    }
  }

  analogWrite(enA,0);
  analogWrite(enB,0);
  counter1=0;
  counter2=0;
}

void setup() {
  // put your setup code here, to run once:
     digitalWrite(stby,HIGH);
       mySoftwareSerial.begin(9600);

  Serial.begin(115200);
 // Timer1.initialize(1000000); //set timer for 1 sec
  attachInterrupt(digitalPinToInterrupt (MOTOR1),ISR_count1,RISING); //increase counter 1 when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt (MOTOR2),ISR_count2,RISING); //increase counter 2 when speed sensor pin goes High
 // Timer1.attachInterrupt( ISR_timerone); //enable the timer
 // Init serial port for DFPlayer Mini
  
  Serial.print("Starting...\n");

  pixy.init();

Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true);
  }
  Serial.println(F("DFPlayer Mini online."));

  myDFPlayer.volume(28);  //Set volume value. From 0 to 30
  myDFPlayer.play(6);  //sono pronto si parte
  
}

void loop() {
  static int i = 0;
  int maxCurr=0;
  int maxBlock=0;
  int j;
  int dimension;
  uint16_t blocks;
  char buf[32]; 


   //SX
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  //DX
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);

  analogWrite(enA,100);
  analogWrite(enB,150);
  aggiusta();

  blocks = pixy.getBlocks(); //pixy scan for blocks
  

  if(blocks){
              

    i++;
    if(i%80==0){ //action made every 100 frames to not bog the camera
        Serial.println("ENTRA NELL secondo IF");

      for(j=0;j<blocks;j++){ //check between all the blocks in the view, which is the biggest
        dimension=pixy.blocks[j].width * pixy.blocks[j].height;
        if(dimension>maxCurr){
          maxCurr=dimension;
          maxBlock=j; //
        }
      }
         
          sprintf(buf, " Dimensione blocco maggiore: %d", maxCurr);
          Serial.println(buf);
          
          pixy.blocks[maxBlock].print();
      
      if(maxCurr>=max){
        switch(pixy.blocks[maxBlock].signature){
          case 1:
          //SVOLTA A DX ROSSO
                Serial.println("Rosso");
                myDFPlayer.play(7);  //Play the first mp3
                turnRight(20,2000);
                points+=3;
                counter1=0;
                counter2=0;
                break;
          case 2:
          //SVOLTA A SX vERDE
              Serial.println("verde");
              myDFPlayer.play(10);  //Play the first mp3
                points+=2;

                //player.play(2);
                turnLeft(20,2000);
              counter1=0;
              counter2=0;
                break;
          /*case 3:
                Serial.println("GIRAVOLTA");
                myDFPlayer.play(2);  //Play the first mp3

                turnLeft(25,200);
                points=+10;

                break;*/
                
          case 3: //victory case
                //Serial.println("VITTORIA1");
                //Play the first mp3
                myDFPlayer.play(5);

                //Serial.println("VITTORIA2");
               // myDFPlayer.play(5);
                
                analogWrite(enA,0);
                analogWrite(enB,0);
                delay(4000);


             /// DA SISTEAMRE - VEDI TEST AUDIO
             
                switch (points){
                case 2:
                   myDFPlayer.play(4); 
                   break;
                case 3:
                 myDFPlayer.play(3); 
                 break;
                case 4:
                 myDFPlayer.play(2);
                 break;
                case 5:
                 myDFPlayer.play(1);
                 break;
                 case 6:
                 myDFPlayer.play(9);
                 break;
                 default:
                 myDFPlayer.play(4); 
                 break;
                }
                analogWrite(enA,0);
                analogWrite(enB,0);
                while(true)
                {
                    Serial.println("Delay");
                    delay(1000);
                    
                }
                break;
         default :
        break;
        }
      }
      //if no blocks nearby, proceed forward
      else{     
                  Serial.println("SECOND MOVE FORWARD");
                   //SX
                  digitalWrite(in1,HIGH);
                  digitalWrite(in2,LOW);
                  //DX
                  digitalWrite(in3,LOW);
                  digitalWrite(in4,HIGH);
                
                    analogWrite(enA,100);
                    analogWrite(enB,150);
        

      }
      
    }
  }

}
