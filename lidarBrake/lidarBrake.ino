#include <SoftwareSerial.h>
#include <Servo.h>

SoftwareSerial Serial1(2,3); //define software serial connection with RX (receiving) pin 2 and TX (transmitting) pin 3
Servo rightSide; //define right servo
Servo leftSide; //define left servo

int dist; //actual distance measurements
int strength; //signal strength
float temperature; //package temperature
int check;  //save check value
int uart[9];  //LIDAR sensor save data
const int HEADER=0x59;  //frame header of data package

int dists[1]; //interval stop and start distances from obstacle
int bikeSpeed; //instantaneous bike speed
long lastMillis = 0; //interval start time, starts at 0

int gateway = 145; //define gateway distance to start braking

void setup() {
  Serial.begin(9600); //set bit rate of serial port connecting Arduino with computer
  Serial1.begin(115200);  //set bit rate of serial port connecting LiDAR with Arduino

  rightSide.write(1000); //set starting position of rightSide motor
  rightSide.attach(10); //attaches rightSide motor to pin 9

  leftSide.write(1000);
  leftSide.attach(11);  //same for left side
  
  dists[1] = -1; //set interval start distance as undefined
}

void loop() {
  if (Serial1.available()) {  //check if serial port has data input
    if(Serial1.read() == HEADER) {  //assess data package frame header 0x59
      uart[0]=HEADER;
      if (Serial1.read() == HEADER) { //assess data package frame header 0x59
        uart[1] = HEADER;
        for (int i = 2; i < 9; i++) { //save data in array
          uart[i] = Serial1.read();
        }
        check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
        if (uart[8] == (check & 0xff)){ //verify the received data as per protocol
          dist = uart[2] + uart[3] * 256;     //calculate distance value
          strength = uart[4] + uart[5] * 256; //calculate signal strength value
          temperature = uart[6] + uart[7] * 256;//calculate chip temperature
          temperature = temperature/8 - 256;

          dists[1] = dists[0]; //set interval start distance
          dists[0] = dist; //set interval end distance

          unsigned long currentMillis = millis(); //end interval
          
          if(dists[1] != -1) //makes sure one full cycle has passed before judging speed
          
          {
            bikeSpeed = (dists[1] - dists[0])/(currentMillis - lastMillis); //set speed to interval distance over interval time
             lastMillis = currentMillis; //reset timer

            if(dist < gateway && bikeSpeed > 1) //arbitrary value for now
            {
              for(int i=rightSide.read();i<=1500;i+=100)
              {
                rightSide.write(i);
                delay(100); 
              }
              for(int i=leftSide.read();i<=1500;i+=100) //1500 is arbitrary--just based on other side for now
              {
                rightSide.write(i);
                delay(100); 
              }
            }
            else
            {
              rightSide.write(1000);
              leftSide.write(1000); //1000 is arbitrary--based on other side
            }
            
          }
        }
      }
    }
  }
}
