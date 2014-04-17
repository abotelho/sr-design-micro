/*******************************************************************************
 * p2d2.ino
 *  
 * Authors: Andrew Botelho, Praveer Nidamaluri, Sylvester Tate, Wenbin Zhao
 * Bluetooth communications based off "Example Bluetooth Serial Passthrough 
 * Sketch" by Jim Lindblom of SparkFun Electronics
 * Last update: 2014-04-17
 * 
 * This sketch is run on the Arduino Duemilanove inside the P2D2 diagnostic 
 * device. It controls the device's heating, lighting, fluid actuation, and 
 * bluetooth communication with a smartphone. 
 ******************************************************************************/


#include "bluetooth.h"
#include "Tag.h"
#include <SoftwareSerial.h>  

/******************************************************************************
 * Variable initializations
 ******************************************************************************/
// pins
const int bluetoothTx = 2;  // TX-O pin of bluetooth mate, Arduino D2
const int bluetoothRx = 3;  // RX-I pin of bluetooth mate, Arduino D3
const int led = 13;  // LED output pin

int length; //length of packet
int cmd; //command selection from bluetooth
int srcmd; //status requests
int heatingcmd; //heating commands
int ledcmd; //led commands
int fluidscmd; //fluid commands
int minPacketSize = 5;
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx); // connection to Blue SmiRF

// Runs before loop()
void setup()
{
  // Bluetooth setup
  Serial.begin(9600);  // Begin the serial monitor at 9600bps

  bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
  bluetooth.print("$");  // Print three times individually
  bluetooth.print("$");
  bluetooth.print("$");  // Enter command mode
  delay(100);  // Short delay, wait for the Mate to send back CMD
  bluetooth.println("U,9600,N");  // Change baudrate to 9600, no parity
  // (115200 can be too fast at times for NewSoftSerial to relay data reliably)
  bluetooth.begin(9600);  // Start bluetooth serial at 9600
}

// Runs continuously after setup
void loop(){
  // If bluetooth receiverd at least minPacketSize characters
  if(bluetooth.available() >= minPacketSize)  
  {
    // Send any characters the bluetooth prints to the serial monitor
    //Serial.print((char)bluetooth.read());  

    if(Serial.available())  // If stuff was typed in the serial monitor
    {
      // Send any characters the Serial monitor prints to the bluetooth
      bluetooth.print((char)Serial.read());
    }
    // and loop forever and ever!
  }
}

// Processes status requests received over bluetooth
void status_requests(int cmd){
  switch(cmd){
  case LED_STATE: 
    {
      Tag ledTag;
      if(digitalRead(led) == HIGH){
        ledTag.setValues(LED_STATE, LED_ON);
      } 
      else {
        ledTag.setValues(LED_STATE, LED_OFF);
      }
      Tag tagArray[] = {
        ledTag                                    };
      writeToBT(tagArray);
      Serial.print("//led state request");
      break;
    }
  case HEATING_STATE: 
    Serial.print(" //heating state request");
    break;
  case FLUID_STATE:
    Serial.print("//fluid acutation state request");
    break;
  case TEMP_DATA:
    Serial.print("//temp data request");
    break;
  case FULL_STATUS:
    Serial.print("//full status (all state values and data)");
    break;
  default:
    Serial.print("invalid input received");
  }
} 

// Processes heating commands received over bluetooth
void heating_commands(int cmd){
  switch(cmd){
  case HEAT_TEMP_1:
    {
      //Start/switch heating-- temp 1
      Serial.print("Start/switch heating-- temp 1\r\n");
      break;  
    }
  case HEAT_TEMP_2:
    {
      //Start/switch heating-- temp 2
      Serial.print("Start/switch heating-- temp 2\r\n");
      break;  
    }
  case HEAT_TEMP_3:
    {
      //Start/switch heating-- temp 3
      Serial.print("Start/switch heating-- temp 3\r\n");
      break;  
    }
  case HEAT_TEMP_4:
    {
      //Start/switch heating-- temp 4
      Serial.print("Start/switch heating-- temp 4\r\n");
      break;  
    }
  case STOP_HEATING:
    {
      //Stop heating
      Serial.print("Stop heating \r\n");
      break;  
    }
  }
}

// Processes LED commands received over bluetooth
void led_command(int cmd){
  if (cmd == LED_ON){
    Serial.print("//Leds on\r\n");
  }
  else if(cmd == LED_OFF){
    Serial.print("//Leds off\r\n");
  }
}  

// Processes fluid actuation commands received over bluetooth
void fluids_command(int cmd){
  if (cmd == FLUIDS_ACTUATE){
    //Start fluids actuation
    Serial.print("Start fluids actuation");
  }
  else if(cmd == FLUIDS_STOP){
    //Stop fluids actuation
    Serial.print("Stop fluids actuation");
  }
}  

// Processes packets received over bluetooth
void processPacket(){
  if(bluetooth.read() == HEADER_IN_1){
    //Serial.print("first byte received\r\n");
    if(bluetooth.read() == HEADER_IN_2){
      //Serial.print("second byte received\r\n");
      length = bluetooth.read();
      //Serial.print(length);
      for( int i = 1; i <= length; i++){
        cmd = bluetooth.read();
        switch(cmd){
        case STATUS_REQUEST:
          {
            srcmd = bluetooth.read();
            status_requests(srcmd);
            break;
          }
        case HEATING_STATE:
          {
            heatingcmd = bluetooth.read();
            heating_commands(heatingcmd);
            break;
          }
        case LED_STATE:
          {
            ledcmd = bluetooth.read();
            led_command(ledcmd);
            break;
          }
        case FLUID_STATE:
          {
            fluidscmd = bluetooth.read();
            fluids_command(fluidscmd);
            break;
          }
        }
      }

    }
    else{
      bluetooth.flush();
    }
  }
  else{
    bluetooth.flush();
  }
}

void writeToBT(Tag tagArray[]){
  byte tagArraySize = sizeof(tagArray)/sizeof(Tag);
  bluetooth.write(HEADER_OUT_1);
  bluetooth.write(HEADER_OUT_1);
  bluetooth.write(tagArraySize);
  for(int i = 0; i< tagArraySize; i++){
    byte type = tagArray[i].getType();
    bluetooth.write(type);
    bluetooth.write(tagArray[i].getData1());
    if(type == TEMP_DATA){
      bluetooth.write(tagArray[i].getData2());
    }
  }
}

