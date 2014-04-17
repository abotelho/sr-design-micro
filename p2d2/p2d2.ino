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
 */

#include <SoftwareSerial.h>  

/*******************************************************************************
 * Definitions 
 ******************************************************************************/
//Header bytes
#define HEADER_IN_1 0xB8
#define HEADER_IN_2 0xD3
#define HEADER_OUT_1 0xFA
#define HEADER_OUT_2 0xCE

//Led status bytes
#define LED_STATE 0x3C
#define LED_ON 0xFF
#define LED_OFF 0x00

// Heating status bytes
#define HEATING_STATE 0x57
#define HEAT_TEMP_1 0x33
#define HEAT_TEMP_2 0x55
#define HEAT_TEMP_3 0x65
#define HEAT_TEMP_4 0xFF
#define STOP_HEATING 0x00
#define HEATING_TEMP_1 0x31
#define HEATING_TEMP_2 0x51
#define HEATING_TEMP_3 0x62
#define HEATING_TEMP_4 0xF7

// Fluid status bytes
#define FLUID_STATE 0x8F
#define FLUIDS_ACTUATE 0xFF
#define FLUIDS_STOP 0x00
#define FLUIDS_NOT_ACTUATED 0x00
#define FLUIDS_ACTUATING 0x44
#define FLUIDS_ACTUATED 0xFF

// Temp data byte
#define TEMP_DATA 0x6A

// Full status request bytes
#define STATUS_REQUEST 0xFF
#define FULL_STATUS 0xA5

/******************************************************************************
 * Variable initializations
 ******************************************************************************/
int bluetoothTx = 2;  // TX-O pin of bluetooth mate, Arduino D2
int bluetoothRx = 3;  // RX-I pin of bluetooth mate, Arduino D3
int length; //length of packet
int cmd; //command selection from bluetooth
int srcmd; //status requests
int heatingcmd; //heating commands
int ledcmd; //led commands
int fluidscmd; //fluid commands
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
  if (cmd == LED_STATE){
    Serial.print("//led state request");
  }
  else if(cmd == HEATING_STATE){
   Serial.print(" //heating state request");
  }
  else if (cmd == FLUID_STATE){
   Serial.print("//fluid acutation state request");
  }
  else if (cmd == TEMP_DATA){
   Serial.print("//temp data request");
  }
  else if (cmd == FULL_STATUS){
    Serial.print("//full status (all state values and data)");
  }else{
    Serial.print("invalid input received");
  }
} 

// Processes heating commands received over bluetooth
void heating_commands(int cmd){
  if (cmd == HEAT_TEMP_1){
    //Start/switch heating-- temp 1
    Serial.print("Start/switch heating-- temp 1\r\n");  
  }
  else if(cmd == HEAT_TEMP_2){
    //Start/switch heating-- temp 2
     Serial.print("Start/switch heating-- temp 2\r\n");
  }
  else if (cmd == HEAT_TEMP_3){
    //Start/switch heating-- temp 3
    Serial.print("Start/switch heating-- temp 3\r\n");
  }
  else if (cmd == HEAT_TEMP_4){
    //Start/switch heating-- temp 4
    Serial.print("Start/switch heating-- temp 4\r\n");
  }
  else if (cmd == STOP_HEATING){
    //Stop heating
    Serial.print("Stop heating \r\n");
  }
}

// Processes LED commands received over bluetooth
void led_command(int cmd){
  if (cmd == LED_ON){
    Serial.print("//Leds on\r\n");
  }else if(cmd == LED_OFF){
     Serial.print("//Leds off\r\n");
  }
}  

// Processes fluid actuation commands received over bluetooth
void fluids_command(int cmd){
  if (cmd == FLUIDS_ACTUATE){
    //Start fluids actuation
   Serial.print("Start fluids actuation");
  }else if(cmd == FLUIDS_STOP){
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
          if (cmd == STATUS_REQUEST){
            srcmd = bluetooth.read();
            status_requests(srcmd);
          }
          else if (cmd == HEATING_STATE){
            heatingcmd = bluetooth.read();
            heating_commands(heatingcmd);
          }
          else if (cmd == LED_STATE){
            ledcmd = bluetooth.read();
            led_command(ledcmd);
          }
          else if (cmd == FLUID_STATE){
            fluidscmd = bluetooth.read();
            fluids_command(fluidscmd);
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
