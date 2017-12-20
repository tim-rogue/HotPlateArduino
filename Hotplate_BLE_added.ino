#include <Adafruit_MAX31855.h>

#include <Adafruit_BLE_UART.h>


//TO DO: add bluetooth functionality


#include <PID_v1.h>
#include <SPI.h>
//---------Start BLE Communication definitions and variables----------//
//--------------------------------------------------------------//
// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 3     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 0
//The number of data fields contained in the incomming data packet
#define NUM_DATA 4

//The packet to be received from the app
//The packet will look like "OFF,1120.0,120.0,45.7,34.8;" 
//i.e "heating state(ON/OFF),setpoint1,setpoint2,curr_temp1,curr_temp2;"
char receivePacket[10];
const char delimiters[]={",;"};
//< data packet headers that identify the data sent from app
const char Heating_ID[]="1";
const char Setpoint1_ID[]="2";
const char Setpoint2_ID[]="3";
//>
uint8_t packet_index = 0;
char* parsedData[NUM_DATA]; 
//string buffers to hold the current temp readings. needed for 
//creating the data packet to send to app 
//char temp1_buff[6];
//char temp2_buff[6];
//this string will carry the temp data packet sent to app
//char BTLEtempData[13];



Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
//---------End BLE Communication definitions and variables----------//
//--------------------------------------------------------------//

//****************Start PID definitions and variables************//
//***************************************************************//
// IO Pin assignments for first thermocouple
// digital IO pins.
#define MAXDO_1   A3
#define MAXCS_1   2
#define MAXCLK_1  4
#define RELAY_PIN_1 5

// IO Pin assignments for first thermocouple
// digital IO pins.
#define MAXDO_2   7
#define MAXCS_2   6
#define MAXCLK_2  8
#define RELAY_PIN_2 9

// initialize the Two Thermocouple
Adafruit_MAX31855 thermocouple_1(MAXCLK_1, MAXCS_1, MAXDO_1);
Adafruit_MAX31855 thermocouple_2(MAXCLK_2, MAXCS_2, MAXDO_2);

char heat_instruction[4] ="OFF";  
//Define PID Variables we'll be connecting to
double Setpoint_1, Input_1, Output_1, celsius_1;
double Setpoint_2, Input_2, Output_2, celsius_2;

//Specify the links and initial tuning parameters
double Kp_1=2, Ki_1=0.5, Kd_1=2;
double Kp_2=5, Ki_2=0.5, Kd_2=3;
PID myPID_1(&Input_1, &Output_1, &Setpoint_1, Kp_1, Ki_1, Kd_1, DIRECT);
PID myPID_2(&Input_2, &Output_2, &Setpoint_2, Kp_2, Ki_2, Kd_2, DIRECT);
double WindowSize = 5000.0;
unsigned long windowStartTime;
unsigned long LastReportTime = 0;
int ReportingInterval = 2000;
//****************Ends PID definitions and variables************//
//***************************************************************//

void setup() {


  /*************************/
  //Initialize Relay Pins 
  /************************/
  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);
  
  
  /*************************/
  //Initialize Thermocouple 
  /************************/
  while (!Serial); // wait for Serial on Leonardo/Zero, etc
  Serial.begin(9600);
  Serial.println("MAX31855 test");
  // wait for MAX chip to stabilize
  delay(500);
  
  /*************************/
  //Initialize PID 
  /************************/
  windowStartTime = millis();
  //Setpoint is the temperature in Celcius
  Setpoint_1 = 40;
  Setpoint_2 = 40;
  //tell the PID to range between 0 and the full window size
  myPID_1.SetOutputLimits(0.0, WindowSize);
  myPID_2.SetOutputLimits(0.0, WindowSize);
  //turn the PID on
  myPID_1.SetMode(AUTOMATIC);
  myPID_2.SetMode(AUTOMATIC);
  /*************************/
  //Initialize BLE serial//
  /*************************/
  BTLEserial.setDeviceName("Hot_Plate"); /* 7 characters max! */
  BTLEserial.begin();
  //initialize the parsed data array
  parsedData[0] = "OFF";;
  parsedData[1] = "";
  parsedData[2] = "";
  parsedData[3] = "";
}

aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop() {
  
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();
  // Ask what is our current BLE SErial status
  aci_evt_opcode_t status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  // put your main code here, to run repeatedly:
  if (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    if (BTLEserial.available()) {
      Serial.print("* "); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));
    }
    // Read in all the incomming characters and add them to receivedPacket
    while (BTLEserial.available()) {
      char c = BTLEserial.read();
      //Serial.println(c);
      if(c != ';')
      {
        receivePacket[packet_index]=c;
        receivePacket[packet_index+1]='\0';
        //Serial.println(receivePacket);
        packet_index++;
      }else
      {
        receivePacket[packet_index]=c;
        receivePacket[packet_index+1]='\0';
        //Serial.println(receivePacket);
        packet_index = 0;
        //parse the received packet into its subcomponents
        bleSerialParse(receivePacket,parsedData, delimiters);
       }
    }
  }
  
  //celsius_1 = thermocouple_1.readCelsius();
  //celsius_2 = thermocouple_2.readCelsius();
  /***************************************************/
  //Print Thermocouple Temp
  /***************************************************/
  // basic readout test, just print the current temp
  if (millis() - LastReportTime > ReportingInterval|| LastReportTime== 0)
  {
   //Serial.print("Internal Temp = ");
   //Serial.println(thermocouple.readInternal());
   //Serial.println(F("##########################"));
   Serial.print(F("Heat Instruction: "));Serial.println(heat_instruction);
   
   //Serial.print(F("Setpoint 1 -- Setppoint 2 = "));
   Serial.print(Setpoint_1); Serial.print(" -- ");Serial.print(Setpoint_2);Serial.print("\n");
   
   reportTempBLE(&BTLEserial, &Serial);
   
   //<<code needed to write send current temp data to teh app
   //get the current temperatures 
   /*Input_1 = thermocouple_1.readCelsius();
   Input_2 = thermocouple_2.readCelsius();
   //convert float values to strings
   dtostrf(Input_1, 5, 1, temp1_buff);
   dtostrf(Input_2, 5, 1, temp2_buff);
   //concatinate and format the string for sending to the app
   sprintf(BTLEtempData,"%s,%s;",temp1_buff,temp2_buff);
   char sendbuffersize = min(20, sizeof(BTLEtempData));
   //send the data packet via BTLE to app
   BTLEserial.write(BTLEtempData, sendbuffersize);*/
   //>>


   
   //Serial.print("Window Size = ");
   //Serial.println(WindowSize);
   //Serial.print("PID Output 1 ** PID Output 1 = ");
   //Serial.print(Output_1); Serial.print(" ** ");Serial.print(Output_2);Serial.print("\n");
   //Serial.print("windowStartTime = ");
   //Serial.println(windowStartTime);
   //Serial.print("millis() - windowStartTime = ");
   //Serial.println(millis() - windowStartTime); 

   
   /*if (isnan(celsius_1)) {
     Serial.println("Something wrong with thermocouple_1!");
   } else {
     Serial.print("celsius_1 = "); 
     Serial.println(celsius_1);
   }

   if (isnan(celsius_2)) {
     Serial.println("Something wrong with thermocouple_2!");
   } else {
     Serial.print("celsius_2 = "); 
     Serial.println(celsius_2);
   }*/

    LastReportTime = millis();
   
  }
   /***************************************************/
  //Print Thermocouple Temp
  /***************************************************/

  /***************************************************/
  //Update PID and control Relay based on it
  /***************************************************/
  //If the app has sent an ON data packet, signifying
  //an instruction to begin heating 
  if(strcmp(heat_instruction,"ON")==0){
    
      //Serial.print("inside \"ON\"loop ");
      //assign PID input value to celsius
      Input_1 = thermocouple_1.readCelsius();
      Input_2 = thermocouple_2.readCelsius();
      while(isnan(Input_1)||isnan(Input_2))
      {
        
        if(isnan(Input_1))
        {
        Serial.println(F("thermocouple_1 returning NaN"));
        Input_1 = thermocouple_1.readCelsius();
        }
        
        if(isnan(Input_2))
        {
        Serial.println(F("thermocouple_2 returning NaN"));
        Input_2 = thermocouple_2.readCelsius();
        }
        
        
      }
      
      if (millis() - LastReportTime > ReportingInterval|| LastReportTime== 0)
      { reportTempBLE(&BTLEserial, &Serial);
        Serial.print(F("celsius_1, Celsius 2 = ")); 
        Serial.print(Input_1);Serial.print(" , ");Serial.println(Input_2);
        LastReportTime = millis();
      }
      myPID_1.Compute();
      myPID_2.Compute();
      
      //turn the output pin on/off based on pid output
      if (millis() - windowStartTime > WindowSize)
      { //time to shift the Relay Window
        windowStartTime += WindowSize;
      }
      //changed bool operator from '<' to '>'
      if (Output_1 > millis() - windowStartTime){
        digitalWrite(RELAY_PIN_1, HIGH);
        Serial.print(F("RELAY 1 ON+"));
        
      } else {
        digitalWrite(RELAY_PIN_1, LOW);
        Serial.print(F("RELAY 1 OFF-"));
         
      }
    
      //changed bool operator from '<' to '>'
      if (Output_2 > millis() - windowStartTime){
        digitalWrite(RELAY_PIN_2, HIGH);
        Serial.println(F("  REL 2 ON+"));
        
      } else {
        digitalWrite(RELAY_PIN_2, LOW);
        Serial.println(F("  REL 2 OFF-"));
         
      }
   //else the heating instruction is "OFF" and 
   //The relay pins should be turned off (hot plates un powered)   
  }else{
    
      digitalWrite(RELAY_PIN_1, LOW);
      digitalWrite(RELAY_PIN_2, LOW);
    
    }
}

void reportTempBLE(Adafruit_BLE_UART* BLEserial, HardwareSerial* serial){
  
   char buffSize;
   char temp1[6];
   char temp2[6];
   char tempData[13];
   double In_1;
   double In_2;
  //<<code needed to write send current temp data to teh app
   //get the current temperatures 
   
   In_1 = thermocouple_1.readCelsius();
   In_2 = thermocouple_2.readCelsius();
   while(isnan(In_1)||isnan(In_2))
      {
        
        if(isnan(Input_1))
        {
        //Serial.println(F("thermocouple_1 returning NaN"));
        In_1 = thermocouple_1.readCelsius();
        }
        
        if(isnan(Input_2))
        {
        //Serial.println(F("thermocouple_2 returning NaN"));
        In_2 = thermocouple_2.readCelsius();
        }
        
        
      }
   //serial->println(In_1);
   //serial->println(In_2);
   
   //convert float values to strings
   dtostrf(In_1, 5, 1, temp1);
   dtostrf(In_2, 5, 1, temp2);
   //serial->println(temp1);
   //serial->println(temp2);
   //concatinate and format the string for sending to the app
   sprintf(tempData,"%s,%s;",temp1,temp2);
   //serial->println(tempData);
   buffSize = min(20, sizeof(tempData));
   //send the data packet via BTLE to app
   BLEserial->write(tempData, buffSize);
  
  
  }
 
//THis function parses the input data packet
//The packet will look like "OFF,1120.0,120.0" 
//i.e "heating state(ON/OFF),setpoint1,setpoint2"
void bleSerialParse(char inputString[],char* stringArray[], char delimiters[]){

  uint8_t strSize = sizeof(inputString);
  uint8_t stringIndex = 0;
  uint8_t arrayIndex = 0;
  

  char tempString[strSize];
  char* token;
  //make copy of inputString
  strcpy(tempString,inputString);
  //Get first token parsed from input string
  token = strtok(tempString,delimiters);
  //save that token in the reurn array
  stringArray[arrayIndex]=token;
  //increment the index fro the next token
  arrayIndex++;
  
  
  while(token!= NULL){
       Serial.println(token);
       //get another parsed token
       token = strtok(NULL,delimiters);
       //add it to the array
       stringArray[arrayIndex]=token;
       //increment the array index
       arrayIndex++;
       
    }
    //if the data packet headre is '1' and indictates a heating instruction
    if(strcmp(stringArray[0],Heating_ID)==0)
    { 
      //copy the second part of the data packet into heat_instuction variable.
      //will be "ON" or "OFF"
      strcpy(heat_instruction, stringArray[1]);
      Serial.print(F("Heat Instruct: "));Serial.println(heat_instruction);
      
      //if the data packet header is '2' and indictates a Setpoint1
     }else if(strcmp(stringArray[0],Setpoint1_ID)==0)
     {  //convert Setpoint 1 string into float and save in setpoint variable
        Setpoint_1 = atof(stringArray[1]);
        Serial.print(F("Setpoint1: "));Serial.println(Setpoint_1);

      //if the data packet header is '2' and indictates a Setpoint1  
     }else if(strcmp(stringArray[0],Setpoint2_ID)==0)
     {  
      //convert Setpoint 1 string into float and save in setpoint variable
        Setpoint_2 = atof(stringArray[1]);
        Serial.print(F("Setpoint2: "));Serial.println(Setpoint_2);
     }else
     {
      
      Serial.print(F("Unrecognized header1"));
      
      } 
    

    
    //convert/move values from the string array into
    //strcpy(heat_instruction, stringArray[0]);
    //Serial.print(F("Heating Instruction: "));Serial.println(heat_instruction);
    //Setpoint_1 = atof(stringArray[1]);
    //Serial.print(F("Setpoint_1: "));Serial.println(Setpoint_1);
    //Setpoint_2 = atof(stringArray[2]);
    //Serial.print(F("Setpoint_2: "));Serial.println(Setpoint_2);
    

    
    
}
