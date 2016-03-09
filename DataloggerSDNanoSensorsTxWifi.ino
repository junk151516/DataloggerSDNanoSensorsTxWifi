/*
  SD card Datalogger for Arduino Nano

 This example shows how to log data from three analog sensors
 to an SD card using the SD library.

 The circuit:
 * analog sensors on analog ins 0, 1, and 2
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4

 Created by Forsat Ingenieria S.A.S

*/

#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <EEPROM.h>
#include <math.h>
#include <Timer.h>

// Macros
#define byteUsedForEEprom 50 
#define chipSelect 4
#define sensor1Pin A0
#define sensor2Pin A1
#define sensor3Pin A2
#define sensor4Pin A3
#define sensor5Pin A4
#define sensor6Pin A5
#define wifiModuleEnablePin A7
#define resetPin 2
#define timeToUploadDataToCloud 120000
#define writingOnSDCardState 1
#define readingKarelDataState 2
#define standByState 3
#define noTasksPending 0 
#define sendingPackageDataToWifiTask 1
#define sendingResetFlagToWifiTask 2  
#define resetComand "resetear(1)"  
#define ADCLevelValueForDigitalInput 368
#define sensorFeedVoltaje 5
#define canSave 2


// Global Variables
 
// Change Serial Pins Because Bridge Uses 0 and 1 as Rx and Tx
SoftwareSerial mySerial(8, 9); // RX, TX
   
// Others 
 bool shouldStartReading=false;
 byte globalState=standByState;
 unsigned long milliSeconds=0;
 unsigned int index=0;
 int firstScaler=0;
 int numberOfRawsInExcel=30000; 
 int indexToChangeFileName=0;
 int indexReadFromEEprom=0;
 int sensor1Value=0;
 int sensor2Value=0;
 int sensor3Value=0;
 int sensor4Value=0;
 int sensor5Value=0;
 int sensor6Value=0;
 byte karelData1=0;
 int karelData2=0;
 long karelData3=0;
 word karelData4=0;
 byte karelData5=0;
 unsigned int bufferSize=30;
 unsigned int characterOfEnd=69;
 unsigned int characterOfStart=83;
 volatile char pendingTasks[10];
 char myBuffer[50];
 char numberOfFile[5];
 char *formatOfFile= ".xls";
 char nameWithFormat[30]= {"F0.xls"};      
 // Timer Variables
 Timer dataUploadTimer;
 int saveData = 0;                              
 
void setup() {
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  
  // Welcome Message
  Serial.println("Filesystem datalogger\n");

  // Initialize and Set the data rate for the SoftwareSerial port
  mySerial.begin(9600);

  Serial.print("Initializing SD card...");

  // Initialize SD Card and Check if it's present
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  
  // Initialize timer methods
  
  // Set timer to Upload Sensor Data to Cloud every X time
  dataUploadTimer.every(timeToUploadDataToCloud, timeInterruptHandling);       
  
  // Flag which turn the LED on to check when it's been already set up
  //digitalWrite(13, HIGH);  
  
  // Initialize tasks vector
  
  // Send Package Data Task
  pendingTasks[0]=noTasksPending; 
  // Send Reset Command Task
  pendingTasks[1]=noTasksPending;  
  
  // Initialize IRQ in digital pin #D2 and prepare it as well for digital reading
  pinMode(resetPin,INPUT);
  attachInterrupt(digitalPinToInterrupt(resetPin), eventIRQHandling, RISING);
                                   
}

void loop() {
  
   // Check if there is any pending task
   checkForPendingTasks();
   
   // Update Timers
   updateTimers();
  
   // Write dataString on the SD CARD on Interrupt Serial Handling Event
   while(mySerial.available()>0){
      
      // Read the Serial 
      myBuffer[index]=(mySerial.read());
      
      // Check if should start reading
      if (myBuffer[index]==characterOfStart)
      {
        shouldStartReading=true;
      }
      
      // Go to save the serial data into a vector
      if (shouldStartReading==true)
      {
        saveSerialData();
      }
      
   }

}

// Method to update my timers
void updateTimers(){
  
  dataUploadTimer.update();

}

// Method to check for pending tasks
void checkForPendingTasks(){
  
  if ((pendingTasks[0]==noTasksPending) && (pendingTasks[1]==noTasksPending))
  {
    // Do nothing!
  }
  else{
    
    // Check if it's needing to send data to wifi task and its state allows it
    if ((pendingTasks[0]==sendingPackageDataToWifiTask) && (globalState!=writingOnSDCardState) && (HIGH==checkDigitalInputByAnalogReading(wifiModuleEnablePin)))
    {
      sendPackageDataToWifiModule();

      // Set this pending task as noPendingTask
      pendingTasks[0]=noTasksPending;
    }else {
          Serial.println("Wifi off");
    }
    
    // Check if it's needing to send a reset message to wifi task and its state allows it
    if ((pendingTasks[1]==sendingResetFlagToWifiTask) && (globalState!=writingOnSDCardState))
    {
      byte resetPinStatus = digitalRead(resetPin);
      // Delay to avoid that multiple interrupts occur when pulser is pressed
      delay(1000);
      byte resetPinStatusAfterDelay = digitalRead(resetPin);
      // Second Delay to avoid that multiple interrupts occur when pulser is pressed
      delay(1000);
      byte resetPinStatusAfterSecondDelay = digitalRead(resetPin);
      // Check if pin has been pressed for at least one second
      if ((resetPinStatus==HIGH) && (resetPinStatusAfterDelay==HIGH) && (resetPinStatusAfterSecondDelay==HIGH))
      {
        // Send via serial
        sendResetCommandToWifiModule();
        // Set this pending task as noPendingTask
        pendingTasks[1]=noTasksPending;
        // Re-Enable Interruption IRQ
        attachInterrupt(digitalPinToInterrupt(resetPin), eventIRQHandling, RISING);
      }
    }
  }
}

// Method to save data into a buffer

void saveSerialData(){
  
   //Serial1.print(myBuffer[index]);
   
   // Place the new data in a new position of the vector every time that a byte is in the buffer
   index++;
   
   // Check if data received is completed
   checkIfDataIsComplete();
   
   // Delay to Wait for the Serial Communication
   delay(1);
  
}

// Check if the data is complete to start writing on the SD
void checkIfDataIsComplete(){
  
   // Compare if Character Message has the appendix, the end, and the number of characters indicated
    if ((myBuffer[0]==characterOfStart) && (myBuffer[index-1]==characterOfEnd) && (index==bufferSize))
   {
     
    Serial.println("Cadena correcta");
    
    // Execute all process to write the message
    executeProcessToWriteMessagesOnSD();
       
  }
  
  // Check if should discard the message and reset the buffer
  if (index>bufferSize)
  {
    // Discard Message
    discardMessage();
  }
  
}

// Method which orders a way of execution in the software for saving on SD 
void executeProcessToWriteMessagesOnSD(){

  // Stop Listening to serial to give a time to my processor to write on SD
  mySerial.end();
  
  // Tell to the program that it's busy writing on SD Card
  changeState(writingOnSDCardState);
  
  // Check if there was an plug off into the system
  readActualNameOfFileOnEEprom();
  
  // Ask if should change the file in which it's actually writing
  askIfShouldChangeTheFile();
  
  // Read sensors values from Analog Inputs
  readSensorsValues();
  
  if(saveData == canSave ){
   // Prepare the data to write on SD and Upload to CLoud
    handlingDataReceived();
    saveData = 0;
  }
  saveData ++;
  
  // Restart some values needed to enable reading from serial
  prepareToReadSerialAgain();
  
}

// Method to change the current state of micro-controller
void changeState(byte state){
  
  globalState=state;
}

// Method to restart the Serial Reading

void prepareToReadSerialAgain(){

  // Switch flag to control the reading
  shouldStartReading=false;
  
  // Write on EEProm the name of the file where it's actually saving data to continue writing on that file in case that the system plugs off
  saveActualNameOfFileOnEEprom();
  
  // Method to clean buffers 
  cleanBuffers();

  // Tell to the program that it's not writing on SD Card anymore
  changeState(readingKarelDataState);
  
  // Restart the Serial Listening 
  mySerial.begin(9600);

}

// Method to Discard Message

void discardMessage(){
  
  Serial.println("ERRONEA");
    
  // Stop the Serial Communication
  mySerial.end();
    
  // Clean the buffer
  for (int h=0;h<index;h++)
  {
    myBuffer[h]='/0';
  }
    
  // Restart the index
  index=0;
    
  // Restart the Serial Communication
  mySerial.begin(9600);
  
}
// Classify the data to write on SD
void handlingDataReceived(){
  
   // Clean the Buffer and Pass the info to another Vector
   for (int i=0;i<index;i++)
   {
     
    // Method to Write a Karel data on the SD byte per byte
    writeKarelDataOnSDWithComma(myBuffer[i]);
       
    // Check if Character Received Was ACK, which is the last character Karel Serial data and it's 6 in DECIMAL format. If it does, we continue to write the following stuff on SD
    if ((myBuffer[i]==characterOfEnd) && (i==bufferSize-1)){
          
      // Write Sensors Values on SD read from Analog Inputs
      writeSensorsValuesOnSD();
          
      // Write MilliSeconds On SD
      writeMilliSecondsOnSD();
      
      // Hold Bytes Values From Karel Data
      holdKarelData();
      
    }
     
   }
   
   // Debug to check the current version of the file where software is saving
   Serial.println(nameWithFormat);
   
}

// Method to write data on SD when called
void writeKarelDataOnSDWithComma(char byteWithCommaReceived){
  
  
  // Open the file. Note that only one file can be open at a time, so you have to close this one before opening another the FileSystem card is mounted at the following "/mnt/FileSystema1"
  File dataFile = SD.open(nameWithFormat, FILE_WRITE);
  
  // if the file is available, write to it:
  if (dataFile) {
    
    // Flag to physically know if there is any mistake or if it's writing correctly
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
           
    // Write Data on SD
    dataFile.print((byte)byteWithCommaReceived,HEX);
    
    // Write Separation among bytesF
    dataFile.print(",");
    
    // Close the File
    dataFile.close();
    
    // Flag to physically know if there is any mistake or if it's writing correctly
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    
  }
  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

// Method to write MilliSeconds since Software Started on SD
void writeMilliSecondsOnSD(void){
  
  // Open the file. Note that only one file can be open at a time, so you have to close this one before opening another the FileSystem card is mounted at the following "/mnt/FileSystema1"
  File dataFile = SD.open(nameWithFormat, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    
  // Write milliseconds at which the message was registered
  
  // Separation Message between milliseconds and Messages
  dataFile.println("MilliSeconds");
  
  // Read milliseconds since working
  milliSeconds=millis();
  
  // Write milliseconds on SD
  dataFile.println(milliSeconds);
  
  // Real Time Debug 
  Serial.println("Milliseconds Are");
  Serial.println(milliSeconds);
  
  // Complete String Separation Message
  dataFile.println("Nueva Trama");
  
  // Close the File
  dataFile.close();
  
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
  
}

// Method to write Sensor values on SD from Analog Reads
void writeSensorsValuesOnSD(void){
  
  // Open the file. Note that only one file can be open at a time, so you have to close this one before opening another the FileSystem card is mounted at the following "/mnt/FileSystema1"
  File dataFile = SD.open(nameWithFormat, FILE_WRITE);
  
  // if the file is available, write to it:
  if (dataFile) {
  
  // Write Sensors Values Separated By Commas
  dataFile.print(sensor1Value, HEX); dataFile.print(","); dataFile.print(sensor2Value, HEX); dataFile.print(","); dataFile.print(sensor3Value, HEX); dataFile.print(","); dataFile.print(sensor4Value, HEX); dataFile.print(","); dataFile.print(sensor5Value, HEX); dataFile.print(","); dataFile.println(sensor6Value, HEX);
  
  // Close the File
  dataFile.close();
  
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

// Ask if there is a need to change the file to write data on SD
void askIfShouldChangeTheFile(){
  
  // Increase the counter to know how many data has been written
  firstScaler++;
  
  // Check if there is a need to change the file to write on
  if ((firstScaler>=numberOfRawsInExcel) || (indexToChangeFileName==0))
  {
    // Set the value read on EEprom to value which controls the name of the file for it to continue in the file it was before plug off
    indexToChangeFileName=indexReadFromEEprom;
    
    // Go to method to change the file
    changeTheFileToWriteOn();
    
    // Restart the value
    firstScaler=0;
  }
}

// Method to write the file in another location when the actual is full
void changeTheFileToWriteOn(){

  // Process to change the file name increasing just one number in the name every time the actual file is full
  
  // Increase the number of the filename and convert it into a char
  indexToChangeFileName++;

  // Convert int to char the indexToChangeFileName
  String numberOfFileString;
  numberOfFileString=String(indexToChangeFileName);
  numberOfFileString.toCharArray(numberOfFile,5);
  
  // Write together number of File and root
  strcpy(nameWithFormat, "F");
  
  // Put together the number of the file and the F key 
  strcat(nameWithFormat, numberOfFile);

  // Put together the format ".xls" and the name of File in variable "nameWithFomat"
  strcat(nameWithFormat, formatOfFile);
  
}

// Method to read sensors data from analog inputs
void readSensorsValues(void){
  
  // Read & Calculate the real temperature value according to ADC Scale read from analog inputs and Alloc the values
  sensor1Value=calculateRealTemperatureValue(analogRead(sensor1Pin));
  sensor2Value=calculateRealTemperatureValue(analogRead(sensor2Pin));

  Serial.println("Analog Channels Reading");
  Serial.println("Analog Channels Reading");
  Serial.println(analogRead(sensor1Pin));
  Serial.println(analogRead(sensor1Pin));
  
  Serial.println("Valor de Sensores");
  
  // Real time debugging printing
  Serial.println(sensor1Value);
  Serial.println(sensor2Value);

}

// Method to hold the Karel data needed in global variables
void holdKarelData(){

  // Assign to global variables
  
  // Outputs Register
  karelData1=myBuffer[1];
  // Temperature Register Signed Conversion
  byte temporalKarelData2=myBuffer[4];
  if(temporalKarelData2<=50){
    karelData2=temporalKarelData2;}
  else{
    karelData2=(temporalKarelData2-255);}
  // AR Register Signed Conversion
  word temporalKarelData3=word(myBuffer[5],myBuffer[6]);
  if(temporalKarelData3<32000){
    karelData3=word(myBuffer[5],myBuffer[6]);}
  else{
    karelData3=(temporalKarelData3-65535);}
  // T Accumulated Register
  karelData4=word(myBuffer[15],myBuffer[16]);
  // Set Level Register
  karelData5=myBuffer[2];
  
  Serial.println("Valor de Karel Data");
  
  // Real time debugging 
  Serial.println(karelData1);
  Serial.println(karelData2);
  Serial.println(karelData3);
  Serial.println(karelData4);
  Serial.println(karelData5);
  
}

// Method to Calculate and return the real temperature value according to ADC Scale Value
double calculateRealTemperatureValue(double ADCValue){
  
  // Process to calculate the value in temperature
  double measuredVoltage=sensorFeedVoltaje*ADCValue/1024;
  double resistenceValue=(10)*(sensorFeedVoltaje-measuredVoltage)/measuredVoltage;
  double temperatureValue=((-0.0983)*pow(log(resistenceValue),3))+((1.5293)*pow(log(resistenceValue),2))+((-25.848)*log(resistenceValue))+49.746;
    
  // Return Value
  return temperatureValue;
}

// Method to send data to modulo wifi
void sendPackageDataToWifiModule(){

  // Read Sensor Values Before Uploading
  readSensorsValues();

  // Debug Comments
  Serial.println("Sending Data to Wifi Module");
  Serial.println("Sending Data to Wifi Module");
  Serial.println("Sending Data to Wifi Module");
  
  // Send Karel Data Package to Wifi Module in following order: Outputs Register-Temperature Register-AR Register-T Accumulated Register-Sensor1-Sensor2
  
  // Build a string as needed to send the data package to wifi module
  String packageDataToSend = "sendData(" + String(karelData1) + ',' + String(karelData2) + ',' + String(karelData3) + ',' + String(karelData4) + ',' + String(karelData5) + ',' + String(sensor1Value) + ',' + String(sensor2Value) + ')';
  
  // Send Data by Serial Interface
  mySerial.println(packageDataToSend);
  
  // Debugging String
  Serial.println(packageDataToSend);
  
}

// Method to send reset command to modulo wifi
void sendResetCommandToWifiModule(){
  
  // Debug Comments
  Serial.println("Sending Reset To Wifi");
  Serial.println("Sending Reset To Wifi");
  
  // Send Reset Command (R) 
  mySerial.println(resetComand);
  
  // Debug
  Serial.println(resetComand);
  
}

// Method to clean the buffers and get it ready to keep receiving data
void cleanBuffers(){
  
  // Run the vector
   for (int j=0;j<index;j++)
   {
     // Clean the buffer
     myBuffer[j]='/0';
   }    
   
  // Set the index of the vector to the position 0
  index=0;
}

// Save the index of the name of file in which it's actually saving data to retake saving in this file in case of plugging off
void saveActualNameOfFileOnEEprom(){
  
  EEPROMWriteInt(byteUsedForEEprom,indexToChangeFileName);
}

// Read the last value of index to change file to check if there was a plug off into the system
void readActualNameOfFileOnEEprom(){
  
  indexReadFromEEprom=EEPROMReadInt(byteUsedForEEprom);
  //Serial.println(indexReadFromEEprom);
}


//This function will write a 2 byte integer to the EEprom at the specified address and address + 1
void EEPROMWriteInt(int p_address, int p_value)
{
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);

  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
}

//This function will read a 2 byte integer from the EEprom at the specified address and address + 1
unsigned int EEPROMReadInt(int p_address)
{
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);

  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

// Method to check for State of Analog Pin
byte checkDigitalInputByAnalogReading(byte analogPinNumber){
  
  // Debug
  Serial.print("Analog Value is:");
  Serial.println(analogRead(analogPinNumber));
  
  // Check if the digital input is being a High or a Low according to ADCReading Value
  if ((analogRead(analogPinNumber))>=ADCLevelValueForDigitalInput){
    
    // Debug
    Serial.println("Pin is High");
    
    return(HIGH);
    
  }else{
    
    // Debug
    Serial.println("Pin is Low");
    
    return(LOW);
  }
  
} 
// Method which IRQ interrupt lands
void eventIRQHandling(){
  
  // Turns interruption down to avoid getting in here repeatedly for just one push 
  detachInterrupt(0);
  
  // Accumulate a pending task for when it's available to do it
  pendingTasks[1]=sendingResetFlagToWifiTask;
  
  // Debug
  Serial.println("Interrumpio IRQ");
}

// Method to update all data to cloud
void timeInterruptHandling(){

  // Flag to notify the software that something is pending
  pendingTasks[0]=sendingPackageDataToWifiTask;
    
  Serial.println("Time to Send Data Package");
  Serial.println("Time to Send Data Package");
  
}
//// Cast the variable
//dataString = String(number);
//
//// Add the "," to separate
//dataString = dataString + ",";

// Read sensors values from cloud to debug
//  Serial.println(ubiclient.get_value(idSensor1));
//    Serial.println(ubiclient.get_value(idSensor2));
//    Serial.println(ubiclient.get_value(idSensor3));

