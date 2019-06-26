/*
 Example of reading the disk properties on OpenLog
 By: Nathan Seidle
 SparkFun Electronics
 Date: September 22nd, 2013
 License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

 This is an example of issuing the 'disk' command and seeing how big the current SD card is.
 
 Connect the following OpenLog to Arduino:
 RXI of OpenLog to pin 2 on the Arduino
 TXO to 3
 GRN to 4
 VCC to 5V
 GND to GND
 
 This example code assumes the OpenLog is set to operate at 9600bps in NewLog mode, meaning OpenLog 
 should power up and output '12<'. This code then sends the three escape characters and then sends 
 the commands to create a new random file called log###.txt where ### is a random number from 0 to 999.
 The example code will then read back the random file and print it to the serial terminal.
 
 This code assume OpenLog is in the default state of 9600bps with ASCII-26 as the esacape character.
 If you're unsure, make sure the config.txt file contains the following: 9600,26,3,0
 
 Be careful when sending commands to Serial3. println() sends extra newline characters that 
 cause problems with the command parser. The new v2.51 ignores \n commands so it should be easier to 
 talk to on the command prompt level. This example code works with all OpenLog v2 and higher.
 
 */

//#include <SoftwareSerial.h>

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//Connect TXO of OpenLog to pin 3, RXI to pin 2
//SoftwareSerial OpenLog(3, 2); //Soft RX on 3, Soft TX out on 2
//SoftwareSerial(rxPin, txPin)

int resetOpenLog = 16; //This pin resets Serial3. Connect pin 4 to pin GRN on Serial3.
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

int statLED = 13;

float dummyVoltage = 3.50; //This just shows to to write variables to OpenLog

void setup() {                
  pinMode(statLED, OUTPUT);
  SerialUSB.begin(0);

  setupOpenLog(); //Resets logger and waits for the '<' I'm alive character
  SerialUSB.println("OpenLog online");

  delay(5000);
}

void loop() {

  randomSeed(analogRead(A0)); //Use the analog pins for a good seed value
  int fileNumber = random(999); //Select a random file #, 0 to 999
  char fileName[12]; //Max file name length is "12345678.123" (12 characters)
  sprintf(fileName, "log%03d.txt", fileNumber);

  gotoCommandMode(); //Puts OpenLog in command mode
  createFile(fileName); //Creates a new file called log###.txt where ### is random

  SerialUSB.print("Random file created: ");
  SerialUSB.println(fileName);

  //Write something to OpenLog
  Serial3.println("Hi there! How are you today?");
  Serial3.print("Voltage: ");
  Serial3.println(dummyVoltage);
  dummyVoltage++;
  Serial3.print("Voltage: ");
  Serial3.println(dummyVoltage);

  SerialUSB.println("Text written to file");
  SerialUSB.println("Reading file contents:");
  SerialUSB.println();

  //Now let's read back
  gotoCommandMode(); //Puts OpenLog in command mode
  readFile(fileName); //This dumps the contents of a given file to the serial terminal

  //Now let's read back
  readDisk(); //Check the size and stats of the SD card

  SerialUSB.println();
  SerialUSB.println("File read complete");

  //Infinite loop
  SerialUSB.println("Yay!");
  while(1) {
    digitalWrite(13, HIGH);
    delay(250);
    digitalWrite(13, LOW);
    delay(250);
  }
}

//Setups up the software serial, resets OpenLog so we know what state it's in, and waits
//for OpenLog to come online and report '<' that it is ready to receive characters to record
void setupOpenLog(void) {
  pinMode(resetOpenLog, OUTPUT);
  Serial3.begin(9600);

  //Reset OpenLog
  digitalWrite(resetOpenLog, LOW);
  delay(100);
  digitalWrite(resetOpenLog, HIGH);

  //Wait for OpenLog to respond with '<' to indicate it is alive and recording to a file
  while(1) {
    if(Serial3.available())
      if(Serial3.read() == '<') break;
  }
}

//This function creates a given file and then opens it in append mode (ready to record characters to the file)
//Then returns to listening mode
void createFile(char *fileName) {

  //Old way
  Serial3.print("new ");
  Serial3.print(fileName);
  Serial3.write(13); //This is \r

  //New way
  //Serial3.print("new ");
  //Serial3.println(filename); //regular println works with OpenLog v2.51 and above

  //Wait for OpenLog to return to waiting for a command
  while(1) {
    if(Serial3.available())
      if(Serial3.read() == '>') break;
  }

  Serial3.print("append ");
  Serial3.print(fileName);
  Serial3.write(13); //This is \r

  //Wait for OpenLog to indicate file is open and ready for writing
  while(1) {
    if(Serial3.available())
      if(Serial3.read() == '<') break;
  }

  //OpenLog is now waiting for characters and will record them to the new file  
}

//Reads the contents of a given file and dumps it to the serial terminal
//This function assumes the OpenLog is in command mode
void readFile(char *fileName) {

  //Old way
  Serial3.print("read ");
  Serial3.print(fileName);
  Serial3.write(13); //This is \r

  //New way
  //Serial3.print("read ");
  //Serial3.println(filename); //regular println works with OpenLog v2.51 and above

  //The OpenLog echos the commands we send it by default so we have 'read log823.txt\r' sitting 
  //in the RX buffer. Let's try to not print this.
  while(1) {
    if(Serial3.available())
      if(Serial3.read() == '\r') break;
  }  

  //This will listen for characters coming from OpenLog and print them to the terminal
  //This relies heavily on the SoftSerial buffer not overrunning. This will probably not work
  //above 38400bps.
  //This loop will stop listening after 1 second of no characters received
  for(int timeOut = 0 ; timeOut < 1000 ; timeOut++) {
    while(Serial3.available()) {
      char tempString[100];
      
      int spot = 0;
      while(Serial3.available()) {
        tempString[spot++] = Serial3.read();
        if(spot > 98) break;
      }
      tempString[spot] = '\0';
      SerialUSB.write(tempString); //Take the string from OpenLog and push it to the Arduino terminal
      timeOut = 0;
    }

    delay(1);
  }

  //This is not perfect. The above loop will print the '.'s from the log file. These are the two escape characters
  //recorded before the third escape character is seen.
  //It will also print the '>' character. This is the OpenLog telling us it is done reading the file.  

  //This function leaves OpenLog in command mode
}

//Check the stats of the SD card via 'disk' command
//This function assumes the OpenLog is in command mode
void readDisk() {

  //Old way
  Serial3.print("disk");
  Serial3.write(13); //This is \r

  //New way
  //Serial3.print("read ");
  //Serial3.println(filename); //regular println works with OpenLog v2.51 and above

  //The OpenLog echos the commands we send it by default so we have 'disk\r' sitting 
  //in the RX buffer. Let's try to not print this.
  while(1) {
    if(Serial3.available())
      if(Serial3.read() == '\r') break;
  }  

  //This will listen for characters coming from OpenLog and print them to the terminal
  //This relies heavily on the SoftSerial buffer not overrunning. This will probably not work
  //above 38400bps.
  //This loop will stop listening after 1 second of no characters received
  for(int timeOut = 0 ; timeOut < 1000 ; timeOut++) {
    while(Serial3.available()) {
      char tempString[100];
      
      int spot = 0;
      while(Serial3.available()) {
        tempString[spot++] = Serial3.read();
        if(spot > 98) break;
      }
      tempString[spot] = '\0';
      SerialUSB.write(tempString); //Take the string from OpenLog and push it to the Arduino terminal
      timeOut = 0;
    }

    delay(1);
  }

  //This is not perfect. The above loop will print the '.'s from the log file. These are the two escape characters
  //recorded before the third escape character is seen.
  //It will also print the '>' character. This is the OpenLog telling us it is done reading the file.  

  //This function leaves OpenLog in command mode
}

//This function pushes OpenLog into command mode
void gotoCommandMode(void) {
  //Send three control z to enter OpenLog command mode
  //Works with Arduino v1.0
	byte rstArr[3] = { 26,26,26 };
  Serial3.write(rstArr,3);

  //Wait for OpenLog to respond with '>' to indicate we are in command mode
  while(1) {
    if(Serial3.available())
      if(Serial3.read() == '>') break;
  }
}
