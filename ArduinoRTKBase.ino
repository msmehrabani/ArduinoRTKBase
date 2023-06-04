
#include <Wire.h> //Needed for I2C to GNSS
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;


SFE_UBLOX_REDUCED_PROG_MEM

//To save eprom space do uncomment the following lines in: SparkFun_u-blox_GNSS_Arduino_Library.h
//#define SFE_UBLOX_REDUCED_PROG_MEM // Uncommenting this line will delete the minor debug messages to save memory
//#define SFE_UBLOX_DISABLE_AUTO_NMEA // Uncommenting this line will disable auto-NMEA support to save memory

RF24 radio(8, 7); // CE, CSN

uint8_t addresses[][6] = { "Base", "Rover" };


static float accuracy = 5.000; //2.0
static int   mintime = 10; //300
static uint8_t rtk_frame_data[1024] = "";
static uint8_t rtk_frame_size = 0 ;


struct Data_Package {
  byte    data_type = 0;
  byte    data_size = 0;
  byte    data_appendnext = 0;
  uint8_t data[28];
};

Data_Package radio_data;

void setup()
{

  memset(rtk_frame_data,0,1024);
  
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println(F("u-blox NEO-M8P-2 Base Station"));

  radio.begin();
  radio.openWritingPipe(addresses[1]);    // Rover
  radio.openReadingPipe(1, addresses[0]); // Base
  radio.setPALevel(RF24_PA_MIN);

  if(radio.isChipConnected()){
    Serial.println(F("RF24 Initialized!"));
  }else{
    Serial.println(F("Error: RF24 not connected!"));
  }
  

  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); // Ensure RTCM3 is enabled
  myGNSS.saveConfiguration(); //Save the current settings to flash and BBR
  
  while (Serial.available()) Serial.read(); //Clear any latent chars in serial buffer

  bool response;

  //Check if Survey is in Progress before initiating one
  // From v2.0, the data from getSurveyStatus (UBX-NAV-SVIN) is returned in UBX_NAV_SVIN_t packetUBXNAVSVIN
  // Please see u-blox_structs.h for the full definition of UBX_NAV_SVIN_t
  // You can either read the data from packetUBXNAVSVIN directly
  // or can use the helper functions: getSurveyInActive; getSurveyInValid; getSurveyInObservationTime; and getSurveyInMeanAccuracy
  
  response = myGNSS.getSurveyStatus(2000); //Query module for SVIN status with 2000ms timeout (request can take a long time)

  if (response == false) // Check if fresh data was received
  {
    Serial.println(F("Failed to get Survey In status. Freezing..."));
    while (1); //Freeze
  }

  if (myGNSS.getSurveyInActive() == true) // Use the helper function
  {
    Serial.print(F("Survey already in progress."));
  }
  else
  {
    //Start survey
    response = myGNSS.enableSurveyMode(mintime, accuracy); //Enable Survey in, 300 seconds, 3.5m
    if (response == false)
    {
      Serial.println(F("Survey start failed. Freezing..."));
      while (1);
    }
    Serial.println(F("Survey started. This will run until 300s has passed and less than 2m accuracy is achieved."));
  }

  while (Serial.available()) Serial.read(); //Clear buffer
  
  //Begin waiting for survey to complete
  
  while (myGNSS.getSurveyInValid() == false) // Call the helper function
  {
    if (Serial.available())
    {
      byte incoming = Serial.read();
      if (incoming == 'x')
      {
        //Stop survey mode
        response = myGNSS.disableSurveyMode(); //Disable survey
        Serial.println(F("Survey stopped"));
        break;
      }
    }
    
    // From v2.0, the data from getSurveyStatus (UBX-NAV-SVIN) is returned in UBX_NAV_SVIN_t packetUBXNAVSVIN
    // Please see u-blox_structs.h for the full definition of UBX_NAV_SVIN_t
    // You can either read the data from packetUBXNAVSVIN directly
    // or can use the helper functions: getSurveyInActive; getSurveyInValid; getSurveyInObservationTime; and getSurveyInMeanAccuracy
    response = myGNSS.getSurveyStatus(2000); //Query module for SVIN status with 2000ms timeout (req can take a long time)
    if (response == true) // Check if fresh data was received
    {
      Serial.print(F("Press x to end survey - "));
      Serial.print(F("Time elapsed: "));
      Serial.print((String)myGNSS.getSurveyInObservationTime());

      Serial.print(F(" Accuracy: "));
      Serial.print((String)myGNSS.getSurveyInMeanAccuracy());
      Serial.println();
    }
    else
    {
      Serial.println(F("SVIN request failed"));
    }

    delay(1000);
  }
  Serial.println(F("Survey valid!"));

  response = true;
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_I2C, 1); //Enable message 1005 to output through I2C port, message every second
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1077, COM_PORT_I2C, 1);
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1087, COM_PORT_I2C, 1);
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_I2C, 10); //Enable message every 10 seconds

  if (response == true)
  {
    Serial.println(F("RTCM messages enabled"));
  }
  else
  {
    Serial.println(F("RTCM failed to enable. Are you sure you have an NEO-M8P?"));
    while (1); //Freeze
  }

  Serial.println(F("Base survey complete! RTCM now broadcasting."));
  
}

void loop()
{
  
  myGNSS.checkUblox(); //See if new data is available. Process bytes as they come in.
  delay(250); //Don't pound too hard on the I2C bus

  if(rtk_frame_size!=0){
      Serial.print("RTCM Message size: ");
      Serial.print(rtk_frame_size);
      Serial.print(" data: ");     
      for(int i=0;i<rtk_frame_size;i++){
        if (rtk_frame_data[i] < 0x10) Serial.print(F("0"));  //WTF work around for eroneus HEX printing
        Serial.print(rtk_frame_data[i], HEX);
      }
       Serial.println("");

      int index = 0;
      while(rtk_frame_size>0){
        radio_data.data_type=1;
        if(rtk_frame_size>28){
            radio_data.data_appendnext=1;
            radio_data.data_size=28;
            memcpy(radio_data.data, &rtk_frame_data[index], 28);
            rtk_frame_size-=28;
            index+=28;
        }else{
            radio_data.data_appendnext=0;
            radio_data.data_size=rtk_frame_size;
            memcpy(radio_data.data, &rtk_frame_data[index], rtk_frame_size);
            index+=rtk_frame_size;
            rtk_frame_size-=rtk_frame_size;
        }
        radio.stopListening();
        radio.write(&radio_data, sizeof(Data_Package));
        delay(5);
      }
      radio.startListening();
      delay(5);
      memset(rtk_frame_data,0,1024);
    }

    //delay(5);
    //radio.stopListening();
    //const char text[] = "Hello World from RTK Base";
    //radio.write(&text, sizeof(text));
    //delay(5);
    //radio.startListening();
/*    
  if (radio.available()) {
    char text_incoming[32] = "";
    radio.read(&text_incoming, sizeof(text_incoming));
    Serial.println(text_incoming);
  }
*/  
}

//This function gets called from the SparkFun u-blox Arduino Library.
//As each RTCM byte comes in you can specify what to do with it
//Useful for passing the RTCM correction data to a radio, Ntrip broadcaster, etc.
void SFE_UBLOX_GNSS::processRTCM(uint8_t incoming)
{
  //Let's just pretty-print the HEX values for now
 
   static uint8_t rtk_frame_buffer[1024];
   static int num_bytes=0;
   if (myGNSS.rtcmFrameCounter  == 1) {      
      if(num_bytes>0){
        memset(rtk_frame_data,0,1024);
        rtk_frame_size=num_bytes;
        memcpy(rtk_frame_data, rtk_frame_buffer, 1024);  
      }
      //reset 
      memset(rtk_frame_buffer,0,1024);
      num_bytes=0;
  }
  if (myGNSS.rtcmFrameCounter < 1024) {
    rtk_frame_buffer[myGNSS.rtcmFrameCounter-1]=incoming;
    num_bytes++;
  }else{
    Serial.print("Ooops this RTCM message is too long!");
  }
}
