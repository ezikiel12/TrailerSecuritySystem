/*  Trailer securty system code

     Hardware: Arduino Pro Mini (5V)
     SIM7000A dev board
     Hologram IOT SIM

     Written by: Justin Britt (2022)
 ****************************************************/

#include "Adafruit_FONA.h" // https://github.com/botletics/SIM7000-LTE-Shield/tree/master/Code

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
// Required for Serial on Zero based boards
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

// Define *one* of the following lines:
//#define SIMCOM_2G // SIM800/808/900/908, etc.
//#define SIMCOM_3G // SIM5320
#define SIMCOM_7000
//#define SIMCOM_7070
//#define SIMCOM_7500
//#define SIMCOM_7600

// For botletics SIM7000 shield
#define FONA_PWRKEY 6
#define FONA_RST 7
//#define FONA_DTR 8 // Connect with solder jumper
//#define FONA_RI 9 // Need to enable via AT commands
#define FONA_TX 11 // Microcontroller RX
#define FONA_RX 10 // Microcontroller TX
//#define T_ALERT 12 // Connect with solder jumper

// For botletics SIM7500 shield
//#define FONA_PWRKEY 6
//#define FONA_RST 7
////#define FONA_DTR 9 // Connect with solder jumper
////#define FONA_RI 8 // Need to enable via AT commands
//#define FONA_TX 11 // Microcontroller RX
//#define FONA_RX 10 // Microcontroller TX
////#define T_ALERT 5 // Connect with solder jumper

// Definitions for IO
#define doorSense 2 //magnetic switch input for trailer door switches
#define armSwitch 3 //Physical switch for arming and disarming system
#define armStateLED 4  // LED for arm state. 

// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines
// and uncomment the HardwareSerial line


#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);

// Use the following line for ESP8266 instead of the line above (comment out the one above)
//SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX, false, 256); // TX, RX, inverted logic, buffer size

SoftwareSerial *fonaSerial = &fonaSS;

// Hardware serial is also possible!
//HardwareSerial *fonaSerial = &Serial1;

// For ESP32 hardware serial use these lines instead
//#include <HardwareSerial.h>
//HardwareSerial fonaSS(1);

// Use this for 2G modules
#ifdef SIMCOM_2G
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

// Use this one for 3G modules
#elif defined(SIMCOM_3G)
Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

// Use this one for LTE CAT-M/NB-IoT modules (like SIM7000)
// Notice how we don't include the reset pin because it's reserved for emergencies on the LTE module!
#elif defined(SIMCOM_7000) || defined(SIMCOM_7070) || defined(SIMCOM_7500) || defined(SIMCOM_7600)
Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();
#endif

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;
char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
float latitude, longitude, speed_kph, heading, altitude, second; //store lat/long values
char latBuff[12], longBuff[12], speedBuff[12];
void setup() {
  //  while (!Serial);

  pinMode(FONA_RST, OUTPUT);
  digitalWrite(FONA_RST, HIGH); // Default state
  pinMode (doorSense, INPUT_PULLUP);
  pinMode (armSwitch, INPUT_PULLUP);
  pinMode (armStateLED, OUTPUT);


  // Turn on the module by pulsing PWRKEY low for a little bit
  // This amount of time depends on the specific module that's used
  //fona.powerOn(FONA_PWRKEY); // Power on the module

  Serial.begin(9600);
  Serial.println(F("SMS Response Test"));
  Serial.println(F("Initializing....(May take several seconds)"));

  // Software serial:
  fonaSS.begin(115200); // Default SIM7000 shield baud rate

  Serial.println(F("Configuring to 9600 baud"));
  fonaSS.println("AT+IPR=9600"); // Set baud rate
  delay(100); // Short pause to let the command run
  fonaSS.begin(9600);
  if (! fona.begin(fonaSS)) {
    Serial.println(F("Couldn't find FONA"));
    while (1); // Don't proceed if it couldn't find the device
  }

  // Hardware serial:
  /*
    fonaSerial->begin(115200); // Default SIM7000 baud rate

    if (! fona.begin(*fonaSerial)) {
    DEBUG_PRINTLN(F("Couldn't find SIM7000"));
    }
  */

  type = fona.type();
  Serial.println(F("FONA is OK"));
  Serial.print(F("Found "));
  switch (type) {
    case SIM800L:
      Serial.println(F("SIM800L")); break;
    case SIM800H:
      Serial.println(F("SIM800H")); break;
    case SIM808_V1:
      Serial.println(F("SIM808 (v1)")); break;
    case SIM808_V2:
      Serial.println(F("SIM808 (v2)")); break;
    case SIM5320A:
      Serial.println(F("SIM5320A (American)")); break;
    case SIM5320E:
      Serial.println(F("SIM5320E (European)")); break;
    case SIM7000:
      Serial.println(F("SIM7000")); break;
    case SIM7070:
      Serial.println(F("SIM7070")); break;
    case SIM7500:
      Serial.println(F("SIM7500")); break;
    case SIM7600:
      Serial.println(F("SIM7600")); break;
    default:
      Serial.println(F("???")); break;
  }

  // Print module IMEI number.
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: "); Serial.println(imei);
  }

  // Set modem to full functionality
  fona.setFunctionality(1); // AT+CFUN=1


  // Configure a GPRS APN, username, and password.
  // You might need to do this to access your network's GPRS/data
  // network.  Contact your provider for the exact APN, username,
  // and password values.  Username and password are optional and
  // can be removed, but APN is required.

  //fona.setNetworkSettings(F("your APN"), F("your username"), F("your password"));
  //fona.setNetworkSettings(F("EMNIFY")); // For AT&T IoT SIM card
  //fona.setNetworkSettings(F("telstra.internet")); // For Telstra (Australia) SIM card - CAT-M1 (Band 28)
  fona.setNetworkSettings(F("hologram")); // For Hologram SIM card

  // Optionally configure HTTP gets to follow redirects over SSL.
  // Default is not to follow SSL redirects, however if you uncomment
  // the following line then redirects over SSL will be followed.
  //fona.setHTTPSRedirect(true);

  /*
    // Settable stuff
    fona.setPreferredMode(38); // Use LTE only, not 2G
    fona.setPreferredLTEMode(1); // Use LTE CAT-M only, not NB-IoT
    fona.setOperatingBand("CAT-M", 12); // AT&T uses band 12
    fona.setOperatingBand("CAT-M", 13); // Verizon uses band 13
    fona.enableRTC(true);

    fona.enableSleepMode(true);
    fona.set_eDRX(1, 4, "0010");
    fona.enablePSM(true);

    // Set the network status LED blinking pattern while connected to a network (see AT+SLEDS command)
    fona.setNetLED(true, 2, 64, 3000); // on/off, mode, timer_on, timer_off
    fona.setNetLED(false); // Disable network status LED
  */
  //fona.setOperatingBand("CAT-M", 12);
  //fona.setPreferredMode(38);
  fonaSerial->print("AT+CMGD=4");        //clear sim card SMS memory
  fonaSerial->print("AT+CNMI=2,1\r\n");  // Set up the FONA to send a +CMTI notification when an SMS is received


  Serial.println("FONA Ready");
  delay(8000);

}


char fonaNotificationBuffer[64];  //for notifications from the FONA
char smsBuffer[250];
char callerIDbuffer[32];  //store the SMS sender number in here
const char* myPhone = "+15302623096"; //phone number to send alert notifications

uint8_t slot = 0;            //this will be the slot number of the SMS
uint8_t armStatus = 1;
uint8_t runGPS = 0;
uint8_t checkSpeedInterval = 5; // time * 2 between checking GPS speed
uint8_t speedThresh = 1; //kph trailer moves before sending notification
uint8_t armSwitchStatus = 1;
const char* armMessage = "arm"; // Arming SMS command
const char* disarmMessage = "disarm"; //Disarm SMS command
const char* GPSmessage = "location";  //get GPS location command
void loop() {


  checkSMS();  //check for incoming SMS
  //checkArmSwitch(); //check state of arm switch.
  digitalWrite(armStateLED, LOW); //LED off while disarmed


  if (strcmp(smsBuffer, armMessage) == 0) {   // Check for "arm" sms
    armStatus = 1;
    clearSMSmem();
  }

  else if (strcmp(smsBuffer, disarmMessage) == 0) { // Check for "disarm" sms
    armStatus = 0;
    //sendText("System is disarmed");
    clearSMSmem();
  }
  else if (strcmp(smsBuffer, GPSmessage) == 0) { //Check for "location" sms
    runGPS = 1;
    clearSMSmem();
  }
  else {

    if (armSwitchStatus == 1 && armStatus == 1) {
      uint8_t counter = 0; //used for checkSpeedInterval
      for (int count = 0; count >= 5; count++) {
        digitalWrite(armStateLED, HIGH);
        delay(200);
        digitalWrite(armStateLED, LOW);
        delay(200);
      }
      Serial.println("System is now armed");
      while (armStatus == 1 && armSwitchStatus == 1) {
        //checkArmSwitch();   //check state of arm switch
        checkSMS();         //check for new SMS
        digitalWrite(armStateLED, HIGH); //LED on while armed
        counter++;
        Serial.println("*****************************");
        Serial.println("System Status: Armed");
        if (CheckDoorStatus() == 1) { //check if door switches are triggered
          armStatus = 0;
          clearSMSmem();   //clear SMS buffer so it doesn't rearm automatically
        }

        else if (strcmp(smsBuffer, disarmMessage) == 0) { //check for "disarm" message
          armStatus = 0;
          //sendText("System has been disarmed");
          Serial.println("System has been disarmed");
          clearSMSmem();
        }
        else if (counter >= checkSpeedInterval) {   // check if trailer is moving
          getSpeed();                                //after specified amount of time
          counter = 0;
        }
        else
        {
          if (strcmp(smsBuffer, GPSmessage) == 0) { //check GPS location if "location"
            getGPS();                               //SMS is received
            armStatus = 1;
            clearSMSmem();
          }
        }
      }
    }
  }
  if (armStatus == 0 || armSwitchStatus == 0) {    //check GPS location even if system is not armed
    if (runGPS == 1) {
      getGPS();
      runGPS = 0;
      clearSMSmem();
    }
    else
    {
      Serial.println("*****************************");
      Serial.println("System Status: Disarmed");
      clearSMSmem();
      delay(2000);
    }
  }
}





// Send an SMS response
void sendText(const char* textMessage) {
  Serial.println("Sending reponse...");

  if (!fona.sendSMS(myPhone, textMessage)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("Sent!"));
  }

  // Delete the original message after it is processed.
  // Otherwise we will fill up all the slots and
  // then we won't be able to receive any more!
  if (fona.deleteSMS(slot)) {
    Serial.println(F("OK!"));
  } else {
    Serial.print(F("Couldn't delete SMS in slot ")); Serial.println(slot);
    fona.print(F("AT+CMGD=?\r\n"));
  }
}

uint8_t CheckDoorStatus() {
  Serial.println("Door status: Closed");
  delay(5000);
  if (digitalRead(doorSense) == LOW) { //change to low for final install!
    delay(10000); //give time to deactivate arm switch
    checkArmSwitch();
    if (armSwitchStatus == HIGH) {
      //sendText("Trailer door has opened!!!"); //mesage sent to phone
      Serial.println("*****************************");
      Serial.println("Trailer door opened. Notifying authorities");
      delay(10000); //delay until next text is sent
      //sendText("Trailer door has opened!!!"); //mesage sent to phone
      Serial.println("*****************************");
      Serial.println("Trailer door opened. Notifying authorities");
      delay(10000); //delay until next text is sent
      return 1;
    }
  }
  else
  {
    return 0;
  }
}

void checkSMS() {
  char* bufPtr = fonaNotificationBuffer;
  if (fona.available())      //any data available from the FONA?
  {
    int charCount = 0;
    //Read the notification into fonaInBuffer
    do  {
      *bufPtr = fona.read();
      Serial.write(*bufPtr);
      delay(1);
    } while ((*bufPtr++ != '\n') && (fona.available()) && (++charCount < (sizeof(fonaNotificationBuffer) - 1)));

    //Add a terminal NULL to the notification string
    *bufPtr = 0;

    //Scan the notification string for an SMS received notification.
    //  If it's an SMS message, we'll get the slot number in 'slot'
    if (1 == sscanf(fonaNotificationBuffer, "+CMTI: " FONA_PREF_SMS_STORAGE ",%d", &slot)) {
      Serial.print("slot: "); Serial.println(slot);

      // Retrieve SMS sender address/phone number.
      if (! fona.getSMSSender(slot, callerIDbuffer, 31)) {
        Serial.println("Didn't find SMS message in slot!");
      }
      Serial.print(F("FROM: ")); Serial.println(callerIDbuffer);

      // Retrieve SMS value.
      uint16_t smslen;
      if (fona.readSMS(slot, smsBuffer, 250, &smslen)) { // pass in buffer and max len!
        Serial.println(smsBuffer);
      }
    }
  }
}

void clearSMSmem() { //clear the SMS buffer
  memset(smsBuffer, 0, sizeof(smsBuffer));
}

void getGPS() { //get GPS location and send text
  // Perform first-time GPS/data setup
  //Enable GPS
  while (!fona.enableGPS(true)) {
    Serial.println(F("Failed to turn on GPS, retrying..."));
    delay(2000); // Retry every 2s
  }
  Serial.println(F("Turned on GPS!"));
  delay(2000);
  //Get info from GPS, retry every 2 seconds if failed
  while (!fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude)) {
    Serial.println(F("Failed to get GPS location, retrying..."));
    delay(2000); // Retry every 2s
  }
  Serial.println(F("Found 'eeeeem!"));
  Serial.println(F("---------------------"));
  Serial.print(F("Latitude: ")); Serial.println(latitude, 6);
  Serial.print(F("Longitude: ")); Serial.println(longitude, 6);
  Serial.print(F("Speed: ")); Serial.println(speed_kph);
  String lat = String(latitude, 6);
  String lon = String(longitude, 6);
  String spd = String(speed_kph);
  //String sms = ("Longitude: " + lon + " Latitude: " + lat + " Speed: " + spd);
  String sms2 ("http://maps.google.com/maps?z=12&t=m&q=loc:" + lat + "+" + lon);
  //char* locationSMS = sms.c_str();
  char* locGoogleMaps = sms2.c_str();
  //Serial.println(sms);
  Serial.println(locGoogleMaps);
  delay(100);
  //sendText(locationSMS);
  sendText(locGoogleMaps);

  fona.enableGPS(false);
}

void getSpeed() { //check GPS for current speed, send text if moving
  while (!fona.enableGPS(true)) {
    Serial.println(F("Failed to turn on GPS, retrying..."));
    delay(2000); // Retry every 2s
  }
  Serial.println(F("Turned on GPS!"));
  delay(2000);
  //Get info from GPS, retry every 2 seconds if failed
  while (!fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude)) {
    Serial.println(F("Failed to get GPS location, retrying..."));
    delay(2000); // Retry every 2s
  }
  if (speed_kph > speedThresh) {  //if trailer is moving faster than speedThresh
    Serial.println("Trailer is moving");
    //sendText("Trailer is currently moving");
  }
  else
  {
    Serial.println(speed_kph);
    Serial.println("Trailer is parked");
  }
}

void checkArmSwitch() {     //check if arm switch has changed state
  if (digitalRead(armSwitch) == LOW)
  {
    armSwitchStatus = 1;
  }
  else {
    armSwitchStatus = 0;
  }
}
