/***************************************************


  WiFi code adapted from the Adafruit CC3000 Wifi Breakout & Shield

  Designed specifically to work with the Adafruit WiFi products:
  ----> https://www.adafruit.com/products/1469

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried, Kevin Townsend and Phil Burgess for
  Adafruit Industries.  BSD license, all text above must be included
  in any redistribution
 ****************************************************/

#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include "utility/debug.h"
//#include <string.h>

// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIVIDER); // you can change this clock speed but DI

#define WLAN_SSID       "lunas"        // cannot be longer than 32 characters!
#define WLAN_PASS       "ohigginssucre"
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2

#define TIMEOUT  3000

#define SEDER_HOST "monster.akualab.com"
#define SEDER_PORT 3000
#define SAMP_PERIOD 800 // sampling period in millisec
#define TIME_SERVER_PERIOD 86400000 // period to request time for server in millisec
#define NUM_SAMP_PER_MSG 5 // number of sample periods between REST messages
#define MAX_SENSORS 4 // needed to allocate array of sensors (better way?)

#define POST_DATA_ENDPOINT "/v0/data"
#define MAX_MSG_LEN 300

Adafruit_CC3000_Client client;
Adafruit_CC3000_Client getClient();

//const unsigned long
  //connectTimeout  = 15L * 1000L, // Max time to wait for server connection
  //responseTimeout = 15L * 1000L; // Max time to wait for data from server
unsigned int
  sampIndex       = 0;  // current sample index - range is 0 .. NUM_SAMP_PER_MSG - 1
unsigned long
  lastPolledTime  = 0L, // Last value retrieved from time server
  sketchTime      = 0L, // CPU milliseconds since last server query
  lastSampleTime  = 0L; // Time since last sample.
char sederPortStr[5];
char *crlf = "\r\n";
struct AnalogPin {
  byte pin;
  int values[NUM_SAMP_PER_MSG];
  char *name;
} ;

struct AnalogPin *sensors[MAX_SENSORS];
byte numSensors;
struct AnalogPin ir, us;
void sendData(const char * id, byte nsens, struct AnalogPin **sa, unsigned long t1, 
  unsigned long t2, unsigned int per, unsigned int nsamp);

//unsigned long timeValues[NUM_SAMP_PER_MSG];

int ledPin = 13;      // select the pin for the LED

void setup(void)
{
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  
  Serial.print(F("Free RAM: ")); Serial.println(getFreeRam(), DEC);
  Serial.println(F("Hello, SEDER!\n")); 

  displayDriverMode();
  
  Serial.println(F("\nInitialising the CC3000 ..."));
  if (!cc3000.begin()) {
    Serial.println(F("Unable to initialise the CC3000! Check your wiring?"));
    for(;;);
  }

  uint16_t firmware = checkFirmwareVersion();
  if ((firmware != 0x113) && (firmware != 0x118)) {
    Serial.println(F("Wrong firmware version!"));
    for(;;);
  }
  
  displayMACAddress();
  
  Serial.println(F("\nDeleting old connection profiles"));
  if (!cc3000.deleteProfiles()) {
    Serial.println(F("Failed!"));
    while(1);
  }

  /* Attempt to connect to an access point */
  char *ssid = (char *)WLAN_SSID;             /* Max 32 chars */
  Serial.print(F("\nAttempting to connect to ")); Serial.println(ssid);
  
  /* NOTE: Secure connections are not available in 'Tiny' mode! */
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while(1);
  }
   
  Serial.println(F("Connected!"));
  
  /* Wait for DHCP to complete */
  Serial.println(F("Request DHCP"));
  while (!cc3000.checkDHCP()) {
    delay(100); // ToDo: Insert a DHCP timeout!
  }

  /* Display the IP address DNS, Gateway, etc. */  
  while (!displayConnectionDetails()) {
    delay(1000);
  }
  
  /* Prepare SEDER */
   sprintf(sederPortStr, "%d", SEDER_PORT); // need string for the HTTP header.
   
   //strcpy(crlf, (char *)"\r\n");

  // declare the ledPin as an OUTPUT:
  //(ledPin, OUTPUT);  
  
  // create array of sensors
  ir.pin = A0;
  ir.name = (char *)"ir"; 
  us.pin = A2;
  us.name = (char *)"us";
  
  sensors[0] = &ir;
  sensors[1] = &us;
  
  numSensors = 2;
  if (numSensors > MAX_SENSORS) {
      Serial.println(F("Num sensors exceed max num sensors."));
      exit(numSensors);
  }
  
  
  // Initialize TCP client for sending data.
  client = getClient();
}


void loop(void) {
    
  // To reduce load on NTP servers, time is polled once per roughly 24 hour period.
  // Otherwise use millis() to estimate time since last query.  Plenty accurate.
  // TODO: write a func and run first time in setup instead of checking sketchTime == 0.
  // We need condition sampIndex == 0 so lastPolledTime is the same for all values in a message.
  if(sketchTime == 0 || ((millis() - sketchTime > TIME_SERVER_PERIOD) && sampIndex == 0)) {            // Time's up?
    //unsigned long t  = getTime(); // Query time server
    unsigned long t  = 111111L; // MOCK FOR TESTING REMEMBER TO CHANGE -- DEBUG
    if(t) {                       // Success?
      lastPolledTime = t;         // Save time
      sketchTime     = millis();  // Save sketch time of last valid time query
    }
  }
  
  if((millis() - lastSampleTime) >= SAMP_PERIOD) {

    lastSampleTime = millis();    
    
    // read values from sensors
    for(int i = 0; i < numSensors; i++) {
      AnalogPin *ap = sensors[i];
      ap->values[sampIndex] = analogRead(ap->pin);
    }
    
    // Update index after reading all values.
    sampIndex++;

    // check if time to send a message
    if(sampIndex == NUM_SAMP_PER_MSG) {
      sampIndex = 0;

      // Send data to server.
      Serial.println(F("*** SEND DATA ***"));

      // Time info needed to reconstract the absolute time for each sample.
      Serial.print(F("Base UNIX time in seconds: ")); // time in secs since epoch
      Serial.println(lastPolledTime);
      Serial.print(F("Delta UNIX time in milliseconds: "));
      Serial.println(millis() - sketchTime); // add this delta to get time for first sample
      Serial.print(F("Sampling Period in milliseconds: "));
      Serial.println(SAMP_PERIOD); // add samp period to get time for subsequent samples

      char id[] = "123";
      sendData(id, numSensors, sensors, lastPolledTime, millis() - sketchTime, SAMP_PERIOD, NUM_SAMP_PER_MSG);
    }
  }
}


/**************************************************************************/
/*!
    @brief  Sends data to server using HTTP.

    @note  
   
   {
     "id":"123",         // account id
     "t1":1394413060,    // base unix time in seconds
     "t2":57326,         // delta time to be added to base time in milliseconds
     "per":200,          // sample period in milliseconds
     "nsamp":100,        // num samples per measurement
     "nmeas":3,          // num measurements
     "m":[               // array of measurements
        {
           "k":"ir",       // name of measurement is "ir"
           "v":[11,22,33]  // values, num measurements is 3 in this case
        },
        {
           "k":"us",
           "v":[21,22,23]
        }
      ]
   }
  
*/
/**************************************************************************/
void sendData(char * id, byte nsens, struct AnalogPin **sa, unsigned long t1, 
  unsigned long t2, unsigned int per, unsigned int nsamp)
{
  
  // Print to serial console.
  for(int j=0; j < numSensors; j++) {
         struct AnalogPin *ap = sa[j];
        Serial.print(ap->name);
        Serial.print(" ==> ");
        for(int i = 0; i < (int)nsamp; i++) {
          Serial.print(ap->values[i]);
          Serial.print(", ");
        }
        Serial.println("");
  }
  
  Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);

  // Prepare JSON message
  char jm [MAX_MSG_LEN];
  int cx = 0;
  cx += snprintf (jm, MAX_MSG_LEN, "{\"id\":\"%s\", \"t1\":%lu, \"t2\":%lu, \"per\":%u, \"nsamp\":%u, \"nmeas\":%hu, \"m\":[",  
    id,t1,t2,per,nsamp,nsens);
  Serial.print("MSG Len: "); Serial.println(cx, DEC);
  
  for (int j = 0; j < nsens; j++) {
      struct AnalogPin *ap = sa[j];
      cx += snprintf (jm+cx, MAX_MSG_LEN-cx, "{\"k\":\"%s\", \"v\":[", ap->name);
      
      Serial.print("DEBUG A: "); Serial.println(cx, DEC);

      for (int i = 0; i < (int)nsamp; i++) {
        cx += snprintf (jm+cx, MAX_MSG_LEN-cx, "%d", ap->values[i]);
              Serial.print("DEBUG B: "); Serial.println(cx, DEC);

        if (i < (int)nsamp-1) 
          cx += snprintf(jm+cx, MAX_MSG_LEN-cx, ",");
        else
          cx += snprintf (jm+cx, MAX_MSG_LEN-cx, "]}");
      }
      if (j < nsens-1)
         cx += snprintf (jm+cx, MAX_MSG_LEN-cx, ",");
      else
         cx += snprintf (jm+cx, MAX_MSG_LEN-cx, "]}\n");
         
      Serial.print("MSG Len: "); Serial.println(cx, DEC);
  }
  
  //strcpy(jm,"TESTING!! XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"); // DEBUG
  int length = strlen(jm); 

  // Convert int to char array. Max int is six digits.
  char msglen[6];
  snprintf(msglen, 6, "%d", length);
  
  Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);
  Serial.print("Data length: ");
  Serial.println(length);
  Serial.println(jm);

  while (!client.connected()) {
    Serial.println("Lost connection to server...trying to reconnect.");
    client = getClient();  
    if (client.connected()) {
      break;
    }
    delay(1000);
  }

// Send request (In HTTP 1.1 connection is persistent by default - keep-alive).
  if (client.connected()) {
    
    unsigned long n_debug;
    Serial.println("Start transmission.");
    n_debug = client.fastrprint(F("POST ")); client.fastrprint(POST_DATA_ENDPOINT); client.fastrprint(F(" HTTP/1.1")); client.fastrprint(crlf);
    Serial.print("n debug: "); Serial.println(n_debug, DEC);

    client.fastrprint(F("Host: ")); client.fastrprint(SEDER_HOST); client.fastrprint(F(":")); client.fastrprint(sederPortStr); client.fastrprint(crlf);
    client.fastrprint(F("Content-Length: ")); client.fastrprint(msglen); client.fastrprint(crlf);
    client.fastrprint(F("Content-Type: application/json")); client.fastrprint(crlf);
    client.fastrprint(F("User-Agent: SEDER_ARD/1.0")); client.fastrprint(crlf);
    client.fastrprint(crlf);
    
    //char strx[10];
    //strncpy(strx,jm, sizeof(strx));
    n_debug = client.fastrprint(jm);
    Serial.print("n debug: "); Serial.println(n_debug, DEC);
    //client.fastrprint(strx);
  } else {
      Serial.println(F("Connection failed"));    
    return;
  }

  // Print response.  
  Serial.println(F("-------------------------------------"));

  /* Read data until either the connection is closed, or the idle timeout is reached. */ 
  unsigned long lastRead = millis();
  while (client.connected() && (millis() - lastRead < TIMEOUT)) {
    while (client.available()) {
      char c = client.read();
      Serial.print(c);
      lastRead = millis();
    }
  }
  Serial.println(F("-------------------------------------"));
  
  // Not calling client.close() and cc3000.disconnect() to keep the connection open between requests. 
  // If the server disconnects, a new client will be created before sending a request.
}

Adafruit_CC3000_Client getClient() {
  
  // Get the website IP & print it
  uint32_t ip = 0;
  Serial.print(SEDER_HOST); Serial.print(F(" -> "));
  while (ip == 0) {
    if (! cc3000.getHostByName((char *)SEDER_HOST, &ip)) {
      Serial.println(F("Couldn't resolve!"));
    }
    delay(500); // needed?? TODO
  }
  cc3000.printIPdotsRev(ip);
  
  // Send request
  Adafruit_CC3000_Client client = cc3000.connectTCP(ip, SEDER_PORT);
  
  return client;
}

/**************************************************************************/
/*!
    @brief  Displays the driver mode (tiny of normal), and the buffer
            size if tiny mode is not being used

    @note   The buffer size and driver mode are defined in cc3000_common.h
*/
/**************************************************************************/
void displayDriverMode(void)
{
  #ifdef CC3000_TINY_DRIVER
    Serial.println(F("CC3000 is configure in 'Tiny' mode"));
  #else
    Serial.print(F("RX Buffer : "));
    Serial.print(CC3000_RX_BUFFER_SIZE);
    Serial.println(F(" bytes"));
    Serial.print(F("TX Buffer : "));
    Serial.print(CC3000_TX_BUFFER_SIZE);
    Serial.println(F(" bytes"));
  #endif
}

/**************************************************************************/
/*!
    @brief  Tries to read the CC3000's internal firmware patch ID
*/
/**************************************************************************/
uint16_t checkFirmwareVersion(void)
{
  uint8_t major, minor;
  uint16_t version;
  
#ifndef CC3000_TINY_DRIVER  
  if(!cc3000.getFirmwareVersion(&major, &minor))
  {
    Serial.println(F("Unable to retrieve the firmware version!\r\n"));
    version = 0;
  }
  else
  {
    Serial.print(F("Firmware V. : "));
    Serial.print(major); Serial.print(F(".")); Serial.println(minor);
    version = major; version <<= 8; version |= minor;
  }
#endif
  return version;
}

/**************************************************************************/
/*!
    @brief  Tries to read the 6-byte MAC address of the CC3000 module
*/
/**************************************************************************/
void displayMACAddress(void)
{
  uint8_t macAddress[6];
  
  if(!cc3000.getMacAddress(macAddress))
  {
    Serial.println(F("Unable to retrieve MAC Address!\r\n"));
  }
  else
  {
    Serial.print(F("MAC Address : "));
    cc3000.printHex((byte*)&macAddress, 6);
  }
}


/**************************************************************************/
/*!
    @brief  Tries to read the IP address and other connection details
*/
/**************************************************************************/
bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  
  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}

// Minimalist time server query; adapted from Adafruit Gutenbird sketch,
// which in turn has roots in Arduino UdpNTPClient tutorial.
unsigned long getTime(void) {

  Adafruit_CC3000_Client gt;
  uint8_t       buf[48];
  unsigned long ip, startTime, t = 0L;

  Serial.print(F("Locating time server..."));

  // Hostname to IP lookup; use NTP pool (rotates through servers)
  if(cc3000.getHostByName((char *)"pool.ntp.org", &ip)) {
    static const char PROGMEM
      timeReqA[] = { 227,  0,  6, 236 },
      timeReqB[] = {  49, 78, 49,  52 };

    Serial.println(F("\r\nAttempting connection..."));
    startTime = millis();
    do {
      gt = cc3000.connectUDP(ip, 123);
    } while((!gt.connected()) &&
            ((millis() - startTime) < TIMEOUT));

    if(gt.connected()) {
      Serial.print(F("connected!\r\nIssuing request..."));

      // Assemble and issue request packet
      memset(buf, 0, sizeof(buf));
      memcpy_P( buf    , timeReqA, sizeof(timeReqA));
      memcpy_P(&buf[12], timeReqB, sizeof(timeReqB));
      gt.write(buf, sizeof(buf));

      Serial.print(F("\r\nAwaiting response..."));
      memset(buf, 0, sizeof(buf));
      startTime = millis();
      while((!gt.available()) &&
            ((millis() - startTime) < TIMEOUT));
      if(gt.available()) {
        gt.read(buf, sizeof(buf));
        t = (((unsigned long)buf[40] << 24) |
             ((unsigned long)buf[41] << 16) |
             ((unsigned long)buf[42] <<  8) |
              (unsigned long)buf[43]) - 2208988800UL;
        Serial.print(F("OK\r\n"));
      }
      gt.close();
    }
  }
  if(!t) Serial.println(F("error"));
  return t;
}
