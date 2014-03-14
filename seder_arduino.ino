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
#include <string.h>
#include "utility/debug.h"
#include "DebugUtils.h"

//#define DEBUG_MODE 1

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
#define SAMP_PERIOD 200 // sampling period in millisec
#define TIME_SERVER_PERIOD 86400000 // period to request time for server in millisec

#define POST_DATA_ENDPOINT "/v0/data"

// We allocate data buffer here to use static allocation. Be careful when changing parameters.

// Header description.
// TYPE      BYTES     EXAMPLE      DESCRIPTION
// char[10]   10       123          account id (using ID_SIZE = 10)
// u long      4       1394413060   base unix time in seconds (lastPolledTime)
// u long      4       57326        delta time to be added to base time in milliseconds
// short       2       800          sample period in milliseconds
// short       2       5            num samples per measurement
// byte        1       2            num measurements (num analog sensors)
// TOTAL      23 bytes
#define ID_SIZE 10   // define it here so we can allocate the buffer at compile time.
#define HEAD_SIZE 23 // define it here so we can allocate the buffer at compile time.
// The data is organized as follows:
// SAMPLE    SENSOR    VALUE
// 0         0         123
// 0         1         23
// 1         0         112
// 1         1         25
// ...
// values are of type short (a short stores a 16-bit (2-byte) value. This yields a range of -32,768 to 32,767)
// note that the size of int varies for different ATNEL processors and short is always 2 bytes.
//
// Sample times can be decoded as follows:
// SAMPLE   TIME
// 0        lastPolledTime + (millis() - sketchTime)
// 1        lastPolledTime + (millis() - sketchTime) * 1000 + 1 * SAMP_PERIOD
// 2        lastPolledTime + (millis() - sketchTime) * 1000 + 2 * SAMP_PERIOD
// ...
#define NUM_SENSORS 2 // number of analog sensors
#define ANALOG_SIZE 2 // analog value in bytes
#define NUM_SAMP_PER_MSG 20 // number of sample periods between REST messages
#define MSG_SIZE NUM_SAMP_PER_MSG * NUM_SENSORS * ANALOG_SIZE + HEAD_SIZE

Adafruit_CC3000_Client client;
Adafruit_CC3000_Client getClient();

//const unsigned long
  //connectTimeout  = 15L * 1000L, // Max time to wait for server connection
  //responseTimeout = 15L * 1000L; // Max time to wait for data from server
short sampIndex       = 0;  // current sample index
unsigned long
  lastPolledTime  = 0L, // Last value retrieved from time server
  sketchTime      = 0L, // CPU milliseconds since last server query
  lastSampleTime  = 0L; // Time since last sample.
char sederPortStr[5];
char accountID[ID_SIZE];
byte msg[MSG_SIZE];
char msglen[6]; // msg size encoded as a string (needed for HTTP header) 
const char * fakeid = "ABCDEFGHIJ";

// analog pins in the order they are read.
// Magnitudes: infrared, ultrasound
int analogPins[] = {A0,A2}; 

//void sendData(const char * id, byte nsens, struct AnalogPin **sa, unsigned long t1, 
//  unsigned long t2, unsigned int per, unsigned int nsamp);

//int ledPin = 13;      // select the pin for the LED

void setup(void)
{
 #if defined DEBUG_MODE
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
 #endif
 
  DEBUG_PRINT(F("Free RAM: ")); DEBUG_PRINT(getFreeRam());
  DEBUG_PRINT(F("Hello, SEDER!\n")); 
  DEBUG_PRINT(F("\nInitialising the CC3000 ..."));

  displayDriverMode();
  
  if (!cc3000.begin()) {
    DEBUG_PRINT(F("Unable to initialise the CC3000! Check your wiring?"));
    for(;;);
  }

  uint16_t firmware = checkFirmwareVersion();
  if ((firmware != 0x113) && (firmware != 0x118)) {
    PRINT_F("Wrong firmware version!");
    for(;;);
  }
  
  displayMACAddress();
  
  DEBUG_PRINT(("\nDeleting old connection profiles\n"));
  if (!cc3000.deleteProfiles()) {
    PRINT_F("Failed!");
    while(1);
  }

  /* Attempt to connect to an access point */
  char *ssid = (char *)WLAN_SSID;             /* Max 32 chars */
  DEBUG_PRINT(F("Attempting to connect to ")); DEBUG_PRINT(ssid); DEBUG_PRINT(F("\n"));
  
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
   snprintf(msglen, 6, "%d", MSG_SIZE);
   Serial.print(F("Message size in bytes: ")); Serial.println(msglen);

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
  
  // Read data from sensors.
  if((millis() - lastSampleTime) >= SAMP_PERIOD) {

    lastSampleTime = millis();    
    
    for(int i = 0; i < NUM_SENSORS; i++) {
      writeAnalogValue((short)analogRead(analogPins[i]), sampIndex, i, NUM_SENSORS, msg);
      //values[sampIndex * NUM_SENSORS + i] = analogRead(analogPins[i]);
    }
    
    // Update index after reading all values.
    sampIndex++;

    // check if time to send a message
    if(sampIndex == NUM_SAMP_PER_MSG) {
      sampIndex = 0;

      // Send data to server.
      Serial.println(F("*** SEND DATA ***"));
      DEBUG_PRINT(F("*** SEND DATA ***"));
      DEBUG_PRINT(1234);

      // Time info needed to reconstract the absolute time for each sample.
      Serial.print(F("Base UNIX time in seconds: ")); // time in secs since epoch
      Serial.println(lastPolledTime);
      Serial.print(F("Delta UNIX time in milliseconds: "));
      Serial.println(millis() - sketchTime); // add this delta to get time for first sample
      Serial.print(F("Sampling Period in milliseconds: "));
      Serial.println(SAMP_PERIOD); // add samp period to get time for subsequent samples
      
      /// DEBUG
      Serial.print(F("id bytes: ")); 
      for (int i = 0; i <10; i++) {
         Serial.print(fakeid[i], DEC);Serial.print(F(", "));
      }
      Serial.println("");

      sendData(fakeid, NUM_SENSORS, msg, lastPolledTime, millis() - sketchTime, SAMP_PERIOD, NUM_SAMP_PER_MSG);
    }
  }
}

// Writes a value to the msg buffer.
void writeAnalogValue(short val, short sampIndex, byte sensorIndex, byte numSensors, byte *msg) {
  
      byte *ptr = msg + HEAD_SIZE + (sampIndex * numSensors + sensorIndex) * sizeof(short);      
      memcpy(ptr, &val, sizeof(val));
      Serial.print(F("sampIndex: ")); Serial.print(sampIndex, DEC);
      Serial.print(F(", sensorIndex: ")); Serial.print(sensorIndex, DEC);
      Serial.print(F(", val: ")); Serial.print(val, DEC);
      Serial.print(F(", msg addr start: ")); Serial.print(uint16_t(msg), HEX);
      Serial.print(F(", msg addr end: ")); Serial.print(uint16_t(msg + MSG_SIZE), HEX);
      Serial.print(F(", address: ")); Serial.println(uint16_t(ptr), HEX);
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
void sendData(const char * id, byte nsens, byte *msg, unsigned long t1, 
  unsigned long t2, unsigned int per, unsigned int nsamp)
{ 
      Serial.print("*** id[0]: "); Serial.println(id[0], DEC); // DEBUG

  Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);

  // Write msg header. (See documentation at the top of the file.)
  byte *ptr = msg;      
  Serial.print(F("Header addr start: ")); Serial.println(uint16_t(ptr), HEX);
  memcpy(ptr, id, sizeof(accountID));
  Serial.print(F("val: ")); Serial.print(id); Serial.print(F(", size: ")); Serial.println(sizeof(accountID), DEC);
    
    Serial.print("*** ptr[0]: "); Serial.println(ptr[0], DEC); // DEBUG
    Serial.print("*** msg[0]: "); Serial.println(msg[0], DEC); // DEBUG

  ptr += sizeof(accountID);
  memcpy(ptr, &t1, sizeof(t1));
  Serial.print(F("t1: ")); Serial.print(t1, DEC); Serial.print(F(", size: ")); Serial.println(sizeof(t1), DEC);
  ptr += sizeof(t1);
  memcpy(ptr, &t2, sizeof(t2));
  Serial.print(F("t2: ")); Serial.print(t2, DEC); Serial.print(F(", size: ")); Serial.println(sizeof(t2), DEC);
  ptr += sizeof(t2);
  memcpy(ptr, &per, sizeof(per));
  Serial.print(F("per: ")); Serial.print(per, DEC); Serial.print(F(", size: ")); Serial.println(sizeof(per), DEC);
  ptr += sizeof(per);
  memcpy(ptr, &nsamp, sizeof(nsamp));
  Serial.print(F("nsamp: ")); Serial.print(nsamp, DEC); Serial.print(F(", size: ")); Serial.println(sizeof(nsamp), DEC);
  ptr += sizeof(nsamp);
  memcpy(ptr, &nsens, sizeof(nsens));
  Serial.print(F("nsens: ")); Serial.print(nsens, DEC); Serial.print(F(", size: ")); Serial.println(sizeof(nsens), DEC);
  ptr += sizeof(nsens);
  Serial.print(F("Header addr end: ")); Serial.println(uint16_t(ptr), HEX);

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
    
    Serial.println("Start transmission.");
    client.fastrprint(F("POST ")); client.fastrprint(POST_DATA_ENDPOINT); client.fastrprint(F(" HTTP/1.1\r\n"));
    client.fastrprint(F("Host: ")); client.fastrprint(SEDER_HOST); client.fastrprint(F(":")); client.fastrprint(sederPortStr);
       client.fastrprint(F("\r\n"));
    client.fastrprint(F("Content-Length: ")); client.fastrprint(msglen); client.fastrprint("\r\n");
    client.fastrprint(F("Content-Type: application/octet-stream\r\n"));
    client.fastrprint(F("User-Agent: SEDER_ARD/1.0\r\n"));
    client.fastrprint(F("\r\n"));
    int n = client.write(msg, MSG_SIZE, 0);
    Serial.print(F("n: ")); Serial.println(n, DEC);

  } else {
      Serial.println(F("Connection failed"));    
    return;
  }

  // Print response.  
  Serial.println(F("-------------------------------------"));

  /* Read data until either the connection is closed, or the idle timeout is reached. */ 
  unsigned long lastRead = millis();
  while (client.connected() && (millis() - lastRead < TIMEOUT)) {
    // Serial.println(F("Get response..."));    this loop is a waste, can we improve?
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
