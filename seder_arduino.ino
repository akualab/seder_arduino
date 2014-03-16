/***************************************************
  SEDER Arduino Client
  Copyright AKUALAB INC. 2014
  
  NOTES:
  The cc3k hangs up after sending a few data batches. 
  discussions:
  
  change SPI clock:
  http://forums.adafruit.com/viewtopic.php?f=22&t=49074#p247907
  
  http://www.adafruit.com/forums/viewtopic.php?f=22&t=49722
  https://community.spark.io/t/bug-bounty-kill-the-cyan-flash-of-death/1322/398

  WiFi code adapted from the Adafruit. See notice below:

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

// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIV2); // changed by leo as per forum suggestion.
                                         //SPI_CLOCK_DIVIDER); // you can change this clock speed but DI

#define WLAN_SSID       "lunas"        // cannot be longer than 32 characters!
#define WLAN_PASS       "ohigginssucre"
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2

#define TIMEOUT  3000
#define RESPONSE_TIMEOUT  1000

#define SEDER_HOST "monster.akualab.com"
#define SEDER_PORT 3000
#define SAMP_PERIOD 200L // sampling period in millisec
#define TIME_SERVER_PERIOD 86400000L // period to request time for server in millisec

#define POST_DATA_ENDPOINT "/v0/data"

// We allocate data buffer here to use static allocation. Be careful when changing parameters.

// Header description.
// TYPE      BYTES     EXAMPLE      DESCRIPTION
// char[10]   10       ABCDEFGHIJ   account id (using ACCT_ID_SIZE = 10)
// char[10]   10       0123456789   device ID (using DEVICE_ID_SIZE = 10)
// u long      4       1394413060   base unix time in seconds (lastPolledTime)
// u long      4       57326        delta time to be added to base time in milliseconds
// short       2       800          sample period in milliseconds
// short       2       5            num samples per measurement
// byte        1       2            num measurements (num analog sensors)
// TOTAL      33 bytes
#define ACCT_ID_SIZE 10   // define it here so we can allocate the buffer at compile time.
#define DEVICE_ID_SIZE 10   // define it here so we can allocate the buffer at compile time.
#define HEAD_SIZE 33 // define it here so we can allocate the buffer at compile time.
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
// Sample times can be decoded to msec as follows:
// SAMPLE   TIME
// 0        lastPolledTime * 1000 + (millis() - sketchTime)
// 1        lastPolledTime * 1000 + (millis() - sketchTime) + 1 * SAMP_PERIOD
// 2        lastPolledTime * 1000 + (millis() - sketchTime) + 2 * SAMP_PERIOD
// ...
#define NUM_SENSORS 2 // number of analog sensors
#define ANALOG_SIZE 2 // analog value in bytes
#define NUM_SAMP_PER_MSG 80L // number of sample periods between REST messages
#define MSG_SIZE NUM_SAMP_PER_MSG * NUM_SENSORS * ANALOG_SIZE + HEAD_SIZE
#define CHUNK_SIZE 20L

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
char accountID[ACCT_ID_SIZE];
char deviceID[DEVICE_ID_SIZE];
byte msg[MSG_SIZE];
char msglen[6]; // msg size encoded as a string (needed for HTTP header) 
int msgSize = MSG_SIZE;
int chunkSize = CHUNK_SIZE;
int numChunks = msgSize/chunkSize;

// analog pins in the order they are read.
// Magnitudes: infrared, ultrasound
int analogPins[] = {A0,A2}; 
int ledPin = 13;      // select the pin for the LED

void(* resetFunc) (void) = 0; //declare reset function @ address 0

/*************************************

   SETUP


*************************************/
void setup(void)
{
 int k;
 #ifdef SERIAL_DEBUG_ENABLED > 0
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
 #endif
 
  DebugPrintln(F("\n\nHello, SEDER!\n")); 
  DebugPrint(F("Free RAM: ")); DebugPrintln(getFreeRam());
  DebugPrintln(F("\nInitialising the CC3000 ..."));

  displayDriverMode();
  
  for (k = 0; !cc3000.begin(); k++) {
    DebugPrintln(F("Unable to initialise the CC3000!"));
    delay(delayForIteration(k));
  }

  uint16_t firmware = checkFirmwareVersion();
  if ((firmware != 0x113) && (firmware != 0x118)) {
    DebugPrintln(F("Wrong firmware version!"));
    for(;;);
  }
  
  displayMACAddress();
  
  DebugPrintln(F("Deleting old connection profiles"));
  for (k=0;!cc3000.deleteProfiles();k++) {
    DebugPrintln("Can't delete old profile!...retrying");
    delay(delayForIteration(k));
  }

  /* Attempt to connect to an access point */
  char *ssid = (char *)WLAN_SSID;             /* Max 32 chars */
  DebugPrint(F("Attempting to connect to ")); DebugPrintln(ssid);
  
  /* NOTE: Secure connections are not available in 'Tiny' mode! */
  for (k=0;!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY);k++) {
    DebugPrintln(F("Failed to connect!...retrying"));
    delay(delayForIteration(k));
  }
  DebugPrintln(F("Connected!"));
  
  /* Wait for DHCP to complete */
  DebugPrintln(F("Request DHCP"));
  for (k=0;!cc3000.checkDHCP();k++) {
    DebugPrintln(F("Retrying...")); 
    delay(delayForIteration(k));
  }

  /* Display the IP address DNS, Gateway, etc. */  
  for (k=0;!displayConnectionDetails();k++) {
    DebugPrintln(F("Failed to display connection details!...retrying"));
    delay(delayForIteration(k));
  }
  
  /* Prepare SEDER */
   sprintf(sederPortStr, "%d", SEDER_PORT); // need string for the HTTP header.
   snprintf(msglen, 6, "%d", MSG_SIZE);
   DebugPrint(F("Message size in bytes: ")); DebugPrintln(msglen);
   
   // Use some fake IDs for now.
   char *x = "ABCDEFGHIJ";
   memccpy (accountID, x, 1, ACCT_ID_SIZE);
   char *y = "0000000001";
   memccpy (deviceID, y, 1, DEVICE_ID_SIZE);

  // Initialize TCP client for sending data.
  client = getClient();
}

void loop(void) {

  // To reduce load on NTP servers, time is polled once per roughly 24 hour period.
  // Otherwise use millis() to estimate time since last query.  Plenty accurate.
  // We need condition sampIndex == 0 so lastPolledTime is the same for all values in a message.
  for(int k=0;sketchTime == 0 || ((millis() - sketchTime > TIME_SERVER_PERIOD) && sampIndex == 0);k++) {            // Time's up?
    unsigned long t;
    //#ifdef SERIAL_DEBUG_ENABLED > 0
      //t  = 111111L; // MOCK FOR TESTING REMEMBER TO CHANGE -- DEBUG
    //#else
      t  = getTime(); // Query time server
    //#endif
    if(t) {                       // Success?
      lastPolledTime = t;         // Save time
      sketchTime     = millis();  // Save sketch time of last valid time query
      break;
    } else {
        delay(delayForIteration(k));
    } 
  }
  
  // Read data from sensors.
  if((millis() - lastSampleTime) >= SAMP_PERIOD) {

    lastSampleTime = millis();    
    
    for(int i = 0; i < NUM_SENSORS; i++) {
      writeAnalogValue((short)analogRead(analogPins[i]), sampIndex, i, NUM_SENSORS, msg);
    }
    
    // Update index after reading all values.
    sampIndex++;

    // check if time to send a message
    if(sampIndex == NUM_SAMP_PER_MSG) {
      sampIndex = 0;

      // Send data to server.
      DebugPrintln(F("*** SEND DATA ***"));

      // Time info needed to reconstract the absolute time for each sample.
      DebugPrint(F("Base UNIX time in seconds: ")); // time in secs since epoch
      DebugPrintln(lastPolledTime);
      DebugPrint(F("Delta UNIX time in milliseconds: "));
      DebugPrintln(millis() - sketchTime); // add this delta to get time for first sample
      DebugPrint(F("Sampling Period in milliseconds: "));
      DebugPrintln(SAMP_PERIOD); // add samp period to get time for subsequent samples

      //sendData(fakeid, NUM_SENSORS, msg, lastPolledTime, millis() - sketchTime, SAMP_PERIOD, NUM_SAMP_PER_MSG);
      sendData();
    }
  }
}

// Writes a value to the msg buffer.
void writeAnalogValue(short val, short sampIndex, byte sensorIndex, byte numSensors, byte *msg) {
  
      byte *ptr = msg + HEAD_SIZE + (sampIndex * numSensors + sensorIndex) * sizeof(short);      
      memcpy(ptr, &val, sizeof(val));
//      DebugPrint(F("sampIndex: ")); DebugPrint(sampIndex, DEC);
//      DebugPrint(F(", sensorIndex: ")); DebugPrint(sensorIndex, DEC);
//      DebugPrint(F(", val: ")); DebugPrint(val, DEC);
//      DebugPrint(F(", msg addr start: ")); DebugPrint(uint16_t(msg), HEX);
//      DebugPrint(F(", msg addr end: ")); DebugPrint(uint16_t(msg + MSG_SIZE), HEX);
//      DebugPrint(F(", address: ")); DebugPrintln(uint16_t(ptr), HEX);
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
boolean sendData() { 
  byte nsens = NUM_SENSORS;
  unsigned long t1 = lastPolledTime;
  unsigned long t2 = millis() - sketchTime;
  unsigned int per = SAMP_PERIOD;
  unsigned int nsamp = NUM_SAMP_PER_MSG;
    
  DebugPrint("Free RAM: "); DebugPrintln(getFreeRam(), DEC);

  // Write msg header. (See documentation at the top of the file.)
  byte *ptr = msg;      
  DebugPrint(F("Header addr start: ")); DebugPrintln(uint16_t(ptr), HEX);
  memcpy(ptr, accountID, sizeof(accountID));
  ptr += sizeof(accountID);
  memcpy(ptr, deviceID, sizeof(deviceID));
  ptr += sizeof(deviceID);
  memcpy(ptr, &t1, sizeof(t1));
  ptr += sizeof(t1);
  memcpy(ptr, &t2, sizeof(t2));
  ptr += sizeof(t2);
  memcpy(ptr, &per, sizeof(per));
  ptr += sizeof(per);
  memcpy(ptr, &nsamp, sizeof(nsamp));
  ptr += sizeof(nsamp);
  memcpy(ptr, &nsens, sizeof(nsens));
  ptr += sizeof(nsens);
  DebugPrint(F("Header addr end: ")); DebugPrintln(uint16_t(ptr), HEX);

 for (int k=0;!client.connected();k++) {
    DebugPrintln(F("client is not connected...connecting"));
    client = getClient();  
    if (client.connected()) {
      break;
    }
    delay(delayForIteration(k));
  }

  DebugPrint(F("Free RAM: ")); DebugPrintln(getFreeRam());

// Send request (In HTTP 1.1 connection is persistent by default - keep-alive).
  if (client.connected()) {
    
    int n;
    DebugPrintln("Start transmission.");
    client.fastrprint(F("POST ")); client.fastrprint(POST_DATA_ENDPOINT); client.fastrprint(F(" HTTP/1.1\r\n"));
    client.fastrprint(F("Host: ")); client.fastrprint(SEDER_HOST); client.fastrprint(F(":")); client.fastrprint(sederPortStr);
       client.fastrprint(F("\r\n"));
    client.fastrprint(F("Content-Length: ")); client.fastrprint(msglen); client.fastrprint("\r\n");
    client.fastrprint(F("Content-Type: application/octet-stream\r\n"));
    client.fastrprint(F("User-Agent: SEDER_ARD/1.0\r\n"));
    client.fastrprint(F("\r\n"));
    delay(5); // workaround to give cc3k time to catch up.
    
    int i =0;
    for (;i<numChunks;i++) {
      client.write(msg + i * chunkSize, chunkSize, 0);
      delay(5);
    }
    client.write(msg + i * chunkSize, msgSize % chunkSize, 0);

    //client.write(msg, MSG_SIZE, 0);

    // Handle responses.
    // For now, when response fails, we continue. In the future we must put data in a FIFO queue and
    // use a separate thread to send out data from the queue. 
    unsigned long ts;
    #ifdef SERIAL_DEBUG_ENABLED > 0
    ts = millis();
    #endif
    DebugPrint(F("OK\r\nAwaiting response..."));
    int c = 0;
    // Wait for char '!' in response.
    while(((c = timedRead()) > 0) && (c != '!'));
    if(c == '!')  { 
       DebugPrintln(F("success!"));
    } else if(c < 0) {
       DebugPrintln(F("timeout"));
    } else {
      DebugPrintln(F("error: invalid response"));
    }
    int32_t r = client.close();
    DebugPrint(F("close() returned: ")); DebugPrintln(r);
    DebugPrint(F("response time in millisec: ")); DebugPrintln(millis() - ts);
    return (c == '!');
    
  } else {
      DebugPrintln(F("Connection failed"));    
    return false;
  }
  
  // Print response.  
//  DebugPrintln(F("-------------------------------------"));
//
//  /* Read data until either the connection is closed, or the idle timeout is reached. */ 
//  unsigned long lastRead = millis();
//  while (client.connected() && (millis() - lastRead < RESPONSE_TIMEOUT)) {
//    // DebugPrintln(F("Get response..."));    this loop is a waste, can we improve?
//    while (client.available()) {
//      char c = client.read();
//      DebugPrint(c);
//      lastRead = millis();
//    }
//  }
//  DebugPrintln(F("-------------------------------------"));
  
  // Not calling client.close() and cc3000.disconnect() to keep the connection open between requests. 
  // If the server disconnects, a new client will be created before sending a request.
}

Adafruit_CC3000_Client getClient() {
  
  // Get the website IP & print it
  uint32_t ip = 0;
  DebugPrint(SEDER_HOST); DebugPrint(F(" -> "));
  for (int k=0;ip == 0;k++) {
    if (! cc3000.getHostByName((char *)SEDER_HOST, &ip)) {
      DebugPrintln(F("Couldn't resolve! Retrying... "));
      unsigned long d = delayForIteration(k);
      delay(d);
      if (d >= 192000) {
         // give up.
         cc3000.reboot();
         resetFunc();
      }
    } 
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
    DebugPrintln(F("CC3000 is configure in 'Tiny' mode"));
  #else
    DebugPrint(F("RX Buffer : "));
    DebugPrint(CC3000_RX_BUFFER_SIZE);
    DebugPrintln(F(" bytes"));
    DebugPrint(F("TX Buffer : "));
    DebugPrint(CC3000_TX_BUFFER_SIZE);
    DebugPrintln(F(" bytes"));
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
    DebugPrintln(F("Unable to retrieve the firmware version!\r\n"));
    version = 0;
  }
  else
  {
    DebugPrint(F("Firmware V. : "));
    DebugPrint(major); DebugPrint(F(".")); DebugPrintln(minor);
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
    DebugPrintln(F("Unable to retrieve MAC Address!\r\n"));
  }
  else
  {
    DebugPrint(F("MAC Address : "));
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
    DebugPrintln(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    DebugPrint(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    DebugPrint(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    DebugPrint(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    DebugPrint(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    DebugPrint(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    DebugPrintln();
    return true;
  }
}

// Minimalist time server query; adapted from Adafruit Gutenbird sketch,
// which in turn has roots in Arduino UdpNTPClient tutorial.
unsigned long getTime(void) {

  Adafruit_CC3000_Client gt;
  uint8_t       buf[48];
  unsigned long ip, startTime, t = 0L;

  DebugPrint(F("Locating time server..."));

  // Hostname to IP lookup; use NTP pool (rotates through servers)
  if(cc3000.getHostByName((char *)"pool.ntp.org", &ip)) {
    static const char PROGMEM
      timeReqA[] = { 227,  0,  6, 236 },
      timeReqB[] = {  49, 78, 49,  52 };

    DebugPrintln(F("\r\nAttempting connection..."));
    startTime = millis();
    do {
      gt = cc3000.connectUDP(ip, 123);
    } while((!gt.connected()) &&
            ((millis() - startTime) < TIMEOUT));

    if(gt.connected()) {
      DebugPrint(F("connected!\r\nIssuing request..."));

      // Assemble and issue request packet
      memset(buf, 0, sizeof(buf));
      memcpy_P( buf    , timeReqA, sizeof(timeReqA));
      memcpy_P(&buf[12], timeReqB, sizeof(timeReqB));
      gt.write(buf, sizeof(buf));

      DebugPrint(F("\r\nAwaiting response..."));
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
        DebugPrint(F("OK\r\n"));
      }
      gt.close();
    }
  }
  if(!t) DebugPrintln(F("error"));
  return t;
}

// Read from client stream with a 5 second timeout.  Although an
// essentially identical method already exists in the Stream() class,
// it's declared private there...so this is a local copy.
int timedRead(void) {
  unsigned long start = millis();
  while((!client.available()) && ((millis() - start) < RESPONSE_TIMEOUT));
  return client.read();  // -1 on timeout
}

// Controls how long to wait before retrying an action based on number
// tries. Initally it doubles teh delay for each iteration. After 6 tries
// it stops increasing the delay.
unsigned long delayForIteration(int iter) {
  unsigned long d = 1;
  if (iter < 7) {
     for (int i = 0; i <iter; i++) {
       d = d * 2;
     }
  } else {
     d = 64; 
  }
  DebugPrint(F("wait before retry in millisecs: "));DebugPrintln(d * TIMEOUT);
  return d * TIMEOUT;
}
