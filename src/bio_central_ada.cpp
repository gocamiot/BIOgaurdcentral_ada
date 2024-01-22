
/*
This is copyright Tesrac.com 240117
BIO is written to have a central that has a biometric reader
and a peripheral that has a relay and a button.
240117
- basic pher code

*/
#include <bluefruit.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <Adafruit_Fingerprint.h>

//list functions
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void pher_connect_callback(uint16_t conn_handle); //this is advertised from central to connect to
void pher_disconnect_callback(uint16_t conn_handle, uint8_t reason);
void setLED(bool on, String clr);
void pulseLED(int times, String color);
void switchMachine(int on);
void startAdv(void);
void blink_timer_callback(TimerHandle_t xTimerID);
void bleuart_rx_callback(BLEClientUart& uart_svc);
void sendAll(const char* str);
void scan_callback(ble_gap_evt_adv_report_t* report);
int findConnHandle(uint16_t conn_handle);
int getFingerprintID();
uint8_t getFingerprintIDez();



// max concurrent connections supported by this example
#define MAX_PRPH_CONNECTION   3
uint8_t connection_count = 0;
#define NO_ADVERTISING_TIMEOUT   0 // seconds
#define VBATPIN A1

//tag_name must be unique for every tag
//centrals will be even numbers, its tag will be this number -1
char tag_name[8] = "b0002";
const char ADVERTISING_NAME_PREFIX[] = "b0002";
char tag_ver[30] = "BIOgaurd central 240110";

// Service UUID used to differentiate this device. Note that the byte order is reversed.
// The UUID below corresponds to 0a7c96c1-a53f-40ad-93bc-6aaa8fb4f607
// const uint8_t TAG_UUID_SERVICE[] =
//   {
//     0x07, 0xf6, 0xb4, 0x8f, 0xaa, 0x6a, 0xbc, 0x93,
//     0xad, 0x40, 0x16, 0x3f, 0xa5, 0xc1, 0x96, 0x7c
//   };

//these are services that we find on client side
const uint8_t TAG_UUID_SERVICE[] =
  {
    0x48, 0x2b, 0xdf, 0x9f, 0x2f, 0x2c, 0xd7, 0x9d,
    0x03, 0x41, 0x16, 0x7d, 0xb3, 0x7f, 0xee, 0x5a
  }; 

const uint8_t TAG_UUID_CHR_SWITCH[] =
  {
    0x48, 0x2b, 0xdf, 0x9f, 0x2f, 0x2c, 0xd7, 0x9d,
    0x03, 0x41, 0x16, 0x7d, 0xb5, 0x7f, 0xee, 0x5a
  };

const uint8_t TAG_UUID_CHR_BUTTON[] =
  {
    0x48, 0x2b, 0xdf, 0x9f, 0x2f, 0x2c, 0xd7, 0x9d,
    0x03, 0x41, 0x16, 0x7d, 0xb4, 0x7f, 0xee, 0x5a
  };

//uint8_t tag_hs;  

BLEClientService tag_service = BLEClientService(BLEUuid(TAG_UUID_SERVICE));
BLEClientCharacteristic tag_chr_switch = BLEClientCharacteristic(BLEUuid(TAG_UUID_CHR_SWITCH));

//These are services we want to advertise for connection
const uint8_t CENTRAL_UUID_SERVICE[] =
  {
    0x48, 0x2b, 0xdf, 0x9f, 0x2f, 0x2c, 0xd7, 0x9d,
    0x03, 0x41, 0x16, 0x7d, 0xb6, 0x7f, 0xee, 0x5a
  }; 

const uint8_t CENTRAL_UUID_CHR_SWITCH[] =
  {
    0x48, 0x2b, 0xdf, 0x9f, 0x2f, 0x2c, 0xd7, 0x9d,
    0x03, 0x41, 0x16, 0x7d, 0xb7, 0x7f, 0xee, 0x5a
  };

BLEService central_service = BLEService(BLEUuid(CENTRAL_UUID_SERVICE));
BLECharacteristic central_chr_switch = BLECharacteristic(BLEUuid(CENTRAL_UUID_CHR_SWITCH));
BLEBas blebas;    // BAS (Battery Service) helper class instance

// BLEBas blebas;    // BAS (Battery Service) helper class instance

// const uint8_t LBS_UUID_SERVICE[] =
//       {
//         0x07, 0xf6, 0xb4, 0x8f, 0xaa, 0x6a, 0xbc, 0x93,
//         0xad, 0x40, 0x163f, 0xa5, 0xc3, 0x96, 0x7c, 0x0a
//       };

//const uint8_t LBS_UUID_CHR_BUTTON[] =
//       {
//             0x07, 0xf6, 0xb4, 0x8f, 0xaa, 0x6a, 0xbc, 0x93,
//             0xad, 0x40, 0x163f, 0xa5, 0xc4, 0x96, 0x7c, 0x0a
//       }; 
      
// const uint8_t LBS_UUID_CHR_ON_SWITCH[] =
//       {
//             0x07, 0xf6, 0xb4, 0x8f, 0xaa, 0x6a, 0xbc, 0x93,
//             0xad, 0x40, 0x163f, 0xa5, 0xc5, 0x96, 0x7c, 0x0a
//       };

// //we can use this for pushbutton notice for like a knock sensor.
// const char CHR_NFY_LOW_LVL[] = "LOW_NFY";
// const uint8_t CHR_UUID_LOW_LVL[] =
//       {
//             0x07, 0xf6, 0xb4, 0x8f, 0xaa, 0x6a, 0xbc, 0x93,
//             0xad, 0x40, 0x163f, 0xa5, 0xc6, 0x96, 0x7c, 0x0a
//       };


//ppheriral info
// uint8_t TAG_SERVICE_UUID[] =               {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xB0,0x00,0x40,0x51,0x04,0x70,0xAA,0x00,0xF0};
// uint8_t TAG_CHARACTERISTIC_UUID[] =        {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xB0,0x00,0x40,0x51,0x04,0x71,0xAA,0x00,0xF0};
// uint8_t TAG_SWITCH_CHARACTERISTIC_UUID[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xB0,0x00,0x40,0x51,0x04,0x72,0xAA,0x00,0xF0};
// uint8_t TAG_PERIOD_CHARACTERISTIC_UUID[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xB0,0x00,0x40,0x51,0x04,0x73,0xAA,0x00,0xF0};
// uint8_t TAG_ADV_COMPLETE_LOCAL_NAME[] =    {0x43,0x43,0x32,0x36,0x35,0x30,0x20,0x53,0x65,0x6e,0x73,0x6f,0x72,0x54,0x61,0x67};

// BLEClientService        tagService(TAG_UUID_SERVICE);
// BLEClientCharacteristic tagCharacteristic(TAG_CHARACTERISTIC_UUID); 
// BLEClientCharacteristic tagCharacteristicSwitch(TAG_SWITCH_CHARACTERISTIC_UUID);
//BLEClientCharacteristic tagCharacteristicPeriod(TAG_PERIOD_CHARACTERISTIC_UUID);
// // Struct containing peripheral info
typedef struct {
  char deviceName[16+1];
  uint16_t conn_handle;
  uint8_t addr[6];
  int8_t rssi;
  uint8_t data[32];
  int8_t batt;

  BLEClientService _service;
  BLEClientCharacteristic _notification;
  BLEClientCharacteristic _data;
} prph_info_t;

/* Peripheral info array (one per peripheral device)
 * 
 * There are 'BLE_MAX_CONNECTION' central connections, but the
 * the connection handle can be numerically larger (for example if
 * the peripheral role is also used, such as connecting to a mobile
 * device). As such, we need to convert connection handles <-> the array
 * index where appropriate to prevent out of array accesses.
 * 
 * Note: One can simply declares the array with BLE_MAX_CONNECTION and use connection
 * handle as index directly with the expense of SRAM.
 */
prph_info_t prphs[BLE_MAX_CONNECTION];
uint8_t whitelist[BLE_MAX_CONNECTION][6];
int count_wl = 0;
uint16_t Readbuf[4];


// struct DeviceInfo {
//   uint8_t addr[6];
//   int8_t rssi;
//   uint8_t data[32];
//   String deviceName;
// };
//DeviceInfo devices[];
uint8_t numDevices = 0;

// Software Timer for blinking the RED LED
SoftwareTimer blinkTimer;
uint8_t connection_num = 0; // for blink pattern

// BLEService        lbs(LBS_UUID_SERVICE);
// BLECharacteristic lsbButton(LBS_UUID_CHR_BUTTON);
// BLECharacteristic lbsSwitch(LBS_UUID_CHR_ON_SWITCH);
// BLECharacteristic tag_chr_nfy_low(CHR_UUID_LOW_LVL);

//pulsepin for relay pulse, works with LED flash
#define PULSE_PIN_ACTIVE     LOW
uint8_t pulse_pin = A1;
uint8_t pulse_pin_state;
uint8_t switchPin6 = 6;
uint8_t on_state;
uint8_t low_lvl_pin = 5;
uint8_t low_lvl_pin_state;

//biometrics
#define bioSerial Serial1
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&bioSerial);
bool enrollment_mode = false;

// Use on-board button if available, else use A0 pin
#ifdef PIN_BUTTON1
  uint8_t button = PIN_BUTTON1;
#else
  uint8_t button = A0;
#endif

// CPB button active state is HIGH, all other is low
#ifdef ARDUINO_NRF52840_CIRCUITPLAY
  #define BUTTON_ACTIVE     HIGH
#else
  #define BUTTON_ACTIVE     LOW
#endif

uint8_t buttonState;

#ifdef PIN_NEOPIXEL

#ifndef NEOPIXEL_NUM
#define NEOPIXEL_NUM  1
#endif

Adafruit_NeoPixel neopixel = Adafruit_NeoPixel(NEOPIXEL_NUM, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

#endif

const unsigned int NUM_SAMPLES_IN_RUNNING_AVERAGE = 10;

bool isOk = true;
bool hasRtc = false;
bool hasStorage = false;

void switch6_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  (void) conn_hdl;
  (void) chr;
  (void) len; // len should be 1

  Serial.print("Received for switch 6: ");
  for (int i = 0; i < len; i++) {
    Serial.print(data[i], HEX); // Print each byte as a hexadecimal value
    Serial.print(" ");
  }
  Serial.print(data[0]);
  Serial.println();

  // Check if data[0] is 1
  if (data[0] == 1 ) {
//    digitalWrite(switchPin6, HIGH); // Call setSwitch(1) when data[0] is 1
      switchMachine(1);
  } else if (data[0] == 0) {
    switchMachine(0);
  }
}

//bool rtc_enabled = false;
void setup()
{
Serial.begin(115200);
   delay(100);
   while (!Serial); 
  Serial.println(tag_ver);
  Serial.println("------------------------------\n");

  // pinMode(button, BUTTON_ACTIVE ? INPUT_PULLDOWN : INPUT_PULLUP);
  // buttonState = (BUTTON_ACTIVE == digitalRead(button));
  // pinMode(pulse_pin, OUTPUT);
  // digitalWrite(pulse_pin, LOW); // pulse low
  // pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, 1-LED_STATE_ON); // led off
  // pinMode(switchPin6, OUTPUT);
  // digitalWrite(switchPin6, LOW); // machine off
  // pinMode(low_lvl_pin, BUTTON_ACTIVE ? INPUT_PULLDOWN : INPUT_PULLUP);
  // low_lvl_pin_state = (BUTTON_ACTIVE == digitalRead(low_lvl_pin));

  #ifdef PIN_NEOPIXEL
    neopixel.begin();
  #endif

  

  // Initialize blinkTimer for 100 ms and start it
  // blinkTimer.begin(100, blink_timer_callback);
  // blinkTimer.start();

  // Initialize Bluefruit with max concurrent connections as Peripheral = MAX_PRPH_CONNECTION, Central = 0
  Serial.println("Initialise the Bluefruit nRF52 module");
  Bluefruit.begin(0,4);

   // Init peripheral pool
  // for (uint8_t idx=0; idx<BLE_MAX_CONNECTION; idx++)
  // {
  //   // Invalid all connection handle
  //   prphs[idx].conn_handle = BLE_CONN_HANDLE_INVALID;
    
  //   // All of BLE Central Uart Serivce
  //   prphs[idx].bleuart.begin();
  //   prphs[idx].bleuart.setRxCallback(bleuart_rx_callback);
  // }


  //Bluefruit.Periph.setConnectCallback(connect_callback);
  //Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  // Setup manufacturer tag service
  //Serial.println("Configuring the Manufacturer Tag Service");

  // Note: You must call .begin() on the BLEService before calling .begin() on
  // any characteristic(s) within that service definition.. Calling .begin() on
  // a BLECharacteristic will cause it to be added to the last BLEService that
  // was 'begin()'ed!
  tag_service.begin();
  //now characteristic
  //tag_chr_switch.properties(CHR_PROPS_READ, CHR_PROPS_WRITE_WO_RESP);
  //tag_chr_switch.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  //tag_chr_switch.setFixedLen(2);
  tag_chr_switch.begin();

  // Configure Button characteristic
  // Properties = Read + Notify
  // Permission = Open to read, cannot write
  // Fixed Len  = 1 (button state)
  // tag_chr_hs.setProperties(CHR_PROPS_READ);
  // tag_chr_hs.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  // tag_chr_hs.setFixedLen(2);
  // tag_chr_hs.begin();
  // tag_chr_hs.write8(tag_hs);
  //tag_chr_hs.setWriteCallback(tag_write_callback);


  // Setup the LED-Button service using
  //Serial.println("Configuring the LED-Button Service");
  //tag_service.begin();

  // Configure Button characteristic
  // Properties = Read + Notify
  // Permission = Open to read, cannot write
  // Fixed Len  = 1 (button state)
  // lsbButton.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  // lsbButton.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  // lsbButton.setFixedLen(1);
  // lsbButton.begin();
  // lsbButton.write8(buttonState);

  // lbsSwitch.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  // lbsSwitch.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  // lbsSwitch.setFixedLen(1);
  // lbsSwitch.begin();
  // lbsSwitch.setWriteCallback(switch6_callback);

  
  // tag_chr_nfy_low.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  // tag_chr_nfy_low.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  // tag_chr_nfy_low.setFixedLen(1);
  // tag_chr_nfy_low.begin();
  // tag_chr_nfy_low.write8(low_lvl_pin_state);


// Start the BLE Battery Service and set it to 100%
  // Serial.println("Configuring the Battery Service");
  // blebas.begin();
  //blebas.write(100);

  // Setup the advertising packet(s)
  Serial.print("Setting up the scanning ...");
  Bluefruit.setName(tag_name);
  //startAdv();
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80);       // in units of 0.625 ms
  //Bluefruit.Scanner.filterUuid(BLEUART_UUID_SERVICE);
  //Bluefruit.Scanner.filterUuid("BLEUART_UUID_SERVICE");
  //Bluefruit.Scanner.filterUuid(BLEUuid("b0001"));  // Add this line to filter devices by name
  Bluefruit.Scanner.useActiveScan(false);       // Don't request scan response data
  Bluefruit.Scanner.start(0);                   // 0 = Don't stop scanning after n seconds
  setLED(false,"b");
  Serial.println(" done ...");

  //bio stuff
  // set the data rate for the sensor serial port
  finger.begin(57600);
  //delay(2000);
  

  //now setup advertising for getting commands from phone
  startAdv();

}



// biometrics
void printHex(int num, int precision) {
  char tmp[16];
  char format[128];

  sprintf(format, "%%.%dX", precision);

  sprintf(tmp, format, num);
  Serial.print(tmp);
}

uint8_t downloadFingerprintTemplate(uint16_t id)
{
  Serial.println("------------------------------------");
  Serial.print("Attempting to load #"); Serial.println(id);
  uint8_t p = finger.loadModel(id);
  switch (p) {
    case FINGERPRINT_OK:
      Serial.print("Template "); Serial.print(id); Serial.println(" loaded");
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    default:
      Serial.print("Unknown error "); Serial.println(p);
      return p;
  }

  // OK success!

  Serial.print("Attempting to get #"); Serial.println(id);
  p = finger.getModel();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.print("Template "); Serial.print(id); Serial.println(" transferring:");
      break;
    default:
      Serial.print("Unknown error "); Serial.println(p);
      return p;
  }

  // one data packet is 267 bytes. in one data packet, 11 bytes are 'usesless' :D
  uint8_t bytesReceived[534]; // 2 data packets
  memset(bytesReceived, 0xff, 534);

  uint32_t starttime = millis();
  int i = 0;
  while (i < 534 && (millis() - starttime) < 20000) {
    if (bioSerial.available()) {
      bytesReceived[i++] = bioSerial.read();
    }
  }
  Serial.print(i); Serial.println(" bytes read.");
  Serial.println("Decoding packet...");

  uint8_t fingerTemplate[512]; // the real template
  memset(fingerTemplate, 0xff, 512);

  // filtering only the data packets
  int uindx = 9, index = 0;
  memcpy(fingerTemplate + index, bytesReceived + uindx, 256);   // first 256 bytes
  uindx += 256;       // skip data
  uindx += 2;         // skip checksum
  uindx += 9;         // skip next header
  index += 256;       // advance pointer
  memcpy(fingerTemplate + index, bytesReceived + uindx, 256);   // second 256 bytes

  for (int i = 0; i < 512; ++i) {
    //Serial.print("0x");
    printHex(fingerTemplate[i], 2);
    //Serial.print(", ");
  }
  Serial.println("\ndone.");

  return p;

  /*
    uint8_t templateBuffer[256];
    memset(templateBuffer, 0xff, 256);  //zero out template buffer
    int index=0;
    uint32_t starttime = millis();
    while ((index < 256) && ((millis() - starttime) < 1000))
    {
    if (mySerial.available())
    {
      templateBuffer[index] = mySerial.read();
      index++;
    }
    }

    Serial.print(index); Serial.println(" bytes read");

    //dump entire templateBuffer.  This prints out 16 lines of 16 bytes
    for (int count= 0; count < 16; count++)
    {
    for (int i = 0; i < 16; i++)
    {
      Serial.print("0x");
      Serial.print(templateBuffer[count*16+i], HEX);
      Serial.print(", ");
    }
    Serial.println();
    }*/
}

//enrollment fingerprint suff
int fp_template_count = 0;
uint8_t getFingerprintEnroll() {

  int p = -1;
  Serial.print("Waiting for valid finger to enroll as #"); //Serial.println(id);
  //make the next ID
  fp_template_count = finger.templateCount;
  while (p != FINGERPRINT_OK and enrollment_mode) {
    setLED(true,"y");
    p = finger.getImage();
    switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      pulseLED(2,"g");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.println(".");
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      pulseLED(2,"r");
      break;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      pulseLED(2,"r");
      break;
    default:
      Serial.println("Unknown error");
      pulseLED(2,"r");
      break;
    }
  }
  if(!enrollment_mode)
    return -1;
  
  // OK success!
  p = finger.image2Tz(1);
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      pulseLED(2,"r");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      pulseLED(2,"r");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      pulseLED(2,"r");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      pulseLED(2,"r");
      return p;
    default:
      Serial.println("Unknown error");
      pulseLED(2,"r");
      return p;
  }

  Serial.println("Remove finger");
  pulseLED(2,"g");
  delay(2000);
  p = 0;
  while (p != FINGERPRINT_NOFINGER) {
    p = finger.getImage();
  }
  Serial.print("ID "); Serial.println(fp_template_count);
  p = -1;
  Serial.println("Place same finger again");
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.print(".");
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      pulseLED(2,"r");
      break;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      pulseLED(2,"r");
      break;
    default:
      Serial.println("Unknown error");
      pulseLED(2,"r");
      break;
    }
  }
  pulseLED(2,"g");
  // OK success!

  p = finger.image2Tz(2);
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      pulseLED(2,"r");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      pulseLED(2,"r");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      pulseLED(2,"r");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      pulseLED(2,"r");
      return p;
    default:
      Serial.println("Unknown error");
      pulseLED(2,"r");
      return p;
  }

  // OK converted!
  Serial.print("Creating model for #");  Serial.println(fp_template_count);
  pulseLED(2,"g");

  p = finger.createModel();
  if (p == FINGERPRINT_OK) {
    Serial.println("Prints matched!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    pulseLED(2,"r");
    return p;
  } else if (p == FINGERPRINT_ENROLLMISMATCH) {
    Serial.println("Fingerprints did not match");
    pulseLED(2,"r");
    return p;
  } else {
    Serial.println("Unknown error");
    pulseLED(2,"r");
    return p;
  }

  Serial.print("ID "); Serial.println(fp_template_count);
  p = finger.storeModel(fp_template_count);
  if (p == FINGERPRINT_OK) {
    Serial.println("Stored!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    pulseLED(2,"r");
    return p;
  } else if (p == FINGERPRINT_BADLOCATION) {
    Serial.println("Could not store in that location");
    pulseLED(2,"r");
    return p;
  } else if (p == FINGERPRINT_FLASHERR) {
    Serial.println("Error writing to flash");
    pulseLED(2,"r");
    return p;
  } else {
    Serial.println("Unknown error");
    pulseLED(2,"r");
    return p;
  }

  return true;
}

uint8_t getFingerprintIDez() {
  uint8_t p = finger.getImage();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.println("No finger detected");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // OK success!

  p = finger.image2Tz();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // OK converted!
  p = finger.fingerSearch();
  if (p == FINGERPRINT_OK) {
    Serial.println("Found a print match!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    return p;
  } else if (p == FINGERPRINT_NOTFOUND) {
    Serial.println("Did not find a match");
    return p;
  } else {
    Serial.println("Unknown error");
    return p;
  }

  // found a match!
  Serial.print("Found ID #"); Serial.print(finger.fingerID);
  Serial.print(" with confidence of "); Serial.println(finger.confidence);

  return finger.fingerID;
}

// returns -1 if failed, otherwise returns ID #
// int getFingerprintIDez() {
//   uint8_t p = finger.getImage();
//   if (p != FINGERPRINT_OK)  return -1;

//   p = finger.image2Tz();
//   if (p != FINGERPRINT_OK)  return -1;

//   p = finger.fingerFastSearch();
//   if (p != FINGERPRINT_OK)  return -1;

//   // found a match!
//   Serial.print("Found ID #"); Serial.print(finger.fingerID);
//   Serial.print(" with confidence of "); Serial.println(finger.confidence);
//   return finger.fingerID;
// }

void startAdv(void)
{
  // Central advertising packet
  Bluefruit.Central.setConnectCallback(pher_connect_callback);
  Bluefruit.Central.setDisconnectCallback(pher_disconnect_callback);

  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include tag Service UUID
  Bluefruit.Advertising.addService(central_service);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  // Start Advertising
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}


void setLED(bool on, String clr)
{
  // data = 1 -> LED = On
  // data = 0 -> LED = Off
  digitalWrite(LED_BUILTIN, on ? LED_STATE_ON : (1-LED_STATE_ON));
  #ifdef PIN_NEOPIXEL
    uint32_t c;

    // Convert the color string to lowercase for case-insensitive comparison
    clr.toLowerCase();

    // Use a case statement to determine the color
    switch (clr[0]) {
      case 'b': // Blue
        c = neopixel.Color(0x00, 0x00, on ? 0x20 : 0x00);
        break;
      case 'y': // Yellow
        c = neopixel.Color(on ? 0x20 : 0x00, on ? 0x20 : 0x00, 0x00);
        break;
      default: // Default to blue for unsupported colors
        c = neopixel.Color(0x00, 0x00, on ? 0x20 : 0x00);
        break;
    }

    neopixel.fill(c, 0, NEOPIXEL_NUM);
    neopixel.show();
    // If the LED is turned off, set all pixels to black (0, 0, 0)
    if (!on) {
      neopixel.fill(neopixel.Color(0, 0, 0), 0, NEOPIXEL_NUM);
      neopixel.show();
    }
  #endif
}

void pulseLED(int times, String color)
{
  // data = 1 -> pulse = On => 250msec then off
  for (int i = 0; i < times; i++) {
    // digitalWrite(LED_BUILTIN, LED_STATE_ON);
    // delay(250);
    // digitalWrite(LED_BUILTIN, 1-LED_STATE_ON);
    // delay(250);
    digitalWrite(pulse_pin, HIGH);
    setLED(true,color);
    delay(1000);
    digitalWrite(pulse_pin, LOW);
    delay(400);
    setLED(false,color);
  }
}

void switchMachine(int on)
{
  switch (on) {
    case 0:
      digitalWrite(switchPin6, LOW);
      Serial.println("Switched machine to " + String(on ? "ON" : "OFF"));
      break;
    case 1:
      if (low_lvl_pin_state != 0){
        digitalWrite(switchPin6, HIGH);
        Serial.println("Switched machine to " + String(on ? "ON" : "OFF"));
      }
      else Serial.println("Cannot switch machine, level error!");
     
      break;
    // Add more cases here if needed
    default:
      Serial.println("Cannot switch machine, invalid option!");
      break;
  }
  
}



void loop()
{
  //testing the characterlink by switching the relay on and off on pheripheral
  // delay(10000);
  // Serial.println( tag_chr_switch.write8( 0x01 ) ); 
  // delay(10000);
  // Serial.println( tag_chr_switch.write8( 0x00 ) ); 

  //bio stuff
  Serial.println("Looking for fingerprint sensor!");
  if (finger.verifyPassword()) {
    Serial.println("Found fingerprint sensor!");
    Serial.println(F("Reading sensor parameters"));
  finger.getParameters();
  Serial.print(F("Status: 0x")); Serial.println(finger.status_reg, HEX);
  Serial.print(F("Sys ID: 0x")); Serial.println(finger.system_id, HEX);
  Serial.print(F("Capacity: ")); Serial.println(finger.capacity);
  Serial.print(F("Security level: ")); Serial.println(finger.security_level);
  Serial.print(F("Device address: ")); Serial.println(finger.device_addr, HEX);
  Serial.print(F("Packet len: ")); Serial.println(finger.packet_len);
  Serial.print(F("Baud rate: ")); Serial.println(finger.baud_rate);

  finger.getTemplateCount();

  if (finger.templateCount == 0) {
    Serial.print("Sensor doesn't contain any fingerprint data. Please run the 'enroll' example.");
    pulseLED(2,"r");
  }
  else {
    Serial.println("Waiting for valid finger...");
      Serial.print("Sensor contains "); Serial.print(finger.templateCount); Serial.println(" templates");
      pulseLED(2,"g");
  }
  } else {
    Serial.println("Did not find fingerprint sensor :( \n");
  }

  if(enrollment_mode) {
    Serial.println("Enrollment mode");
    int id = getFingerprintIDez();
    if (id >= 0) {
      Serial.print("Found ID #"); Serial.print(id);
      Serial.print("\n");
      downloadFingerprintTemplate(id);
    }
  }
  else {
    Serial.println("Normal mode");
    int id = getFingerprintIDez();
    if (id >= 0) {
      Serial.print("Found ID #"); Serial.print(id);
      Serial.print("\n");
      downloadFingerprintTemplate(id);
    }
  }




  // First check if we are connected to any peripherals
  // if ( Bluefruit.Central.connected() )
  // {
  //   // default MTU with an extra byte for string terminator
  //   char buf[20+1] = { 0 };
    
  //   // Read from HW Serial (normally USB Serial) and send to all peripherals
  //   if ( Serial.readBytes(buf, sizeof(buf)-1) )
  //   {
  //     sendAll(buf);
  //   }
  // }

  //now low and high level notifications
  // uint8_t newState_low_lvl = (BUTTON_ACTIVE == digitalRead(low_lvl_pin));
  // // only notify if button state changes
  // if ( newState_low_lvl != low_lvl_pin_state)
  // {
  //   low_lvl_pin_state = newState_low_lvl;
  //   tag_chr_nfy_low.write8(low_lvl_pin_state);

  //   // notify all connected clients
  //   for (uint16_t conn_hdl=0; conn_hdl < MAX_PRPH_CONNECTION; conn_hdl++)
  //   {
  //     if ( Bluefruit.connected(conn_hdl) && tag_chr_nfy_low.notifyEnabled(conn_hdl) )
  //     {
  //       tag_chr_nfy_low.notify8(conn_hdl, low_lvl_pin_state);
  //     }
  //   }
  // }

 
}//loop


void printDeviceInfo(const prph_info_t& info) {
  Serial.print("Timestamp Addr              Rssi Data DeviceName\n");
  Serial.printf("%09d ", millis());
  Serial.printBufferReverse(info.addr, 6, ':');
  Serial.print(" ");
  Serial.print(info.rssi);
  Serial.print("  ");
  Serial.printBuffer(info.data, sizeof(info.data), '-');
  Serial.print(" ");
  Serial.println(info.deviceName);
}

void printUuid16List(uint8_t* buffer, uint8_t len)
{
  Serial.printf("%14s %s", "16-Bit UUID");
  for(int i=0; i<len; i+=2)
  {
    uint16_t uuid16;
    memcpy(&uuid16, buffer+i, 2);
    Serial.printf("%04X ", uuid16);
  }
  Serial.println();
}

void printUuid128List(uint8_t* buffer, uint8_t len)
{
  (void) len;
  Serial.printf("%14s %s", "128-Bit UUID");

  // Print reversed order
  for(int i=0; i<16; i++)
  {
    const char* fm = (i==4 || i==6 || i==8 || i==10) ? "-%02X" : "%02X";
    Serial.printf(fm, buffer[15-i]);
  }

  Serial.println();  
}

const char* targetNames[] = {"b00", "c00"}; // List of substrings to check for in device names
const size_t numTargetNames = sizeof(targetNames) / sizeof(targetNames[0]);
bool containsSubstring(const String& str, const char* substr) {
  return str.indexOf(substr) != -1;
}

void connectToPeripheral(ble_gap_evt_adv_report_t* report) {
  for (uint8_t i = 0; i < BLE_MAX_CONNECTION; ++i) {
    if (prphs[i].conn_handle == BLE_CONN_HANDLE_INVALID) {
      if (report->type.connectable) {
        Serial.print("Connecting to ");
        Serial.print(prphs[i].deviceName);
        Serial.println("...");
        prphs[i].conn_handle = Bluefruit.Central.connect(report);
      }
      break;
    }
  }
}

void reverse_chr_array(uint8_t *chr_array, int len) {
  uint8_t temp;
  for (int i = 0; i < ceil(len / 2); i++) {
    temp = chr_array[i];
    chr_array[i] = chr_array[len - 1 - i];
    chr_array[len - 1 - i] = temp; 
  }
}

void add_whitelist(uint8_t *addr) {
  if (count_wl < BLE_MAX_CONNECTION) {
    memcpy(whitelist[count_wl], addr, 6);
    count_wl++;
    Serial.print("Added ");
    Serial.printBuffer(addr, 6, ':');
    Serial.println();
  }
  else {
    Serial.println("Whitelist full!");
  }
}

int is_whitelisted(uint8_t *addr) {
  for (int i = 0; i < BLE_MAX_CONNECTION; i++) {
      if (memcmp(addr, whitelist[i], 6) == 0) return i;
  }
  return -1;
}

void shift_chr_array(uint8_t chr_array[][6], int pos, int shift) {
  int len = sizeof(chr_array) * sizeof(chr_array[0]);
  for (int j = 0; j < shift; j++) {
    for (int i = pos; i < len - 1; i++) {
      *(chr_array[0] + i) = *(chr_array[0] + i + 1);
      if (i == len - 2) {
        *(chr_array[0] + i + 1) = 0;
      }
    }
  }
}

int byte_to_int(uint8_t *bytes, uint16_t len) {
  int val = 0;
  for (int i = 0; i < len; i++) {
    val += bytes[i] << (i * 8);
  }
  return val;
}

void parse_address(char* address, uint8_t* buf) {
  char * pEnd;

  buf[0] = strtol(address, &pEnd, 16);
  buf[1] = strtol(pEnd+1, &pEnd, 16);
  buf[2] = strtol(pEnd+1, &pEnd, 16);
  buf[3] = strtol(pEnd+1, &pEnd, 16);
  buf[4] = strtol(pEnd+1, &pEnd, 16);
  buf[5] = strtol(pEnd+1, NULL, 16);
}

void del_whitelist(uint8_t *addr){
  int id = is_whitelisted(addr);
  if (id >= 0) {
     Serial.println(id);
     shift_chr_array(whitelist, id * 6, 6);
     count_wl--;
     // TODO Disconnect units with matching addr
  }
}



void printWhitelist() {
  Serial.print("[");
  Serial.print(count_wl, DEC);
  for (int i = 0; i < count_wl; i++) {
    Serial.print(',');
    Serial.printBuffer(whitelist[i], 6, ':');
  }
  Serial.println("]");
}

void printArray() {
  for (unsigned int i = 0; i < sizeof(whitelist); i++) {
    Serial.print(*(whitelist[0] + i));
    Serial.print(" ");
  }
  Serial.println();
}

void printNotify(prph_info_t* peer) {
  Serial.printf("!NO ");
  Serial.printBuffer(peer->addr, 6, ':');
  Serial.printf(" %d#", peer->batt);
  Serial.println();
}

void printStatus() {
  char active = 0;
  Serial.print("[");
  for(int id = 0; id < BLE_MAX_CONNECTION; id++) {
    if (prphs[id].conn_handle != BLE_CONN_HANDLE_INVALID) {
      active++;
      if (id > 0)
        Serial.print(",");
      Serial.print("[");
      Serial.print(prphs[id].deviceName);
      Serial.print(",");
      Serial.print(prphs[id].rssi);
      Serial.print(",");
      Serial.print(prphs[id].batt);
      Serial.print(",");
      Serial.printBuffer(prphs[id].addr, 6, ':');
      Serial.print("]");
    }
  }
  Serial.println("]");
}

void scan_callback(ble_gap_evt_adv_report_t* report)
{
  PRINT_LOCATION();
  //uint8_t len = 0;
  uint8_t buffer[32];
  memset(buffer, 0, sizeof(buffer));
  
  //only report if connectable
  /* Display the timestamp and device address */
  //Serial.printf("%14s ", "ADV TYPE");
  if ( report->type.connectable ) 
    {
      int id  = findConnHandle(BLE_CONN_HANDLE_INVALID);
      prph_info_t* peer = &prphs[id];
      Bluefruit.getName(peer->deviceName, sizeof(peer->deviceName));
      reverse_chr_array(peer->addr, 6);
      Serial.printf("Found %s ", peer->deviceName);
      Serial.println();
      Serial.printBufferReverse(report->peer_addr.addr, 6, ':');
      /* RSSI value */
      Serial.printf("%14s %d dBm\n", "RSSI", report->rssi);
      Serial.println();
      Bluefruit.Central.connect(report);
      }
      else
      {
        Serial.print("Device Non-connectable "); //comment this if you want to see all devices scanned
      }
  Serial.println();

  // For Softdevice v6: after received a report, scanner will be paused
  // We need to call Scanner resume() to continue scanning
  Bluefruit.Scanner.resume();
} //scan_callback



// callback invoked when central connects
//ideas from here
void connect_callback(uint16_t conn_handle)
{
  //(void) conn_handle;

  pulseLED(2, "b");	
  Serial.println("");
  Serial.print("Connect Callback, conn_handle: "); 
  Serial.println( conn_handle );
   
  // If Service is not found, disconnect and return
  Serial.print("Discovering Service ... ");
  if ( !tag_service.discover(conn_handle) )
  {
    Serial.println("No Service Found");

    // disconnect since we couldn't find service
    Bluefruit.disconnect(conn_handle);

    return;
  } 
  Serial.println("Service Found");

  // Once service is found, we continue to discover the primary characteristic
  Serial.print("Discovering Characteristic ... ");
  if ( !tag_chr_switch.discover() )
  {
    // Measurement chr is mandatory, if it is not found (valid), then disconnect
    Serial.println("No Characteristic Found. Characteristic is mandatory but not found. ");
    Bluefruit.disconnect(conn_handle);
    return;
  }
  Serial.println("Characteristic Found");
 
  // Data Collection Charactistic. Find and enable. 
  // You enable data collection on the Characteristic before the peripheral will start measuring values.
  // This is different than setting the Notify descriptor, which is handled below.
  // Serial.print("Discovering Data Collection Configuration Characteristic ... ");
  // if ( !tagCharacteristicSwitch.discover() )
  // {
  //   Serial.println("No Switch Characteristic Found. Characteristic is mandatory but not found.");   
  //   Bluefruit.disconnect(conn_handle);
  //   return;
  // }
  // Serial.println("Switch Characteristic Found"); 
  // Found it, now write 0x01 to turn on optical data collection
  // Serial.print("Enabling Data Collection: ");
  // Serial.println( tagCharacteristic.write8( 0x01 ) );
  // // Reaching here means we are ready to go, let's enable
  // Serial.print("Enabling Notify on the Characteristic ... ");
  // if ( tagCharacteristic.enableNotify() )
  // {
  //   Serial.println("enableNotify success, now ready to receive Characteristic values");
  // }else
  // {
  //   Serial.println("Couldn't enable notify for Characteristic. Increase DEBUG LEVEL for troubleshooting");
  // }
 
  // Measurement Period Characteristic. Find and adjust. 
  // Serial.print("Measurement Period Characteristic ... ");
  // if ( !opticalCharacteristicPeriod.discover() )
  // {
  //   Serial.println("No Characteristic Found, but not mandatory so not disconnecting");    
  // }
  // Serial.println("Characteristic Found");
  // Found it, now adjust it: 
  // Resolution 10 ms. Range 100 ms (0x0A) to 2.55 sec (0xFF). Default is 800 milliseconds (0x50).
  // 19 is 250
  // Serial.print("Adjusting Measurement Period, return value: ");
  // // Serial.println( opticalCharacteristicPeriod.write8( 0xFF ) ); // Slowest
  // Serial.println( opticalCharacteristicPeriod.write8( 0x0A ) ); // Fastest

  // notification
  




  //lsbLED.write8(0x01);

  //do battery read
  // float measuredvbat = analogRead(VBATPIN);
  // measuredvbat *= 2;    // we divided by 2, so multiply back
  // measuredvbat *= 3.55;  // Multiply by 3.3V, our reference voltage
  // measuredvbat /= 1024; // convert to voltage
  // measuredvbat /= 4.1;
  //  measuredvbat *= 100; //now %
  // measuredvbat = floor(measuredvbat);
  // int vbat_int = round(measuredvbat);
  // //blebas.write(vbat_int);
  // tag_chr_nfy_low.write8(vbat_int);
  // Serial.print("VBat: " ); Serial.println(vbat_int);

  // If HRM is not found, disconnect and return
  // Find an available ID to use
  // Scan for more devices
  //Bluefruit.Scanner.start(0);

  //Bluefruit.getName(conn_handle->deviceName, sizeof(peer->deviceName));
  //Bluefruit.Gap.getPeerName(conn_handle, peer->name, 32);
  // Bluefruit.getAddr(peer->addr, sizeof(peer->addr));
  // reverse_chr_array(peer->addr, 6);

  // Serial.printf("Connected to %s ", peer->deviceName);
  // Serial.printBuffer(peer->addr, 6, ':');
  // Serial.println();
  
  // Find an available ID to use
  //int id  = findConnHandle(BLE_CONN_HANDLE_INVALID);
  // BLEConnection* conn = Bluefruit.Connection(conn_handle);
  // prph_info_t* peer;
  // peer->conn_handle = conn_handle;
  // //memcpy(peer->addr, conn->getPeerAddr(), 6);
  // memcpy(peer->addr, &(conn->getPeerAddr()), sizeof(conn->getPeerAddr()));
  // reverse_chr_array(peer->addr, 6);
  // //conn.

  // // Eeek: Exceeded the number of connections !!!
  // if ( id < 0 ) return;
  
  // prph_info_t* peer = &prphs[id];
  // peer->conn_handle = conn_handle;
  
  // Bluefruit.getName(peer->deviceName, sizeof(peer->deviceName));
  // //Bluefruit.Gap.getPeerName(conn_handle, peer->name, 32);
  // // Bluefruit.getAddr(peer->addr, sizeof(peer->addr));
  // // reverse_chr_array(peer->addr, 6);

  // Serial.printf("Connected to %s ", peer->deviceName);
  // Serial.printBuffer(peer->addr, 6, ':');
  // Serial.println();

  // Serial.print("!OC ");
  // Serial.printBuffer(peer->addr, 6, ':');
  // Serial.println("#");
  
  connection_num++;

  // Call BLECentralService discover
  // peer->_service.discover(id);

  // // Discover characteristics
  // BLEClientCharacteristic* chr_arr[] = { &peer->_notification, &peer->_data };
  // Bluefruit.Discovery.discoverCharacteristic(id, chr_arr, 2);

  // if (peer->_data.read(Readbuf, 1) > 0) {
  //   if (Readbuf[0] > 100) {
  //     //peer->buckled_state = 1;
  //     peer->batt = Readbuf[0] - 100;
  //   }
  //   else {
  //     //peer->buckled_state = 0;
  //     peer->batt = Readbuf[0];
  //   }
  // }
  // printNotify(peer);

  // peer->_notification.enableNotify();
  // peer->_data.enableNotify();
  }  

  
 // }

//   connection_count++;
//   Serial.print("Connection count: ");
//   Serial.println(connection_count);


//   // Keep advertising if not reaching max
//   // if (connection_count < MAX_PRPH_CONNECTION)
//   // {
//   //   Serial.println("Keep advertising");
//   //   Bluefruit.Advertising.start(0);
//   // }
// }

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  connection_num--;

  // Mark the ID as invalid
  int id  = findConnHandle(conn_handle);

  // Non-existant connection, something went wrong, DBG !!!
  if ( id < 0 ) return;

  // Mark conn handle as invalid
  prphs[id].conn_handle = BLE_CONN_HANDLE_INVALID;

  Serial.print(prphs[id].deviceName);
  Serial.println(" disconnected!");

  setLED(true,"y");
  delay(1000);
  setLED(false,"y");
  //lsbLED.write8(0x00);
  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);

  connection_count--;
  // For Softdevice v6: after received a report, scanner will be paused
  // We need to call Scanner resume() to continue scanning
  Bluefruit.Scanner.resume();
}

/**
 * Callback invoked when BLE UART data is received
 * @param uart_svc Reference object to the service where the data 
 * arrived.
 */
void pher_connect_callback(uint16_t conn_handle)
{
  // As of Adafruit nRF52 Arduino 0.10.1, use BLEConnection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  
  // handle checks for running sessions and other reconnection details here
  char central[32] = { 0 };
  connection->getPeerName(central, sizeof(central));
  Serial.print("Connected to ");
  Serial.println(central);
  pulseLED(2, "b");
  connection_count++;
  Serial.print("Connection count: ");
  Serial.println(connection_count);
  //now set the fingerprintscanner in enrollment mode
  enrollment_mode = true;
  setLED(true,"y");
  while (1) {
    if (getFingerprintEnroll() == FINGERPRINT_OK || !enrollment_mode ) {
      Bluefruit.disconnect(conn_handle);
      break;
    }
  }
  
  //(void) conn_handle;

  
  //lsbLED.write8(0x01);

  //do battery read
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.55;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  measuredvbat /= 4.1;
   measuredvbat *= 100; //now %
  measuredvbat = floor(measuredvbat);
  int vbat_int = round(measuredvbat);
  //blebas.write(vbat_int);
  //tag_chr_nfy_low.write8(vbat_int);
  Serial.print("VBat: " ); Serial.println(vbat_int);
  // Keep advertising if not reaching max
  // if (connection_count < MAX_PRPH_CONNECTION)
  // {
  //   Serial.println("Keep advertising");
  //   Bluefruit.Advertising.start(0);
  // }
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void pher_disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  setLED(true,"y");
  delay(1000);
  setLED(false,"y");
  //lsbLED.write8(0x00);
  Serial.println();
  Serial.print("Pheripharal Disconnected from phone, reason = 0x"); Serial.println(reason, HEX);

  connection_count--;

  //Keep advertising if not reaching max
  if (connection_count < MAX_PRPH_CONNECTION)
  {
    Serial.println("Keep advertising");
    Bluefruit.Advertising.start(0);
  }
}


/**
 * Find the connection handle in the peripheral array
 * @param conn_handle Connection handle
 * @return array index if found, otherwise -1
 */
int findConnHandle(uint16_t conn_handle)
{
  for(int id=0; id<BLE_MAX_CONNECTION; id++)
  {
    if (conn_handle == prphs[id].conn_handle)
    {
      return id;
    }
  }

  return -1;  
}

/**
 * Software Timer callback is invoked via a built-in FreeRTOS thread with
 * minimal stack size. Therefore it should be as simple as possible. If
 * a periodically heavy task is needed, please use Scheduler.startLoop() to
 * create a dedicated task for it.
 * 
 * More information http://www.freertos.org/RTOS-software-timer.html
 */
void blink_timer_callback(TimerHandle_t xTimerID) {
  (void) xTimerID;

  // Period of sequence is 10 times (2 second). 
  // RED LED will toggle first 2*n times (on/off) and remain off for the rest of period
  // Where n = number of connection
  static uint8_t count = 0;

  if ( count < 2 * connection_num) digitalToggle(LED_RED);
  if ( count % 2 && digitalRead(LED_RED)) digitalWrite(LED_RED, LOW);
  
  count++;
  if (count >= 10) count = 0;
}

