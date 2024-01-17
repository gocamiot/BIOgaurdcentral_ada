
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

//list functions
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void setLED(bool on, String clr);
void switchMachine(int on);
void startAdv(void);


// max concurrent connections supported by this example
#define MAX_PRPH_CONNECTION   3
uint8_t connection_count = 0;
#define NO_ADVERTISING_TIMEOUT   0 // seconds
#define VBATPIN A1

//tag_name must be unique for every tag
char tag_name[8] = "b0001";
const char ADVERTISING_NAME_PREFIX[] = "b0001";
char tag_ver[16] = "BIOgaurd 240110";

// Service UUID used to differentiate this device. Note that the byte order is reversed.
// The UUID below corresponds to 5aee7fb1-7d16-4103-9dd7-2c2f9fdf2b48
const uint8_t TAG_UUID_SERVICE[] =
      {
            0x48, 0x2b, 0xdf, 0x9f, 0x2f, 0x2c, 0xd7, 0x9d,
            0x03, 0x41, 0x16, 0x7d, 0xb1, 0x7f, 0xee, 0x5a
      }; 
const uint8_t TAG_UUID_CHR_HS[] =
      {
            0x48, 0x2b, 0xdf, 0x9f, 0x2f, 0x2c, 0xd7, 0x9d,
            0x03, 0x41, 0x16, 0x7d, 0xb2, 0x7f, 0xee, 0x5a
      };
uint8_t tag_hs;  

BLEService tag_service = BLEService(BLEUuid(TAG_UUID_SERVICE));
BLECharacteristic tag_chr_hs = BLECharacteristic(BLEUuid(TAG_UUID_CHR_HS));
BLEBas blebas;    // BAS (Battery Service) helper class instance

const uint8_t LBS_UUID_SERVICE[] =
      {
            0x48, 0x2b, 0xdf, 0x9f, 0x2f, 0x2c, 0xd7, 0x9d,
            0x03, 0x41, 0x16, 0x7d, 0xb3, 0x7f, 0xee, 0x5a
      };

const uint8_t LBS_UUID_CHR_BUTTON[] =
      {
            0x48, 0x2b, 0xdf, 0x9f, 0x2f, 0x2c, 0xd7, 0x9d,
            0x03, 0x41, 0x16, 0x7d, 0xb4, 0x7f, 0xee, 0x5a
      }; 
      
const uint8_t LBS_UUID_CHR_ON_SWITCH[] =
      {
            0x48, 0x2b, 0xdf, 0x9f, 0x2f, 0x2c, 0xd7, 0x9d,
            0x03, 0x41, 0x16, 0x7d, 0xb5, 0x7f, 0xee, 0x5a
      };

//we can use this for pushbutton notice for like a knock sensor.
const char CHR_NFY_LOW_LVL[] = "LOW_NFY";
const uint8_t CHR_UUID_LOW_LVL[] =
      {
            0x48, 0x2b, 0xdf, 0x9f, 0x2f, 0x2c, 0xd7, 0x9d,
            0x03, 0x41, 0x16, 0x7d, 0xb6, 0x7f, 0xee, 0x5a
      };

BLEService        lbs(LBS_UUID_SERVICE);
BLECharacteristic lsbButton(LBS_UUID_CHR_BUTTON);
BLECharacteristic lbsSwitch(LBS_UUID_CHR_ON_SWITCH);
BLECharacteristic tag_chr_nfy_low(CHR_UUID_LOW_LVL);

//pulsepin for relay pulse, works with LED flash
#define PULSE_PIN_ACTIVE     LOW
uint8_t pulse_pin = A1;
uint8_t pulse_pin_state;
uint8_t switchPin6 = 6;
uint8_t on_state;
uint8_t low_lvl_pin = 5;
uint8_t low_lvl_pin_state;


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
  pinMode(button, BUTTON_ACTIVE ? INPUT_PULLDOWN : INPUT_PULLUP);
  buttonState = (BUTTON_ACTIVE == digitalRead(button));
  pinMode(pulse_pin, OUTPUT);
  digitalWrite(pulse_pin, LOW); // pulse low
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 1-LED_STATE_ON); // led off
  pinMode(switchPin6, OUTPUT);
  digitalWrite(switchPin6, LOW); // machine off
  pinMode(low_lvl_pin, BUTTON_ACTIVE ? INPUT_PULLDOWN : INPUT_PULLUP);
  low_lvl_pin_state = (BUTTON_ACTIVE == digitalRead(low_lvl_pin));

  #ifdef PIN_NEOPIXEL
    neopixel.begin();
  #endif

  Serial.begin(115200);
   delay(100);
  Serial.println(tag_ver);
  Serial.println("------------------------------\n");

  // Initialize Bluefruit with max concurrent connections as Peripheral = MAX_PRPH_CONNECTION, Central = 0
  //Serial.println("Initialise the Bluefruit nRF52 module");
  Bluefruit.begin(MAX_PRPH_CONNECTION, 0);
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Setup manufacturer tag service
  Serial.println("Configuring the Manufacturer Tag Service");

  // Note: You must call .begin() on the BLEService before calling .begin() on
  // any characteristic(s) within that service definition.. Calling .begin() on
  // a BLECharacteristic will cause it to be added to the last BLEService that
  // was 'begin()'ed!
  tag_service.begin();

  // Configure Button characteristic
  // Properties = Read + Notify
  // Permission = Open to read, cannot write
  // Fixed Len  = 1 (button state)
  tag_chr_hs.setProperties(CHR_PROPS_READ);
  tag_chr_hs.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  tag_chr_hs.setFixedLen(2);
  tag_chr_hs.begin();
  tag_chr_hs.write8(tag_hs);
  //tag_chr_hs.setWriteCallback(tag_write_callback);


  // Setup the LED-Button service using
  Serial.println("Configuring the LED-Button Service");
  lbs.begin();

  // Configure Button characteristic
  // Properties = Read + Notify
  // Permission = Open to read, cannot write
  // Fixed Len  = 1 (button state)
  lsbButton.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  lsbButton.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  lsbButton.setFixedLen(1);
  lsbButton.begin();
  lsbButton.write8(buttonState);

  lbsSwitch.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  lbsSwitch.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  lbsSwitch.setFixedLen(1);
  lbsSwitch.begin();
  lbsSwitch.setWriteCallback(switch6_callback);

  
  tag_chr_nfy_low.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  tag_chr_nfy_low.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  tag_chr_nfy_low.setFixedLen(1);
  tag_chr_nfy_low.begin();
  tag_chr_nfy_low.write8(low_lvl_pin_state);


// Start the BLE Battery Service and set it to 100%
  Serial.println("Configuring the Battery Service");
  blebas.begin();
  //blebas.write(100);

  // Setup the advertising packet(s)
  Serial.println("Setting up the advertising");
  Bluefruit.setName(tag_name);
  startAdv();

  setLED(false,"b");
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include HRM Service UUID
  Bluefruit.Advertising.addService(lbs);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  // Include characteristic names in the scan response/ does not work..
  //Bluefruit.ScanResponse.addManufacturerData(tag_chr_nfy_read.uuid, (uint8_t*)CHR_NAME_NFY_READ, strlen(CHR_NAME_NFY_READ));
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
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

void pulseLED()
{
  // data = 1 -> pulse = On => 250msec then off
  digitalWrite(pulse_pin, HIGH);
  setLED(true,"b");
  delay(1000);
  digitalWrite(pulse_pin, LOW);
  delay(400);
  setLED(false,"b");
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
  delay(10); 

  //now low and high level notifications
  uint8_t newState_low_lvl = (BUTTON_ACTIVE == digitalRead(low_lvl_pin));
  // only notify if button state changes
  if ( newState_low_lvl != low_lvl_pin_state)
  {
    low_lvl_pin_state = newState_low_lvl;
    tag_chr_nfy_low.write8(low_lvl_pin_state);

    // notify all connected clients
    for (uint16_t conn_hdl=0; conn_hdl < MAX_PRPH_CONNECTION; conn_hdl++)
    {
      if ( Bluefruit.connected(conn_hdl) && tag_chr_nfy_low.notifyEnabled(conn_hdl) )
      {
        tag_chr_nfy_low.notify8(conn_hdl, low_lvl_pin_state);
      }
    }
  }

 
}//loop



// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  (void) conn_handle;

  pulseLED();
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
  tag_chr_nfy_low.write8(vbat_int);
  Serial.print("VBat: " ); Serial.println(vbat_int);

  connection_count++;
  Serial.print("Connection count: ");
  Serial.println(connection_count);

  // Keep advertising if not reaching max
  if (connection_count < MAX_PRPH_CONNECTION)
  {
    Serial.println("Keep advertising");
    Bluefruit.Advertising.start(0);
  }
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  setLED(true,"y");
  delay(1000);
  setLED(false,"y");
  //lsbLED.write8(0x00);
  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);

  connection_count--;
}
