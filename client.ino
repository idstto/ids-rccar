#include <Arduino.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>
#include <SPI.h>
// #include <Adafruit_PWMServoDriver.h>
#include <PCF8574.h>

#define DEBUG
#ifdef DEBUG
  #define DEBUG_PRINT(x)  Serial.print(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

#define PC_BAUDRATE 115200
#define GPS_BAUDRATE 9600

//---------------------------------------- LoRa
// NSS, NRESET, and DIO0 pins can be changed by using LoRa.setPins(ss, reset, dio0).
// DIO0 pin is optional, it is only needed for receive callback mode.
// If DIO0 pin is used, it must be interrupt capable via attachInterrupt(...).
// LoRa   | Arduino
// VCC    | 3.3V
// GND    | GND
// SCK    | SCK (PD13)
// MISO   | MISO (PD12)
// MOSI   | MOSI (PD11)
// NSS    | PD10
// NRESET | PD9 -> PD4
// DIO0   | PD2 -> EMPTY (WE DON'T USE CALLBACK)
#define LORA_NSS 10   // new slave select pin to use, defaults to 10
#define LORA_NRESET 4 // 9 WATCH THIS WATCH THIS WATCH THIS WATCH THIS WATCH THIS
// #define LORA_NRESET 9 // new reset pin to use, defaults to 9
#define LORA_DIO0 2   // new DIO0 pin to use, defaults to 2. Must be interrupt capable via attachInterrupt(...).

//---------------------------------------- Message type
#define MSG_MOVEMENT   0x01
#define MSG_CAR_STATUS 0x02

//---------------------------------------- GPS
#define ENABLE_GPS false
#define FAKE_GPS true
#define GPS_RX 8 // connected to Rx pin of the GPS
#define GPS_TX 7 // connected to Tx pin of the GPS
#define DATETIME_FORMAT "%04d.%02d.%02d %02d:%02d:%02d.%03d"
#define DATETIME_LENGTH 24
char datetime[DATETIME_LENGTH];
TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
volatile unsigned long millisGPSBefore;
volatile double prevLat, prevLng;

#ifdef FAKE_GPS
const char *gpsStream =
  "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
  "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
  "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
  "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
  "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
  "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";
#endif

//---------------------------------------- SD Card pin
const int cskGpsSdCardPin = 10;
Sd2Card card;
bool enableSDCard = false;

//---------------------------------------- LoRa data transmission configuration.
int LoraSyncWord = 0x63;
int LocalAddress = 0x01;                   //--> address of this device (Master Address).
int LoRaController = 0x01;                 //--> destination to send to Controller (ESP32).
int LoRaRcCar = 0x02;                      //--> destination to send to RC Car.
volatile bool doRead = false;
volatile int incomingPacketSize;

//---------------------------------------- Variables for LoRa
typedef struct {
  int8_t x;
  int8_t y;
} JoystickAxis;

typedef struct {
  uint8_t sender;
  uint8_t recipient;
  uint8_t messageType;
  JoystickAxis position;
  long millis;
} CarMovement;

typedef struct {
  TinyGPSLocation location;
  TinyGPSDate date;
  TinyGPSTime time;
  TinyGPSSpeed speed;
  TinyGPSCourse course;
  TinyGPSAltitude altitude;
  TinyGPSInteger satellites;
  TinyGPSHDOP hdop;
} GpsData;

typedef struct {
  uint8_t sender;
  uint8_t recipient;
  uint8_t messageType;
  GpsData gps;
  long millis;
} CarStatus;

volatile bool newGpsData = false;
volatile CarStatus carStatus;

//---------------------------------------- Motor
// pcf8574   | Arduino
// VCC       | 5V
// GND       | GND
// SCL       | A5 (Analog 5)
// SDA       | A4 (Analog 4)
// Set i2c HEX address
PCF8574 pcf8574(0x20, A4, A5);

// Pin 2, 9 - 13 used for LoRa

const int mFR1 = P0;                       // Digital Pin Front Right
const int enFR = 3;                        // Enable Pin Front Right
const int mFR2 = P1;                       // Digital Pin Front Right

const int mFL1 = P2;                       // Digital Pin Front Left
const int enFL = 5;                        // Enable Pin Front Left
const int mFL2 = P3;                       // Digital Pin Front Left

const int mRR1 = P4;                       // Digital Pin Rear Right
const int enRR = 6;                        // Enable Pin Rear Right
const int mRR2 = P5;                       // Digital Pin Rear Right

const int mRL1 = P6;                       // Digital Pin Rear Left
const int enRL = 9;                        // Enable Pin Rear Left
const int mRL2 = P7;                       // Digital Pin Rear Left

#define TACHOMETER 2
volatile int holes;
const float totalHoles = 12.0;
float rpm = 0;
volatile unsigned long millisRPMBefore;

//---------------------------------------- Configure OLED screen size in pixels.
#define SCREEN_WIDTH 128                   //--> OLED display width, in pixels
#define SCREEN_HEIGHT 64                   //--> OLED display height, in pixels

//---------------------------------------- Declaration for an SSD1306 display connected to I2C (SDA, SCL pins).
#define OLED_RESET     -1                  // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define ENABLE_OLED true
bool enableOled = false;

File dataFile;

// Arduino Nano

void setup() {
  #ifdef DEBUG
  Serial.begin(PC_BAUDRATE);
  while (!Serial) ; // Wait for serial port to be available
  #endif

  // pwm.begin();
  // pwm.setOscillatorFrequency(27000000);
  // pwm.setPWMFreq(1600);  // This is the maximum PWM frequency
  // Wire.setClock(400000);


  if (ENABLE_OLED) {
    enableOled = display.begin(SSD1306_EXTERNALVCC, 0x3C);
    if(!enableOled) {
      DEBUG_PRINTLN("E: SSD1306 failed");
    }
    displayMessage("Oled initialized");
  }

  // SD Card
  if (enableSDCard) {
    pinMode(cskGpsSdCardPin, OUTPUT);
    if(card.init(SPI_HALF_SPEED, cskGpsSdCardPin)) {
      dataFile = SD.open("datalog.txt", FILE_WRITE);
      if (!dataFile)
        enableSDCard = false;
    } else {
      DEBUG_PRINTLN("E: Card failed");
    }
  }

  // LoRa
  LoRa.setPins(LORA_NSS, LORA_NRESET, LORA_DIO0);
  // LoRa.setSPIFrequency(8E6); // Some logic level converters cannot support high speeds such as 8 MHz, so a lower SPI frequency can be selected with LoRa.setSPIFrequency(frequency).
  // LoRa.setSPIFrequency(4E6);
  while (!LoRa.begin(433E6)) {  // initialize ratio at 433 MHz
    DEBUG_PRINTLN("E: LoRa failed");
    while (1) delay(1000);  // Halt at this point as is not possible to continue
  }
  
  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa.setSyncWord(LoraSyncWord);

  // Set motors' pins
  pcf8574.pinMode(mFL1, OUTPUT);
  pcf8574.pinMode(mFL2, OUTPUT);
  pcf8574.digitalWrite(mFL1, LOW);
  pcf8574.digitalWrite(mFL2, LOW);

  pcf8574.pinMode(mFR1, OUTPUT);
  pcf8574.pinMode(mFR2, OUTPUT);
  pcf8574.digitalWrite(mFR1, LOW);
  pcf8574.digitalWrite(mFR2, LOW);

  pcf8574.pinMode(mRL1, OUTPUT);
  pcf8574.pinMode(mRL2, OUTPUT);
  pcf8574.digitalWrite(mRL1, LOW);
  pcf8574.digitalWrite(mRL2, LOW);

  pcf8574.pinMode(mRR1, OUTPUT);
  pcf8574.pinMode(mRR2, OUTPUT);
  pcf8574.digitalWrite(mRR1, LOW);
  pcf8574.digitalWrite(mRR2, LOW);

  analogWrite(enFR, 0);
  analogWrite(enFL, 0);
  analogWrite(enRR, 0);
  analogWrite(enRL, 0);

  while (!pcf8574.begin()) {
    DEBUG_PRINTLN("E: pcf8574 failed");
    while(1) delay(1000);  // Halt at this point as is not possible to continue
  }

  // GPS
  // baud rate for GPS - 38400 is prefered, but 4800 can also be used
  if (ENABLE_GPS) {
    gpsSerial.begin(GPS_BAUDRATE);
  }

  // LoRa Rx mode
  LoRa.enableInvertIQ();  // active invert I and Q signals
  LoRa.receive();         // set receive mode
}


void countHoles() {
  holes++;
}

void loop() {
  long currentMillis = millis();
  if (currentMillis - millisRPMBefore > 1000) {
    rpm = (holes / totalHoles) * 60;
    holes = 0;
    millisRPMBefore = currentMillis;
  }

  // GPS
  if (ENABLE_GPS) {
    #ifdef FAKE_GPS
    while (*gpsStream)
    if (gps.encode(*gpsStream++))
      newGpsData = true;
    #else
    while (gpsSerial.available() > 0 && currentMillis - millisGPSBefore > 1000) {
      millisGPSBefore = currentMillis;
      if (gps.encode(gpsSerial.read())) {
        if (gps.location.isValid() && prevLat != gps.location.lat() && prevLng != gps.location.lng()) {
          newGpsData = true;
          prevLat = gps.location.lat();
          prevLng = gps.location.lng();
        }
      }
    }
    #endif
  }

  if (doRead) {
    readMessage();
    doRead = false; // Set flag back to false so next read will happen only after next ISR event
  } else if (newGpsData) { // always ENABLE_GPS
    DEBUG_PRINTLN();
    DEBUG_PRINT("NEWGPSDATA\n");
    if (setGpsInfo()) {
      setLoRaToTxMode();
      sendStatus(&carStatus);
      setLoRaToRxMode();
    }
  }
}

bool setGpsInfo() {
  String data;

  if (!gps.location.isValid())
    return false;

  DEBUG_PRINTLN("setGpsInfo");

  carStatus.gps.date = gps.date;
  carStatus.gps.time = gps.time;
  carStatus.gps.speed = gps.speed;
  carStatus.gps.course = gps.course;
  carStatus.gps.location = gps.location;
  carStatus.gps.altitude = gps.altitude;
  #ifndef FAKE_GPS
  carStatus.gps.satellites = gps.satellites;
  carStatus.gps.hdop = gps.hdop;
  #endif

  carStatus.gps.location.lat(); // hack blocked code
  carStatus.gps.location.lng(); // hack blocked code

  data += String(carStatus.gps.location.lat(), 6);
  data += ",";
  data += String(carStatus.gps.location.lng(), 6);

  if (carStatus.gps.date.isValid() && carStatus.gps.time.isValid()) {
    carStatus.gps.date.year(); // hack blocked code
    carStatus.gps.time.hour(); // hack blocked code
    snprintf(datetime, DATETIME_LENGTH, DATETIME_FORMAT, carStatus.gps.date.year(), carStatus.gps.date.month(), carStatus.gps.date.day(),
      carStatus.gps.time.hour(), carStatus.gps.time.minute(), carStatus.gps.time.second(), carStatus.gps.time.centisecond());
    data += ",";
    data += datetime;
  }

  DEBUG_PRINTLN(data);
  newGpsData = false;

  if (dataFile) {
    dataFile.println(data);
    dataFile.close();
  }

  return true;
}

void displayMessage(String message) {
  if (!enableOled)
    return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0); //--> (x position, y position)
  display.print(message.substring(0, 20));
  display.display();
}

void setLoRaToTxMode() {
  // LoRa Tx mode
  LoRa.onReceive(NULL);
  LoRa.onTxDone(onTxDone);
  LoRa.idle();             // set standby mode
  LoRa.disableInvertIQ();  // normal mode
}

void setLoRaToRxMode() {
  // LoRa Rx mode
  LoRa.onReceive(onReceive);
  LoRa.onTxDone(NULL);
  LoRa.enableInvertIQ();  // active invert I and Q signals
  LoRa.receive();
}

void onTxDone() {
  DEBUG_PRINTLN();
  DEBUG_PRINT("RC Car onTxDone");

  // LoRa Rx mode
  setLoRaToRxMode();
  if (LoRa.peek() > 0) {
    DEBUG_PRINTLN();
    DEBUG_PRINT("onTxDone LoRa.peek() > 0");
  } else {
    setLoRaToTxMode();
  }
}

void onReceive(int packetSize) {
  DEBUG_PRINTLN("\nonReceive");
  doRead = (packetSize > 0);
  incomingPacketSize = packetSize;

  setLoRaToTxMode();
  sendStatus(&carStatus);
  setLoRaToRxMode();
}

void readMessage() {
  CarMovement movement;
  LoRa.readBytes((uint8_t *)&movement, incomingPacketSize);
  DEBUG_PRINTLN();
  DEBUG_PRINTLN(String(movement.position.x) + "," + String(movement.position.y));
  processIncomingData(&movement);
}

void processIncomingData(CarMovement *data) {
  DEBUG_PRINTLN();
  DEBUG_PRINT("processIncomingData");
  CarMovement movement = *data;
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("sender: " + String(movement.sender));
  DEBUG_PRINTLN("recipient: " + String(movement.recipient));
  DEBUG_PRINTLN("messageType: " + String(movement.messageType));

  //---------------------------------------- accept message only from controller.
  if (movement.sender != LoRaController) {
    DEBUG_PRINTLN();
    DEBUG_PRINT("E: NC"); // Not from our controller
    return;
  }

  //---------------------------------------- checks whether the incoming data or message for this device.
  if (movement.recipient != LoRaRcCar) {
    DEBUG_PRINTLN();
    DEBUG_PRINT("E: recipient to ");
    DEBUG_PRINT(String(movement.recipient));
    return;
  }

  //---------------------------------------- checks whether the message is MSG_CAR_STATUS.
  if (movement.messageType != MSG_MOVEMENT) {
    DEBUG_PRINTLN();
    DEBUG_PRINT("E: NCM"); // Not correct message
    return;
  }

  //---------------------------------------- TODO: check data integrity.

  //---------------------------------------- Get all incoming data.
  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  //---------------------------------------- check length for error.
  if (incomingLength != incoming.length()) {
    DEBUG_PRINTLN();
    DEBUG_PRINT("E: corrupt message");     //--> "er" = error: message length does not match length.
    return;
  }

  DEBUG_PRINTLN();
  DEBUG_PRINT("moveMotors: " + String(movement.position.x) + "," + String(movement.position.y));
  moveMotors(&movement.position);
}

void moveMotors(JoystickAxis *axis) {
  JoystickAxis position = *axis;
  if (position.x == 0 && position.y == 0) {
    pcf8574.digitalWrite(mFR1, LOW);
    pcf8574.digitalWrite(mFR2, LOW);

    pcf8574.digitalWrite(mFL1, LOW);
    pcf8574.digitalWrite(mFL2, LOW);

    pcf8574.digitalWrite(mRR1, LOW);
    pcf8574.digitalWrite(mRR2, LOW);

    pcf8574.digitalWrite(mRL1, LOW);
    pcf8574.digitalWrite(mRL2, LOW);
    return;
  }

  int rightMotorsSpeed = 0;
  int leftMotorsSpeed = 0;

  if (position.x < 0) { // turn left, right motor should faster than left motor
    int adj = 255 + position.x; // 255 + -x
    rightMotorsSpeed = max(position.y + adj, 255);
    leftMotorsSpeed = min(position.y - adj, 0);
  } else if (position.x > 0) { // turn right, left motor should faster than right motor
    int adj = 255 - position.x; // 255 + -x
    leftMotorsSpeed = max(position.y + adj, 255);
    rightMotorsSpeed = min(position.y - adj, 0);
  }

  if (position.y > 0) {
    pcf8574.digitalWrite(mFR1, LOW);
    pcf8574.digitalWrite(mFR2, HIGH);
    analogWrite(enFR, rightMotorsSpeed);

    pcf8574.digitalWrite(mFL1, LOW);
    pcf8574.digitalWrite(mFL2, HIGH);
    analogWrite(enFL, leftMotorsSpeed);

    pcf8574.digitalWrite(mRR1, LOW);
    pcf8574.digitalWrite(mRR2, HIGH);
    analogWrite(enRR, 255);

    pcf8574.digitalWrite(mRL1, LOW);
    pcf8574.digitalWrite(mRL2, HIGH);
    analogWrite(enRL, 255);
  } else {
    pcf8574.digitalWrite(mFR1, HIGH);
    pcf8574.digitalWrite(mFR2, LOW);
    analogWrite(enFR, 255);

    pcf8574.digitalWrite(mFL1, HIGH);
    pcf8574.digitalWrite(mFL2, LOW);
    analogWrite(enFL, 255);

    pcf8574.digitalWrite(mRR1, HIGH);
    pcf8574.digitalWrite(mRR2, LOW);
    analogWrite(enRR, rightMotorsSpeed);

    pcf8574.digitalWrite(mRL1, HIGH);
    pcf8574.digitalWrite(mRL2, LOW);
    analogWrite(enRL, leftMotorsSpeed);
  }
}

void sendStatus(CarStatus *packet) {
  DEBUG_PRINTLN("sendStatus\n");
  CarStatus status = *packet;
  status.sender = LoRaRcCar;
  status.recipient = LoRaController;
  status.messageType = MSG_CAR_STATUS;
  status.millis = millis();
  LoRa.beginPacket();                 //--> start packet
  LoRa.write((uint8_t*)&status, sizeof(CarStatus));
  LoRa.endPacket();                   //--> finish packet and send it
  DEBUG_PRINTLN("finished send message");
}

void flushSerialBuffer() {
  while (Serial.available()) {
    Serial.read();
  }
}
