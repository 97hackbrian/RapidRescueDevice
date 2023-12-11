#include "Adafruit_FONA.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define SIM800L_RX     27
#define SIM800L_TX     26
#define SIM800L_PWRKEY 4
#define SIM800L_RST    5
#define SIM800L_POWER  23

char replybuffer[255];

HardwareSerial *sim800lSerial = &Serial1;
Adafruit_FONA sim800l = Adafruit_FONA(SIM800L_PWRKEY);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

#define LED_BLUE  13
#define RELAY 14

String smsString = "";

// MPU6050
Adafruit_MPU6050 mpu;

void setup()
{
  pinMode(LED_BLUE, OUTPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(SIM800L_POWER, OUTPUT);

  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(SIM800L_POWER, HIGH);

  Serial.begin(115200);
  Serial.println(F("ESP32 with GSM SIM800L and MPU6050"));
  Serial.println(F("Initializing....(May take more than 10 seconds)"));
  
  delay(10000);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println(F("Failed to find MPU6050"));
    while (1);
  }
  Serial.println(F("MPU6050 is OK"));

  // Make it slow so it's easy to read!
  sim800lSerial->begin(4800, SERIAL_8N1, SIM800L_TX, SIM800L_RX);
  if (!sim800l.begin(*sim800lSerial)) {
    Serial.println(F("Couldn't find GSM SIM800L"));
    while (1);
  }
  Serial.println(F("GSM SIM800L is OK"));

  char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = sim800l.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("SIM card IMEI: "); Serial.println(imei);
  }

  // Set up the FONA to send a +CMTI notification
  // when an SMS is received
  sim800lSerial->print("AT+CNMI=2,1\r\n");

  Serial.println("GSM SIM800L and MPU6050 Ready");
}

long prevMillis = 0;
int interval = 1000;
char sim800lNotificationBuffer[64];          //for notifications from the FONA
char smsBuffer[250];
boolean ledState = false;
float valX, valY = 0;
float lastValX, lastValY = 0;
float threshold = 3.8*9.81; // Adjust the threshold as needed

void loop()
{
  // Read accelerometer data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  valX = a.acceleration.x;
  valY = a.acceleration.y;

  // Calculate derivatives
  float derivativeX = valX - lastValX;
  float derivativeY = valY - lastValY;

  Serial.print(F("Derivative X: "));
  Serial.print(derivativeX);
  Serial.print(F(", Y: "));
  Serial.println(derivativeY);
  String smsMessage = "Accidente detectado, MENSAJE DE ALERTA!\n";
  smsMessage += "Con un cambio de aceleración en X de: " + String(derivativeX, 2) + "\n";
  smsMessage += "Con un cambio de aceleración en Y de: " + String(derivativeY, 2);
  char smsBuffer[250];
  smsMessage.toCharArray(smsBuffer, sizeof(smsBuffer));
  // Check for collision
  if (abs(derivativeX) >= threshold || abs(derivativeY) >= threshold) {
    char callerIDbuffer[32];  //we'll store the SMS sender number in here
    strcpy(callerIDbuffer, "+59178627162");
    Serial.println("Por enviar....");
    delay(2000);
    if (!sim800l.sendSMS(callerIDbuffer, smsBuffer)) {
      Serial.println(F("Failed"));
    } else {
      Serial.println(F("Sent!"));
    }
  }

  lastValX = valX;
  lastValY = valY;

  if (millis() - prevMillis > interval) {
    ledState = !ledState;
    digitalWrite(LED_BLUE, ledState);
    prevMillis = millis();
  }
  
  char* bufPtr = sim800lNotificationBuffer;    //handy buffer pointer

  if (sim800l.available()) {
    int slot = 0; // this will be the slot number of the SMS
    int charCount = 0;

    // Read the notification into fonaInBuffer
    do {
      *bufPtr = sim800l.read();
      Serial.write(*bufPtr);
      delay(1);
    } while ((*bufPtr++ != '\n') && (sim800l.available()) && (++charCount < (sizeof(sim800lNotificationBuffer)-1)));
    
    // Add a terminal NULL to the notification string
    *bufPtr = 0;
  }
}
