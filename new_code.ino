#include <ESP8266WiFi.h>
#include <Servo.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>

const char* ssid = "realme narzo 30 5G";  // Enter your WiFi Name
const char* pass = "Its Sk CooL";        // Enter your WiFi Password

#define MQTT_SERV "io.adafruit.com"
#define MQTT_PORT 1883
#define MQTT_NAME "swas_4_kul"
#define MQTT_PASS "aio_Peju97zNUkM0umlSgrotANiK0OaC"

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800, 60000);
Servo myservo;         // Servo for entry gate
Servo myservos;        // Servo for exit gate

const int carEnter = D0;   // Entry sensor
const int carExited = D2;  // Exit sensor
const int slot3 = D7;
const int slot2 = D6;
const int slot1 = D3;

int count = 0;
int CLOSE_ANGLE = 150;  // The closing angle of the servo motor arm
int OPEN_ANGLE = 0;    // The opening angle of the servo motor arm
int hh, mm, ss;
String h, m, EntryTimeSlot1, ExitTimeSlot1, EntryTimeSlot2, ExitTimeSlot2, EntryTimeSlot3, ExitTimeSlot3;
boolean entrysensor, exitsensor, s1, s2, s3;
boolean s1_occupied = false;
boolean s2_occupied = false;
boolean s3_occupied = false;

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERV, MQTT_PORT, MQTT_NAME, MQTT_PASS);

// Set up the feed you're subscribing to
Adafruit_MQTT_Subscribe EntryGate = Adafruit_MQTT_Subscribe(&mqtt, MQTT_NAME "/f/EntryGate");
Adafruit_MQTT_Subscribe ExitGate = Adafruit_MQTT_Subscribe(&mqtt, MQTT_NAME "/f/ExitGate");

// Set up the feed you're publishing to
Adafruit_MQTT_Publish CarsParked = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/f/CarsParked");
Adafruit_MQTT_Publish EntrySlot1 = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/f/EntrySlot1");
Adafruit_MQTT_Publish ExitSlot1 = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/f/ExitSlot1");
Adafruit_MQTT_Publish EntrySlot2 = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/f/EntrySlot2");
Adafruit_MQTT_Publish ExitSlot2 = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/f/ExitSlot2");
Adafruit_MQTT_Publish EntrySlot3 = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/f/EntrySlot3");
Adafruit_MQTT_Publish ExitSlot3 = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/f/ExitSlot3");

void setup() {
  delay(1000);
  Serial.begin(9600);
  mqtt.subscribe(&EntryGate);
  mqtt.subscribe(&ExitGate);
  timeClient.begin();
  myservo.attach(D4);      // Servo pin to D4
  myservos.attach(D5);     // Servo pin to D5
  pinMode(carExited, INPUT);
  pinMode(carEnter, INPUT);
  pinMode(slot1, INPUT);
  pinMode(slot2, INPUT);
  pinMode(slot3, INPUT);

  WiFi.begin(ssid, pass);
  Serial.print("Connecting to ");
  Serial.println(ssid);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println();
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  MQTT_connect();
  timeClient.update();
  hh = timeClient.getHours();
  mm = timeClient.getMinutes();
  ss = timeClient.getSeconds();
  h = String(hh);
  m = String(mm);
  h = h + ":" + m;

  entrysensor = !digitalRead(carEnter);
  exitsensor = !digitalRead(carExited);
  s1 = digitalRead(slot1);
  s2 = digitalRead(slot2);
  s3 = digitalRead(slot3);

  if (entrysensor == 1) {
    count = count + 1;
    myservos.write(OPEN_ANGLE);
    delay(3000);
    myservos.write(CLOSE_ANGLE);
  }

  if (exitsensor == 1) {
    count = count - 1;
    myservo.write(OPEN_ANGLE);
    delay(3000);
    myservo.write(CLOSE_ANGLE);
  }

  if (!CarsParked.publish(count)) {}

  if (s1 == 1 && !s1_occupied) {
    Serial.println("Availabile1");
    EntryTimeSlot1 = h;
    s1_occupied = true;
    if (!EntrySlot1.publish((char*)EntryTimeSlot1.c_str())) {}
  }

  if (s1 == 0 && s1_occupied) {
    Serial.println("Occupied1");
    ExitTimeSlot1 = h;
    s1_occupied = false;
    if (!ExitSlot1.publish((char*)ExitTimeSlot1.c_str())) {}
  }

  if (s2 == 1 && !s2_occupied) {
    Serial.println("Available2");
    EntryTimeSlot2 = h;
    s2_occupied = true;
    if (!EntrySlot2.publish((char*)EntryTimeSlot2.c_str())) {}
  }

  if (s2 == 0 && s2_occupied) {
    Serial.println("Occupied2");
    ExitTimeSlot2 = h;
    s2_occupied = false;
    if (!ExitSlot2.publish((char*)ExitTimeSlot2.c_str())) {}
  }

  if (s3 == 1 && !s3_occupied) {
    Serial.println("Available3");
    EntryTimeSlot3 = h;
    s3_occupied = true;
    if (!EntrySlot3.publish((char*)EntryTimeSlot3.c_str())) {}
  }

  if (s3 == 0 && s3_occupied) {
    Serial.println("Occupied3");
    ExitTimeSlot3 = h;
    s3_occupied = false;
    if (!ExitSlot3.publish((char*)ExitTimeSlot3.c_str())) {}
  }

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(500))) {
    if (subscription == &EntryGate) {
      int value = atoi((char*)EntryGate.lastread);
      if (value == 1) {
        myservos.write(OPEN_ANGLE);
        delay(3000);
        myservos.write(CLOSE_ANGLE);
      }
    }
    if (subscription == &ExitGate) {
      int value = atoi((char*)ExitGate.lastread);
      if (value == 1) {
        myservo.write(OPEN_ANGLE);
        delay(3000);
        myservo.write(CLOSE_ANGLE);
      }
    }
  }
}

void MQTT_connect() {
  int8_t ret;

  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);
    retries--;
    if (retries == 0) {
      while (1);
    }
  }

  Serial.println("MQTT Connected!");
}
