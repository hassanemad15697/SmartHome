// Template ID, Device Name and Auth Token 
#define BLYNK_TEMPLATE_ID "TMPLUVOePQMr"
#define BLYNK_DEVICE_NAME "Home"
#define BLYNK_AUTH_TOKEN "Rzn05kyJzc09gaYawXzP8Z8jRGaJRYSU"
#define BLYNK_PRINT Serial
#define DHTPIN 12 //DHT pin
#define DHTTYPE DHT11 //DTH Type

//used libraries
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <DHT.h>
#include <Servo.h>

struct Sensor {
  String time[200] = {};
  int value[200] = {};
};

//Blynk Authontication Token
char auth[] = BLYNK_AUTH_TOKEN;
// Replace with your network credentials
const char * ssid[] = {
  "Asmaa",
  "rahma",
  "Soma"
};
const char * password[] = {
  "Samka1499",
  "HWSNSZeW",
  "azx1499@#"
};
int arrSize = sizeof(ssid) / sizeof(ssid[0]);

// Events for Connecting and Disconnecting modes
WiFiEventHandler wifiConnectHandler;
struct Sensor Temperature;
struct Sensor Humidity;
int networkNumber = 0;
int triesNumToConnect = 0;
bool isConnected = false;
int DHT_Buffer_Counter=0;
// setup sensors pins
const int motionSensor = D5; // PIR Motion Sensor Pin
bool motionDetected = false; // PIR Motion Sensor state
bool motionControl = false; // PIR Motion Sensor control
const int pin = D4; //built_in Led pin
const int rainAndSomkePin = A0; //rain pin
const int rainControl = D7;
const int smokeControl = D8;
const int flame = D0; //flame pin
const int lightingRelay = D1; //lightingRelay pin
int smokeThreshold = 200;

BlynkTimer timer; //Object of Blynk Timer
BlynkTimer motionTimer; // motion timer
DHT dht(DHTPIN, DHTTYPE); //Object of DHT sensor
Servo doorServo;
Servo windowServo;
Servo garageServo;

// method for initializing the WI-Fi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid[networkNumber], password[networkNumber]);
  Serial.print("Connecting to ");
  Serial.print(ssid[networkNumber]);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

// method to handle connected case

void onWifiConnect(const WiFiEventStationModeGotIP & event) {
  Serial.println("Connected to Wi-Fi sucessfully.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  isConnected=true;
  triesNumToConnect = 0;
}

//control of Built-In Led
BLYNK_WRITE(V0) {
  // Set incoming value from pin V0 to a variable
  int value = param.asInt();
  if (value == 1) {
    digitalWrite(pin, LOW);
  } else if (value == 0) {
    digitalWrite(pin, HIGH);
  }
}
//control of Relay
BLYNK_WRITE(V7) {
  // Set incoming value from pin V7 to a variable
  int relayValue = param.asInt();
  if (relayValue == 1) {
    digitalWrite(lightingRelay, HIGH);
    Serial.println("lightingRelay is ON");
  } else if (relayValue == 0) {
    digitalWrite(lightingRelay, LOW);
    Serial.println("lightingRelay is Off");
  }
}
//control of door motor
BLYNK_WRITE(V8) {
  int motorValue = param.asInt();
  int flameValue = digitalRead(flame);
  if (flameValue == 0) {
    if (motorValue == 0) {
      doorServo.write(0);
      motionControl = true;
    } else if (motorValue == 1) {
      doorServo.write(180);
      motionControl = false;
      //motionTimer.enable(control_motion_timer_ID);
    }
  }
}
BLYNK_WRITE(V9) {
  // Set incoming value from pin V1 to a variable
  int motionValue = param.asInt();
  if (motionValue == 1) {
    motionControl = true;
  } else if (motionValue == 0) {
    motionControl = false;
  }
}
//control of window motor
BLYNK_WRITE(V10) {
  int degree = param.asInt();
  digitalWrite(smokeControl, LOW);
  delay(100);
  digitalWrite(rainControl, HIGH);
  delay(100);
  int rainValue = analogRead(rainAndSomkePin);
  digitalWrite(smokeControl, HIGH);
  delay(100);
  digitalWrite(rainControl, LOW);
  delay(100);
  int smokeValue = analogRead(rainAndSomkePin);
  int flameValue = digitalRead(flame);
  if (rainValue > 500 && smokeValue < smokeThreshold && flameValue == 0) {
    windowServo.write(degree);
  }
}
//control of garage Servo motor
BLYNK_WRITE(V11) {
  int motorValue = param.asInt();
  int flameValue = digitalRead(flame);
  if (flameValue == 0) {
    if (motorValue == 0) {
      garageServo.write(0);
      motionControl = true;
    } else if (motorValue == 1) {
      garageServo.write(120);
      motionControl = false;
    }
  }
}
//determing the value of smoke sensor
BLYNK_WRITE(V12) {
  smokeThreshold = param.asInt();

}
//getting readings of DHT Sensor
void sendDHTSensorReadings() {
  float h = dht.readHumidity();
  float t = dht.readTemperature(); // or dht.readTemperature(true) for Fahrenheit
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  if (isConnected == true) {
    //if(){
    Serial.println("in sending mode (debugging)");
    showingSensorBufferData(Temperature);
    showingSensorBufferData(Humidity);
    //}
    Blynk.virtualWrite(V2, h);
    Blynk.virtualWrite(V3, t);
  } else {
    Serial.println("in buffering mode (debugging)");
    Temperature = insertIntoSensorBuffer(Temperature, "11:40", t);
    Humidity = insertIntoSensorBuffer(Humidity, "11:40", h);
  }
}
//Getting the status of the buttons and sensors
void updateSystemStatus() {
  if (doorServo.read() == 0) {
    Blynk.virtualWrite(V8, 0);
  } else if (doorServo.read() == 180) {
    Blynk.virtualWrite(V8, 1);
  }
  Blynk.virtualWrite(V10, windowServo.read());
  if (garageServo.read() == 0) {
    Blynk.virtualWrite(V11, 0);
  } else if (garageServo.read() == 120) {
    Blynk.virtualWrite(V11, 1);
  }

  if (motionControl == true) {
    Blynk.virtualWrite(V9, 1);
  } else if (motionControl == false) {
    Blynk.virtualWrite(V9, 0);
  }

  if (digitalRead(lightingRelay) == HIGH) {
    Blynk.virtualWrite(V7, 1);
  } else if (digitalRead(lightingRelay) == LOW) {
    Blynk.virtualWrite(V7, 0);
  }
}
// Indicates when motion is detected
void ICACHE_RAM_ATTR detectsMovement() {
  Serial.println("MOTION DETECTED!!!");
  motionDetected = true;
}

//setup function to set pinMode for each sensor
//and initialize buarRate , motion Interrupt , DHT sensor and Blynk timer
void setup() {
  // Debug console
  Serial.begin(9600);
  // Built-In Led mode OUTPUT
  pinMode(pin, OUTPUT);
  // lighting Relay mode OUTPUT
  pinMode(lightingRelay, OUTPUT);
  // smoke Control mode OUTPUT
  pinMode(smokeControl, OUTPUT);
  // rain Control mode OUTPUT
  pinMode(rainControl, OUTPUT);
  // Rain Sensor And Somke mode INPUT
  pinMode(rainAndSomkePin, INPUT);
  // Flame Sensor mode INPUT
  pinMode(flame, INPUT);
  //Register event handlers
  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  initWiFi();
  Blynk.config(auth);
  while (Blynk.connect() == false) {
    // Wait until connected
  }
  //Start Operation Of DHT Sensor for Humidity and Temperature
  dht.begin();
  //Updating the readings of DHT sensor every second
  timer.setInterval(1000L, sendDHTSensorReadings);
  //Updating the status of the buttons and sensors
  timer.setInterval(1000L, updateSystemStatus);
  // Motors Pins
  doorServo.attach(2); //D4
  windowServo.attach(0); //D3
  garageServo.attach(4); //D2
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);
}

// insert  Into Sensor Buffer
struct Sensor insertIntoSensorBuffer(struct Sensor sensor, String time, int value) {
  sensor.time[DHT_Buffer_Counter] = time;
  sensor.value[DHT_Buffer_Counter] = value;
  DHT_Buffer_Counter++;
  return sensor;
}
// showing Sensor Buffer Data
void showingSensorBufferData(struct Sensor sensor) {
  for(int i=0; i< DHT_Buffer_Counter;i++){
  Serial.print("time = ");
  Serial.print(sensor.time[i]);
  Serial.print(", value = ");
  Serial.println(sensor.value[i]);
  }
}

unsigned long previousMillis = 0;
unsigned long interval = 3000;
//loop fuction that infinitly repeats itself
void loop() {
  unsigned long currentMillis = millis();
  // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >= interval)) {
    Serial.println("");
    Serial.println("Failed to connect to Wi-Fi, trying to reconnect...");
    isConnected = false;
    WiFi.disconnect();
    triesNumToConnect++;
    if (triesNumToConnect < 4) {
      Serial.print("try number : ");
      Serial.println(triesNumToConnect);
      WiFi.begin(ssid[networkNumber], password[networkNumber]);
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("Try to connect anthor network ...");

      networkNumber++;

      if (networkNumber >= arrSize) {
        networkNumber = 0;
      }

      triesNumToConnect = 0;

      Serial.print("try Connecting to ");
      Serial.println(ssid[networkNumber]);
      WiFi.begin(ssid[networkNumber], password[networkNumber]);
      Serial.println(WiFi.localIP());
      previousMillis = currentMillis;
    }
  }
  /////////////////////////////////////////////////////
  //Start Blynk Server and its timer
  Blynk.run();
  timer.run();
  /////////////////////////////////////////////////////
  //Action If Motion Detected
  if (motionDetected && motionControl == true) {
    // Update state
    Blynk.virtualWrite(V1, 1);
    motionDetected = false;
    Blynk.virtualWrite(V1, 0);
  }
  /////////////////////////////////////////////////////
  //If Smoke Reading will greater than 600 the App will Push a warning notification 
  digitalWrite(smokeControl, HIGH);
  delay(100);
  int smokeValue = analogRead(rainAndSomkePin);
  Blynk.virtualWrite(V6, smokeValue);
  if (smokeValue > smokeThreshold) {
    windowServo.write(180);
  }
  digitalWrite(smokeControl, LOW);
  /////////////////////////////////////////////////////
  //If Rain Reading will below 500 the App will Push a warning notification
  digitalWrite(rainControl, HIGH);
  delay(100);
  int rainValue = analogRead(rainAndSomkePin);
  Blynk.virtualWrite(V5, rainValue);
  if (rainValue < 500 && smokeValue < smokeThreshold) {
    windowServo.write(0);
  }
  digitalWrite(rainControl, LOW);
  /////////////////////////////////////////////////////
  //If Flame Reading will be 1 the App will Push a warning notification 
  int flameValue = digitalRead(flame);
  if (flameValue == 1) {
    windowServo.write(180);
    doorServo.write(180);
    garageServo.write(120);
    motionControl = false;
  }
  Blynk.virtualWrite(V4, flameValue);
  /////////////////////////////////////////////////////
}
