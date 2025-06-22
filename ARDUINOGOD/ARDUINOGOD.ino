#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <time.h>

// WiFi Configuration
const char* ssid = "DIMASARGYA";
const char* password = "12345678";

// Server Configuration
const char* serverURL = "https://kucing-tubes.onrender.com/";

// Hardware Pin Configuration
#define TRIG_PIN 5
#define ECHO_PIN 18
#define SERVO_PIN 4
#define BUTTON_PIN 2
#define LED_PIN 12

// LCD Configuration (I2C)
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Address 0x27, 16x2 LCD

// Servo Configuration
Servo servoMotor;

// Global Variables
float ultrasonicDistance = 0.0;
bool servoActive = false;
bool buttonPressed = false;
bool lastCommandWasFeed = false;

unsigned long lastSensorRead = 0;
unsigned long lastServerUpdate = 0;
unsigned long servoStartTime = 0;
unsigned long lastButtonCheck = 0;
int feedingCount = 0;

// Timing constants
const unsigned long SENSOR_INTERVAL = 2000;    // 2 seconds
const unsigned long SERVER_INTERVAL = 5000;   // 5 seconds
const unsigned long SERVO_DURATION = 3000;     // 3 seconds
const unsigned long BUTTON_DEBOUNCE = 200;     // 200ms

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("CAT FEEDER");
  lcd.setCursor(0, 1);
  lcd.print("STARTING...");

  servoMotor.attach(SERVO_PIN);
  servoMotor.write(0);  // Closed position

  connectToWiFi();
  configTime(7 * 3600, 0, "pool.ntp.org");  // GMT+7

  updateLCDDisplay("CAT FEEDER READY", "WIFI CONNECTED");
  delay(2000);
}

void loop() {
  unsigned long currentTime = millis();

  checkButton();

  if (currentTime - lastSensorRead >= SENSOR_INTERVAL) {
    ultrasonicDistance = readUltrasonicSensor();
    lastSensorRead = currentTime;

    if (!servoActive) {
      String line1 = "CAT FEEDER READY";
      String line2 = "DIST: " + String(ultrasonicDistance, 1) + " CM";
      updateLCDDisplay(line1, line2);
    }
  }

  if (currentTime - lastServerUpdate >= SERVER_INTERVAL) {
    sendSensorDataToServer();
    checkServerCommands();
    lastServerUpdate = currentTime;
  }

  if (servoActive && (currentTime - servoStartTime >= SERVO_DURATION)) {
    stopServo();
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, reconnecting...");
    delay(5000);  // prevent reconnect spam
    connectToWiFi();
  }

  delay(100);
}

void connectToWiFi() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CONNECTING WIFI");
  lcd.setCursor(0, 1);
  lcd.print("PLEASE WAIT...");

  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(1000);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected successfully!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WIFI CONNECTED");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP().toString());
    delay(2000);
  } else {
    Serial.println("Failed to connect to WiFi");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WIFI ERROR");
    lcd.setCursor(0, 1);
    lcd.print("CHECK SETTINGS");
  }
}

float readUltrasonicSensor() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) {
    Serial.println("Ultrasonic sensor timeout");
    return ultrasonicDistance;
  }

  float distance = (duration * 0.034) / 2;

  if (distance < 2 || distance > 400) {
    return ultrasonicDistance;
  }

  Serial.print("Ultrasonic distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  return distance;
}

void checkButton() {
  unsigned long currentTime = millis();

  if (currentTime - lastButtonCheck >= BUTTON_DEBOUNCE) {
    if (digitalRead(BUTTON_PIN) == LOW && !buttonPressed) {
      buttonPressed = true;
      lastButtonCheck = currentTime;
      Serial.println("Button pressed - Starting feeding");
      startFeeding();
    } else if (digitalRead(BUTTON_PIN) == HIGH && buttonPressed) {
      buttonPressed = false;
      lastButtonCheck = currentTime;
    }
  }
}

void startFeeding() {
  if (servoActive) {
    Serial.println("Servo already active, ignoring feed command");
    return;
  }

  Serial.println("Starting feeding sequence...");

  servoActive = true;
  servoStartTime = millis();
  feedingCount++;

  servoMotor.write(45);
  updateLCDDisplay("FEEDING...", "SERVO ACTIVE");
  digitalWrite(LED_PIN, HIGH);

  sendFeedingToServer();
}

void stopServo() {
  Serial.println("Stopping servo...");

  servoActive = false;
  servoMotor.write(0);
  digitalWrite(LED_PIN, LOW);
  updateLCDDisplay("FEEDING COMPLETE", "SERVO STANDBY");

  delay(2000);
}

void updateLCDDisplay(String line1, String line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1.substring(0, 16));
  lcd.setCursor(0, 1);
  lcd.print(line2.substring(0, 16));
}

void sendSensorDataToServer() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skipping server update");
    return;
  }

  HTTPClient http;
  http.begin(String(serverURL) + "/esp32/sensor");
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<200> doc;
  doc["ultrasonic"] = ultrasonicDistance;
  doc["servo_active"] = servoActive;
  doc["feeding_count"] = feedingCount;
  doc["timestamp"] = getTimestamp();

  String jsonString;
  serializeJson(doc, jsonString);

  int httpResponseCode = http.POST(jsonString);

  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("Sensor data sent successfully");
    Serial.println("Response: " + response);
  } else {
    Serial.print("Error sending sensor data: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}

void checkServerCommands() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(String(serverURL) + "/esp32/servo");

  int httpResponseCode = http.GET();

  if (httpResponseCode > 0) {
    String payload = http.getString();
    StaticJsonDocument<200> doc;
    deserializeJson(doc, payload);

    String command = doc["command"];

    if (command == "feed" && !lastCommandWasFeed) {
      Serial.println("Received feed command from server");
      startFeeding();
      lastCommandWasFeed = true;
    } else if (command != "feed") {
      lastCommandWasFeed = false;
    }
  } else {
    Serial.print("Error checking server commands: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}

void sendFeedingToServer() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(String(serverURL) + "/control_servo");
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<200> doc;
  doc["action"] = "feed";
  doc["source"] = "esp32_button";
  doc["distance_before"] = ultrasonicDistance;
  doc["timestamp"] = getTimestamp();

  String jsonString;
  serializeJson(doc, jsonString);

  int httpResponseCode = http.POST(jsonString);

  if (httpResponseCode > 0) {
    Serial.println("Feeding action sent to server");
  } else {
    Serial.print("Error sending feeding action: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}

String getTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "Unknown";
  }

  char timeString[50];
  strftime(timeString, sizeof(timeString), "%d %B %Y, %H:%M:%S", &timeinfo);

  return String(timeString);
}
