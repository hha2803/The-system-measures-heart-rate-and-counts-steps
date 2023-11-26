/*Khai báo thư viện cần sử dụng*/
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MMA8451.h>
#include "MAX30100_PulseOximeter.h"
#include <Arduino_JSON.h>

// #define MMA8451_SDA_PIN 2 //D4
// #define MMA8451_SCL_PIN 0 //D3

// #define MAX30100_SDA_PIN 5 //D1
// #define MAX30100_SCL_PIN 4 //D2

/*Thay đổi id, pw và ip mạng local*/
const char* ssid = "HHA2803";
const char* password = "24082001";
const char* serverName = "192.168.0.106";

Adafruit_MMA8451 mma = Adafruit_MMA8451(); 
PulseOximeter pox;

unsigned long lastTime = 0;
// unsigned long stepInterval = 500;  // Thời gian tối thiểu giữa các bước (đơn vị: milliseconds)
unsigned long timerDelay = 30000;  // 30s sẽ cập nhập lại 

// int threshold = 1500;  // Ngưỡng để xác định khi nào đếm bước
// int stepCount = 0;
// bool isStepDetected = false;

String sensorReadings;

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  // Wire.begin(MMA8451_SDA_PIN, MMA8451_SCL_PIN);
  // Wire.begin(MAX30100_SDA_PIN, MAX30100_SCL_PIN);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

  // if (!mma.begin()) {
  //   Serial.println("Could not find a valid MMA8451 sensor, check wiring!");
  //   while (1);
  // }

  if (!pox.begin()) {
    Serial.println("Could not find a valid MAX30100 sensor, check wiring!");
    while (1);
  }

  Serial.println("Timer set to 30 seconds, it will take 30 seconds before publishing the first reading.");
}

void loop() {
  // Read data from sensors
  // int stepCount = readMMA8451();  // Lấy giá trị bước chân
  int heartRate = readMAX30100();  // Lấy giá trị nhịp tim 

  // Serial.print("stepCount: ");
  // Serial.println(stepCount);  

  Serial.print("heartRate: ");
  Serial.println(heartRate);  
  // Store data
  JSONVar postData;
  postData["userId"] = "ducbui@gmail.com";
  // postData["step_count"] = stepCount;
  postData["heart_rate"] = heartRate;

  // Convert JSON object to string
  String jsonString = JSON.stringify(postData);

  // Send data every 30 seconds
  if ((millis() - lastTime) > timerDelay) {
    if (WiFi.status() == WL_CONNECTED) {
      sensorReadings = httpGETRequest(serverName, jsonString);
      Serial.println(sensorReadings);
    } else {
      Serial.println("WiFi Disconnected");
    }
    lastTime = millis();
  }
}

// int readMMA8451() {
//   sensors_event_t event; 
//   mma.read();
//   mma.getEvent(&event);
//   // Đọc dữ liệu từ cảm biến gia tốc
//   int totalAcc = abs(event.acceleration.x) + abs(event.acceleration.y) + abs(event.acceleration.z);

//   // Kiểm tra nếu vượt quá ngưỡng và đủ thời gian giữa các bước
//   if (totalAcc > threshold && (millis() - lastTime) > stepInterval) {
//     if (!isStepDetected) {
//       // Đếm bước chân khi phát hiện cú nhảy đầu tiên sau một khoảng thời gian không phát hiện
//       stepCount++;
//       isStepDetected = true;
//       Serial.println("Step Detected. Total Steps: " + String(stepCount));
//       lastTime = millis();  // Cập nhật thời điểm lần cuối cùng đếm bước
//     }
//   } else {
//     isStepDetected = false;
//   }

//   return stepCount;
// }

int readMAX30100() {
  pox.update();
  int heartRate = pox.getHeartRate();
  return heartRate;
}

String httpGETRequest(const char* serverName, String data) {
  WiFiClient client;
  HTTPClient http;

  http.begin(client, serverName);

  // Send HTTP POST request with the JSON data
  http.addHeader("Content-Type", "application/json");
  int httpResponseCode = http.POST(data);

  String payload = "{}";

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  } else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }

  http.end();

  return payload;
}
