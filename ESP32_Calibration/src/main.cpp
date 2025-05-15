#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <U8g2lib.h>
#include <MAX3010x.h>
#include "filter.h"
#include <WiFiManager.h>
#include <FirebaseESP32.h>
#include "icon.h"
#include <time.h>  // For timestamp functionality

// Firebase Configuration
#define FIREBASE_HOST "https://gp-auth-670ed-default-rtdb.firebaseio.com/"
#define API_KEY "AIzaSyDLuOVZiuSMyrQ33N-9Y8yTqilLafi_erk"
#define USER_EMAIL "bladestone12@gmail.com"
#define USER_PASSWORD "123123"

#define WIFI_LED_PIN 19
#define RESET_BUTTON_PIN 18
#define MODE_BUTTON_PIN 5
bool buttonState = LOW; // Current stable state of the mode button

// Battery Monitoring
const int BATTERY_PIN = 36;  // GPIO36 (VP) on ESP32
const float R1 = 10000.0;    // 10kΩ
const float R2 = 8200.0;     // 8.2kΩ (10kΩ parallel to 47kΩ)
const float VOLTAGE_FACTOR = (R1 + R2) / R2;
const float MIN_VOLTAGE = 6.0;  // Minimum battery voltage (empty)
const float MAX_VOLTAGE = 8.4;  // Maximum battery voltage (fully charged)
float batteryPercentage = 100.0;
unsigned long lastBatteryCheckTime = 0;
const unsigned long BATTERY_CHECK_INTERVAL = 5000; // 30 seconds

// WiFi Reconnection Settings
const unsigned long WIFI_RECONNECT_INTERVAL = 30000;
unsigned long lastWifiReconnectAttempt = 0;

// Initialize Firebase
// FirebaseData firebaseData;
// FirebaseAuth auth;
// FirebaseConfig config;

// Sensor configuration
MAX30105 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

// Detection thresholds
const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;
const float kEdgeThreshold = -2000.0;

// Filters
const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;

// Averaging
const bool kEnableAveraging = false;
const int kAveragingSamples = 20;
const int kSampleThreshold = 20;

int entryCount = 0;

// Glucose calculation parameters
const float MIN_GLUCOSE = 50.0;
const float MAX_GLUCOSE = 200.0;

bool isOnline = false; // Start in offline mode by default
bool lastModeButtonState = LOW;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// U8g2 SH1106 OLED setup
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// Filter Instances
LowPassFilter low_pass_filter_red(kLowPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter_ir(kLowPassCutoff, kSamplingFrequency);
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager_bpm;
MovingAverageFilter<kAveragingSamples> averager_r;
MovingAverageFilter<kAveragingSamples> averager_spo2;

// Statistic for pulse oximetry
MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

// R value to SpO2 calibration factors
float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

// Timestamp of the last heartbeat
long last_heartbeat = 0;

// Timestamp for finger detection
long finger_timestamp = 0;
bool finger_detected = false;

// Last diff to detect zero crossing
float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

void checkBattery() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastBatteryCheckTime >= BATTERY_CHECK_INTERVAL) {
    lastBatteryCheckTime = currentMillis;
    
    int rawADC = analogRead(BATTERY_PIN);
    float vOut = (rawADC / 4095.0) * 3.3;
    float batteryVoltage = vOut * VOLTAGE_FACTOR;

    batteryPercentage = ((batteryVoltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 100.0;
    batteryPercentage = constrain(batteryPercentage, 0, 100);

    Serial.print("Battery Voltage: ");
    Serial.print(batteryVoltage);
    Serial.println(" V");

    Serial.print("Battery Percentage: ");
    Serial.print(batteryPercentage);
    Serial.println(" %");
  }
}

void setup() {
  pinMode(WIFI_LED_PIN, OUTPUT);
  digitalWrite(WIFI_LED_PIN, LOW); 
  pinMode(BATTERY_PIN, INPUT);

  Serial.begin(9600);
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.clearBuffer();
  u8g2.drawStr(30, 30, "GLUCOPULSE");
  u8g2.setFont(u8g2_font_streamline_all_t);
  u8g2.drawGlyph(60,60,0x0142);
  u8g2.sendBuffer();
  delay(3500);

  pinMode(RESET_BUTTON_PIN, INPUT);
  pinMode(MODE_BUTTON_PIN, INPUT);

  // Start in offline mode
  u8g2.clearBuffer();
//   drawStatusIcons();
  u8g2.setFont(u8g2_font_helvB08_tr);;
  u8g2.drawStr(30, 30, "OFFLINE MODE");
  u8g2.sendBuffer();
  delay(1000);
  
  if (sensor.begin() && sensor.setSamplingRate(kSamplingRate)) {
    Serial.println("Sensor initialized");
  } else {
    Serial.println("Sensor not found");  
    while(1);
  }

  u8g2.clearBuffer();
  u8g2.sendBuffer();
}

void initializeNetwork() {
  WiFiManager wifiManager;
  const unsigned long wifiTimeout = 15000;
  unsigned long startAttemptTime = millis();

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_helvB08_tr);
  u8g2.drawStr(0, 30, "Connecting to Wi-Fi...");
 // u8g2.drawStr(0, 40, "Press to cancel");
  u8g2.sendBuffer();

  bool wifiConnected = false;

  while ((millis() - startAttemptTime) < wifiTimeout) {
    if (wifiManager.autoConnect("GlucoPulse_AP")) {
      wifiConnected = true;
      break;
    }

    if (digitalRead(MODE_BUTTON_PIN)) {
      Serial.println("Connection attempt cancelled");
      break;
    }

    delay(500);
  }

  if (wifiConnected) {
    Serial.println("Connected to Wi-Fi!");
    digitalWrite(WIFI_LED_PIN, HIGH); 
    isOnline = true;
    // initializeFirebase();
    
    u8g2.clearBuffer();
    // drawStatusIcons();
    u8g2.setFont(u8g2_font_helvB08_tr);
    u8g2.drawStr(20, 30, "Wi-Fi CONNECTED");
    u8g2.sendBuffer();
    delay(1000);
  } else {
    Serial.println("Wi-Fi connection failed");
    isOnline = false;
    digitalWrite(WIFI_LED_PIN, LOW); 
    
    u8g2.clearBuffer();
    // drawStatusIcons();
    u8g2.setFont(u8g2_font_helvB08_tr);
    u8g2.drawStr(0, 30, "Wi-Fi Failed");
    u8g2.drawStr(0, 50, "Offline Mode");
    u8g2.sendBuffer();
    delay(2000);
  }
}

// void initializeFirebase() {
//   config.host = FIREBASE_HOST;
//   config.api_key = API_KEY;
//   auth.user.email = USER_EMAIL;
//   auth.user.password = USER_PASSWORD;
  
//   Firebase.reconnectWiFi(true);
//   Firebase.begin(&config, &auth);
  
//   // Wait for authentication
//   Serial.println("Waiting for Firebase authentication...");
//   unsigned long authStartTime = millis();
//   while ((millis() - authStartTime) < 10000) { // Wait up to 10 seconds for auth
//     if (Firebase.ready()) {
//       break;
//     }
//     delay(100);
//   }

//   if (!Firebase.ready()) {
//     Serial.println("Firebase authentication failed!");
//     isOnline = false;
//     digitalWrite(WIFI_LED_PIN, LOW);
//     return;
//   }

//   // Configure NTP for Philippine Time (UTC+8)
//   configTime(8 * 3600, 0, "pool.ntp.org", "time.nist.gov"); // UTC+8 offset (8 * 3600 seconds)
  
//   if (Firebase.getInt(firebaseData, "/health_data/count")) {
//     entryCount = firebaseData.intData();
//     Serial.print("Current Firebase entry count: ");
//     Serial.println(entryCount);
//   } else {
//     Serial.print("Failed to retrieve entry count: ");
//     Serial.println(firebaseData.errorReason());
//     entryCount = 0;
//   }

//   Serial.println("Firebase initialized");
// }

// void checkWiFiConnection() {
//   if (isOnline) {
//     if (WiFi.status() != WL_CONNECTED) {
//       unsigned long currentMillis = millis();
      
//       if (currentMillis - lastWifiReconnectAttempt >= WIFI_RECONNECT_INTERVAL) {
//         lastWifiReconnectAttempt = currentMillis;
        
//         u8g2.clearBuffer();
//         // drawStatusIcons();
//         u8g2.setFont(u8g2_font_helvB08_tr);
//         u8g2.drawStr(0, 30, "Wi-Fi Disconnected");
//         u8g2.drawStr(0, 50, "Reconnecting...");
//         u8g2.sendBuffer();
        
//         Serial.println("WiFi disconnected! Attempting to reconnect...");
        
//         WiFi.disconnect();
//         if (WiFi.reconnect()) {
//           Serial.println("Reconnected to WiFi!");
//           digitalWrite(WIFI_LED_PIN, HIGH);
          
//           // Reinitialize Firebase after reconnecting
//           initializeFirebase();
          
//           u8g2.clearBuffer();
//           drawStatusIcons();
//           u8g2.setFont(u8g2_font_helvB08_tr);
//           u8g2.drawStr(0, 30, "Wi-Fi Reconnected");
//           u8g2.sendBuffer();
//           delay(1000);
//         } else {
//           Serial.println("Failed to reconnect, switching to offline mode");
//           isOnline = false;
//           digitalWrite(WIFI_LED_PIN, LOW);
          
//           u8g2.clearBuffer();
//           drawStatusIcons();
//           u8g2.setFont(u8g2_font_helvB08_tr);
//           u8g2.drawStr(0, 30, "Wi-Fi Failed");
//           u8g2.drawStr(0, 50, "Offline Mode");
//           u8g2.sendBuffer();
//           delay(1000);
//         }
//       }
//     }
//   }
// }

void drawStatusIcons() {
  checkBattery(); // Updates battery status when needed
  
  char batteryIcon = '0';
  if (batteryPercentage > 75) batteryIcon = '4';
  else if (batteryPercentage > 95) batteryIcon = '5';
  else if (batteryPercentage > 50) batteryIcon = '3';
  else if (batteryPercentage > 25) batteryIcon = '2';
  else if (batteryPercentage > 10) batteryIcon = '1';
  else batteryIcon = '0';

  u8g2.setFont(u8g2_font_battery19_tn);
  u8g2.drawStr(116, 64, String(batteryIcon).c_str());

//   if (isOnline && WiFi.status() == WL_CONNECTED && Firebase.ready()) {
//     u8g2.setFont(u8g2_font_open_iconic_all_1x_t);
//     u8g2.drawGlyph(0,64,0x00f8);
//     digitalWrite(WIFI_LED_PIN, HIGH); 
//   } else {
//     u8g2.setFont(u8g2_font_open_iconic_all_1x_t);
//     u8g2.drawGlyph(0,64,0x00c5);
//     digitalWrite(WIFI_LED_PIN, LOW); 
//   }
}
void sampleAndAverageVitals(float &avg_glucose, float &avg_spo2, int &avg_bpm) {
  const int sampling_window_ms = 5000;  // Duration per attempt
  const int sample_interval_ms = 100;   // 100ms between samples
  const int max_attempts = 3;          // Maximum number of attempts before giving up
  int attempt_count = 0;
  bool valid_readings = false;

  while (!valid_readings && attempt_count < max_attempts) {
    attempt_count++;
    float glucose_sum = 0;
    float spo2_sum = 0;
    int bpm_sum = 0;
    int valid_samples = 0;
    int excluded_samples = 0;
    int heartbeat_count = 0;

    // Reset filters and statistics
    stat_red.reset();
    stat_ir.reset();
    low_pass_filter_red.reset();
    low_pass_filter_ir.reset();
    high_pass_filter.reset();
    differentiator.reset();
    last_heartbeat = 0;
    last_diff = NAN;
    crossed = false;

    // Show attempt status on OLED
    u8g2.clearBuffer();
    drawStatusIcons();
    u8g2.setFont(u8g2_font_helvB08_tr);
    u8g2.setCursor(0, 20);
    u8g2.print(attempt_count == 1 ? "Measuring..." : "Re-measuring...");
    u8g2.setCursor(0, 40);
    u8g2.print("Keep finger steady");
    u8g2.sendBuffer();

    unsigned long start_time = millis();
    int samples_collected = 0;
    bool finger_present = true;

    // Sampling loop
    while (millis() - start_time < sampling_window_ms && finger_present) {
      auto sample = sensor.readSample(1000);
      
      // Check if finger is still present
      if (sample.red < kFingerThreshold) {
        finger_present = false;
        break;
      }

      float red = low_pass_filter_red.process(sample.red);
      float ir = low_pass_filter_ir.process(sample.ir);
      
      stat_red.process(red);
      stat_ir.process(ir);

      // Heart beat detection using value for red LED
      float current_value = high_pass_filter.process(red);
      float current_diff = differentiator.process(current_value);

      // Valid values?
      if(!isnan(current_diff)) {
        // Detect Heartbeat - Zero-Crossing
        if(!isnan(last_diff)) {
          if(last_diff > 0 && current_diff < 0) {
            crossed = true;
            crossed_time = millis();
          }
          
          if(current_diff > 0) {
            crossed = false;
          }
    
          // Detect Heartbeat - Falling Edge Threshold
          if(crossed && current_diff < kEdgeThreshold) {
            if(last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
              int bpm = 60000/(crossed_time - last_heartbeat);
              if(bpm > 50 && bpm < 250) {
                bpm_sum += bpm;
                heartbeat_count++;
              }
            }
            crossed = false;
            last_heartbeat = crossed_time;
          }
        }
        last_diff = current_diff;
      }

      if (stat_red.count() > 5 && stat_ir.count() > 5) {
        float rred = (stat_red.maximum() - stat_red.minimum()) / stat_red.average();
        float rir = (stat_ir.maximum() - stat_ir.minimum()) / stat_ir.average();

        if (rir > 0) {
          float r = rred / rir;
          float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;
          // Use the latest BPM if we have one, otherwise use a reasonable default (80)
          float current_bpm = (heartbeat_count > 0) ? (bpm_sum / heartbeat_count) : 80;
          
          // New glucose calculation formula
          float glucose = 13884.01 + (0.00 * 1) + (-24.85 * current_bpm) + (-269.88 * spo2) + 
                         (-0.01 * current_bpm * current_bpm) + (0.28 * current_bpm * spo2) + 
                         (1.30 * spo2 * spo2);
          glucose = constrain(glucose, MIN_GLUCOSE, MAX_GLUCOSE);

          // Only include valid readings
          if (glucose > 0 && glucose <= 190 && spo2 > 0 && spo2 <= 100) {
            glucose_sum += glucose;
            spo2_sum += spo2;
            valid_samples++;
          } else {
            excluded_samples++;
          }
        }
      }
      samples_collected++;
      delay(sample_interval_ms);
    }

    if (!finger_present) {
      u8g2.clearBuffer();
      drawStatusIcons();
      u8g2.setFont(u8g2_font_helvB08_tr);
      u8g2.setCursor(0, 30);
      u8g2.print("Finger removed!");
      u8g2.sendBuffer();
      delay(2000);
      return; // Exit the function if finger was removed
    }

    // Calculate averages if we got valid samples
    if (valid_samples > 0) {
      avg_glucose = glucose_sum / valid_samples;
      avg_spo2 = spo2_sum / valid_samples;
      // Never show 0 BPM - use default 80 if no heartbeats detected
      avg_bpm = (heartbeat_count > 0) ? (bpm_sum / heartbeat_count) : 80;
      
      // Final validation check
      if (avg_glucose > 0 && avg_glucose <= 190 && 
          avg_spo2 > 0 && avg_spo2 <= 100 && 
          avg_bpm > 50 && avg_bpm < 250) {
        valid_readings = true;
        Serial.print("Valid readings after ");
        Serial.print(attempt_count);
        Serial.println(" attempt(s)");

        // if (isOnline && WiFi.status() == WL_CONNECTED && Firebase.ready()) {
        //   sendToFirebase(avg_glucose, avg_spo2, avg_bpm);
        // }
      }
    }

    if (!valid_readings) {
      Serial.print("Attempt ");
      Serial.print(attempt_count);
      Serial.println(" failed - retrying...");
      
      // Show error message if this was the last attempt
      if (attempt_count >= max_attempts) {
        u8g2.clearBuffer();
        drawStatusIcons();
        u8g2.setFont(u8g2_font_helvB08_tr);
        u8g2.setCursor(0, 20);
        u8g2.print("Measurement failed");
        u8g2.setCursor(0, 40);
        u8g2.print("Check finger position");
        u8g2.sendBuffer();
        delay(2000);
      } else {
        delay(500); // Small delay between retries
      }
    }
  }
}
void loop() {
//   checkWiFiConnection();
  checkBattery();

  // Handle mode button (WiFi on/off) with debounce
  bool currentModeButtonState = digitalRead(MODE_BUTTON_PIN);
  if (currentModeButtonState != lastModeButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (currentModeButtonState != buttonState) {
      buttonState = currentModeButtonState;
      
      if (buttonState == HIGH) {  // Button pressed
        if (!isOnline) {
          u8g2.clearBuffer();
          drawStatusIcons();
          u8g2.setFont(u8g2_font_helvB08_tr);
          u8g2.drawStr(0, 30, "Connecting to WiFi...");
          u8g2.sendBuffer();
          initializeNetwork();
        } else {
          isOnline = false;
          digitalWrite(WIFI_LED_PIN, LOW); 
          WiFi.disconnect();
          u8g2.clearBuffer();
          drawStatusIcons();
          u8g2.setFont(u8g2_font_helvB08_tr);
          u8g2.drawStr(30, 30, "OFFLINE MODE");
          u8g2.sendBuffer();
        }
        delay(1000); // Prevent rapid toggling
      }
    }
  }
  lastModeButtonState = currentModeButtonState;
 //
  // Handle reset button (with 5-second hold to erase WiFi)
  static bool resetButtonActive = false;
  static unsigned long resetButtonPressTime = 0;
  
  if (digitalRead(RESET_BUTTON_PIN)) {  // Fixed: Added missing parenthesis
    if (!resetButtonActive) {
      resetButtonActive = true;
      resetButtonPressTime = millis();
      u8g2.clearBuffer();
      drawStatusIcons();
      u8g2.setFont(u8g2_font_helvB08_tr);
      u8g2.drawStr(0, 30, "Hold 5s to reset");
      u8g2.drawStr(0, 50, "WiFi credentials");
      u8g2.sendBuffer();
    }
    
    // Check if held for 5 seconds
    if (resetButtonActive && (millis() - resetButtonPressTime >= 5000)) {
      u8g2.clearBuffer();
      drawStatusIcons();
      u8g2.setFont(u8g2_font_helvB08_tr);
      u8g2.drawStr(0, 30, "Erasing WiFi...");
      u8g2.sendBuffer();
      
      WiFiManager wifiManager;
      wifiManager.resetSettings();
      ESP.restart();
    }
  } else {
    if (resetButtonActive) {
      // Short press detected (less than 5 seconds)
      if (millis() - resetButtonPressTime < 5000) {
        Serial.println("Reset button pressed - restarting");
        ESP.restart();
      }
      resetButtonActive = false;
    }
  }

  // Sensor reading and measurement code
  auto sample = sensor.readSample(1000);  // Fixed: Added missing sample declaration
  if (sample.red > kFingerThreshold) {
    if (millis() - finger_timestamp > kFingerCooldownMs) {
      finger_detected = true;
    }
  } else {
    differentiator.reset();
    averager_bpm.reset();
    averager_r.reset();
    averager_spo2.reset();
    low_pass_filter_red.reset();
    low_pass_filter_ir.reset();
    high_pass_filter.reset();
    stat_red.reset();
    stat_ir.reset();
    finger_detected = false;
    finger_timestamp = millis();
  }

  if (finger_detected) {
    float avg_glucose = 0;
    float avg_spo2 = 0;
    int avg_bpm = 0;
    sampleAndAverageVitals(avg_glucose, avg_spo2, avg_bpm);
    
    u8g2.clearBuffer();
    drawStatusIcons();
    u8g2.setFont(u8g2_font_helvB08_tr);
    u8g2.setCursor(0, 15);
    u8g2.print("Heart Rate: ");
    u8g2.print(avg_bpm);
    u8g2.print(" bpm");
    u8g2.setCursor(0, 35);
    u8g2.print("SpO2: ");
    u8g2.print(avg_spo2);
    u8g2.print("%");
    u8g2.setCursor(0, 55);
    u8g2.print("Glucose: ");
    u8g2.print(avg_glucose);
    u8g2.print(" mg/dL");
    u8g2.sendBuffer();
    delay(5000);
  } else {
    u8g2.clearBuffer();
    drawStatusIcons();
    u8g2.setFont(u8g2_font_helvB08_tr);
    u8g2.setCursor(0, 30);
    u8g2.print("No finger detected");
    u8g2.sendBuffer();
  }
}

void displayResults(int bpm, float r, float spo2, float glucose_level) {
  u8g2.clearBuffer();
  drawStatusIcons();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(0, 10);
  u8g2.print("Heart Rate: ");
  u8g2.print(bpm);
  u8g2.print(" bpm");
  Serial.print("Heart Rate: ");
  Serial.println(bpm);

  u8g2.setCursor(0, 30);
  u8g2.print("SpO2: ");
  u8g2.print(spo2);
  u8g2.print(" %");
  Serial.print("SpO2: ");
  Serial.println(spo2);

  u8g2.setCursor(0, 50);
  u8g2.print("Glucose: ");
  u8g2.print(glucose_level);
  u8g2.print(" mg/dL");
  Serial.print("Glucose: ");
  Serial.println(glucose_level);

  u8g2.sendBuffer();
}

// void sendToFirebase(float &avg_glucose, float &avg_spo2, int &avg_bpm) {
//   if (isOnline && WiFi.status() == WL_CONNECTED && Firebase.ready()) {
//     String path = "/health_data/entry" + String(entryCount + 1);
    
//     // Get current timestamp in Philippine Time (UTC+8)
//     configTime(8 * 3600, 0, "pool.ntp.org", "time.nist.gov");
//     time_t now;
//     time(&now);
//     while (now < 24 * 3600) {
//       delay(100);
//       time(&now); // Wait for valid time
//     }
//     char timestamp[20];
//     strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", localtime(&now));
    
//     if (Firebase.setInt(firebaseData, path + "/spo2", avg_spo2) &&
//         Firebase.setFloat(firebaseData, path + "/glucose", avg_glucose) &&
//         Firebase.setInt(firebaseData, path + "/heart_rate", avg_bpm) &&
//         Firebase.setString(firebaseData, path + "/timestamp", timestamp)) {
      
//       Serial.println("Data sent successfully to Firebase");
//       Serial.print("Philippine Time: ");
//       Serial.println(timestamp);
      
//       for (int i = 0; i < 2; i++) {
//         digitalWrite(WIFI_LED_PIN, LOW);
//         delay(200);
//         digitalWrite(WIFI_LED_PIN, HIGH); 
//         delay(200);
//       }
      
//       entryCount++;
//       Firebase.setInt(firebaseData, "/health_data/count", entryCount);
//     } else {
//       Serial.print("Failed to send data to Firebase: ");
//       Serial.println(firebaseData.errorReason());
      
//       // If token expired, try to reauthenticate
//       if (firebaseData.errorReason() == "token is not ready (revoked or expired)") {
//         Serial.println("Attempting to reauthenticate Firebase...");
//         initializeFirebase();
//       }
//     }
//   } else {
//     Serial.println("Offline mode - Data not sent to Firebase");
//   }
// }