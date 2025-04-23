#include "filter.h"
#include "icon.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <FirebaseESP32.h> // Include Firebase library
#include <MAX3010x.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <WiFiManager.h> // Include WiFiManager
#include <Wire.h>

// Firebase Configuration
#define FIREBASE_HOST "https://gp-auth-670ed-default-rtdb.firebaseio.com/"
// #define FIREBASE_AUTH "gkg28ycr4b1OtY3z1h6wKmYIweU8p1I1v8vNUObT"
#define API_KEY "AIzaSyDLuOVZiuSMyrQ33N-9Y8yTqilLafi_erk"
#define USER_EMAIL "bladestone12@gmail.com"
#define USER_PASSWORD "123123"

#define WIFI_LED_PIN 19

// Initialize Firebase
// FirebaseData firebaseData;
// FirebaseAuth auth;
// FirebaseConfig config;

// Sensor (adjust to your sensor type)
MAX30105 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

// Finger Detection Threshold and Cooldown
const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;

// Edge Detection Threshold (decrease for MAX30100)
const float kEdgeThreshold = -2000.0;

// Filters
const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;

// Averaging
const bool kEnableAveraging = false;
const int kAveragingSamples = 20;
const int kSampleThreshold = 20;

int entryCount = 0;

// U8g2 SH1106 OLED setup
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); // Initialize OLED display

// const uint8_t battery_icon[] PROGMEM = {
// 	0b00111111, 0b11100000,
// 	0b00100000, 0b00100000,
// 	0b00101111, 0b10100000,
// 	0b00101111, 0b10100000,
// 	0b00101111, 0b10100000,
// 	0b00101111, 0b10100000,
// 	0b00100000, 0b00100000,
// 	0b00111111, 0b11100000
//   };

// 12x8 WiFi icon (3 bars)

void setup() {
	Serial.begin(9600);
	u8g2.begin();
	u8g2.setFont(u8g2_font_ncenB08_tr);
	u8g2.clearBuffer();
	u8g2.drawStr(20, 30, "GLUCOPULSE");
	u8g2.sendBuffer();
	delay(2000);
	// Initialize WiFiManager
	// WiFiManager wifiManager;

	// // Try to connect to Wi-Fi, or start a config portal
	// if (!wifiManager.autoConnect("GlucoPulse_AP")) {
	// 	Serial.println("Failed to connect to Wi-Fi and the timeout expired.");
	// 	// You can choose to reboot or do something else here
	// 	ESP.restart();
	// } else {
	// 	Serial.println("Connected to Wi-Fi!");
	// // LED ON
	//  pinMode(WIFI_LED_PIN, OUTPUT);
	//  digitalWrite(WIFI_LED_PIN, HIGH);
	// 	u8g2.setFont(u8g2_font_5x7_tr);
	// 	u8g2.setCursor(92, 60);
	// 	u8g2.print("ONLINE");
	// 	u8g2.sendBuffer(); // Update the OLED display with the message
	// }

	// Firebase Configuration Setup
	// config.host = FIREBASE_HOST;
	// config.api_key = API_KEY;
	// auth.user.email = USER_EMAIL;
	// auth.user.password = USER_PASSWORD;

	// Firebase.begin(&config, &auth);
	// Firebase.reconnectWiFi(true);

	// if (Firebase.getInt(firebaseData, "/health_data/count")) {
	// 	entryCount = firebaseData.intData();
	// 	Serial.print("Current Firebase entry count: ");
	// 	Serial.println(entryCount);
	// } else {
	// 	Serial.print("Failed to retrieve entry count: ");
	// 	Serial.println(firebaseData.errorReason());
	// 	entryCount = 0;
	// }

	// Serial.println("Firebase initialized");

	// Initialize sensor
	if (sensor.begin() && sensor.setSamplingRate(kSamplingRate)) {
		Serial.println("Sensor initialized");
	} else {
		Serial.println("Sensor not found");
		while (1)
			;
	}

	// Initialize the OLED display
	// u8g2.begin();

	// Clear the screen initially
	u8g2.clearBuffer();
	u8g2.sendBuffer();
}

void drawStatusIcons() {
	u8g2.drawXBMP(112, 0, 16, 8, battery_bitmap); // 128 - 16 = 112
	u8g2.drawXBMP(96, 0, 12, 8, wifi_icon);		  // place just left of battery
}

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

void sampleAndAverageVitals(float &avg_glucose, float &avg_spo2) {
	const int duration_ms = 5000;
	const int interval_ms = 100;
	const int num_samples = duration_ms / interval_ms;

	float glucose_sum = 0;
	float spo2_sum = 0;
	int valid_samples = 0;

	// Reset filters and stats
	stat_red.reset();
	stat_ir.reset();
	low_pass_filter_red.reset();
	low_pass_filter_ir.reset();

	u8g2.clearBuffer();
	drawStatusIcons();
	u8g2.setFont(u8g2_font_ncenB08_tr);
	u8g2.setCursor(0, 20);
	u8g2.print("Calculating...");

	u8g2.setCursor(0, 40);
	u8g2.print("Keep your finger");
	u8g2.setCursor(0, 50);
	u8g2.print("on the sensor");
	u8g2.sendBuffer();

	unsigned long start_time = millis();

	while (millis() - start_time < duration_ms) {
		auto sample = sensor.readSample(1000);
		float red = low_pass_filter_red.process(sample.red);
		float ir = low_pass_filter_ir.process(sample.ir);

		if (sample.red > kFingerThreshold) {
			// Update stats for red and IR
			stat_red.process(red);
			stat_ir.process(ir);

			// Only compute once enough samples have been collected
			if (stat_red.count() > 5 && stat_ir.count() > 5) {
				float rred = (stat_red.maximum() - stat_red.minimum()) / stat_red.average();
				float rir = (stat_ir.maximum() - stat_ir.minimum()) / stat_ir.average();

				if (rir > 0) {
					float r = rred / rir;

					float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;
					float bpm = 80; // Placeholder until heartbeat logic is integrated
					//float glucose = 16714.61 + 0.47 * bpm - 351.045 * spo2 + 1.85 * (spo2 * spo2);

					// Clamp values
					glucose = constrain(glucose, 1, 400);
					spo2 = constrain(spo2, 20, 120);

					glucose_sum += glucose;
					spo2_sum += spo2;
					valid_samples++;
				}
			}
		}

		delay(interval_ms);
	}

	if (valid_samples > 0) {
		avg_glucose = glucose_sum / valid_samples;
		avg_spo2 = spo2_sum / valid_samples;
	} else {
		avg_glucose = 0;
		avg_spo2 = 0;
	}

	Serial.print("Averaged Glucose: ");
	Serial.println(avg_glucose);
	Serial.print("Averaged SpO2: ");
	Serial.println(avg_spo2);
}

void loop() {
	auto sample = sensor.readSample(1000);
	float current_value_red = sample.red;
	float current_value_ir = sample.ir;

	// Detect Finger using raw sensor value
	if (sample.red > kFingerThreshold) {
		if (millis() - finger_timestamp > kFingerCooldownMs) {
			finger_detected = true;
		}
	} else {
		// Reset values if the finger is removed
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

		// current_value_red = low_pass_filter_red.process(current_value_red);
		// current_value_ir = low_pass_filter_ir.process(current_value_ir);

		// // Statistics for pulse oximetry
		// stat_red.process(current_value_red);
		// stat_ir.process(current_value_ir);

		// // Heartbeat detection using value for red LED
		// float current_value = high_pass_filter.process(current_value_red);
		// float current_diff = differentiator.process(current_value);

		// // Valid values?
		// if (!isnan(current_diff) && !isnan(last_diff)) {

		// 	// Detect Heartbeat - Zero-Crossing
		// 	if (last_diff > 0 && current_diff < 0) {
		// 		crossed = true;
		// 		crossed_time = millis();
		// 	}

		// 	if (current_diff > 0) {
		// 		crossed = false;
		// 	}

		// 	// Detect Heartbeat - Falling Edge Threshold
		// 	if (crossed && current_diff < kEdgeThreshold) {
		// 		if (last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
		// 			// Calculate Results
		// 			int bpm = 60000 / (crossed_time - last_heartbeat);
		// 			float rred = (stat_red.maximum() - stat_red.minimum()) / stat_red.average();
		// 			float rir = (stat_ir.maximum() - stat_ir.minimum()) / stat_ir.average();
		// 			float r = rred / rir;
		// 			float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;

		// 			// Calculate glucose level
		// 			float glucose_level = 16714.61 + 0.47 * bpm - 351.045 * spo2 + 1.85 * (spo2 * spo2);

		// 			if (bpm > 50 && bpm < 250) {
		// 				// Average?
		// 				if (kEnableAveraging) {
		// 					int average_bpm = averager_bpm.process(bpm);
		// 					int average_r = averager_r.process(r);
		// 					int average_spo2 = averager_spo2.process(spo2);
		// 					int average_glucose = averager_spo2.process(glucose_level);

		// 					// Show on OLED if enough samples have been collected
		// 					if (averager_bpm.count() >= kSampleThreshold) {
		// 						displayResults(average_bpm, average_r, average_spo2, average_glucose);
		// 					}
		// 				} else {
		// 					displayResults(bpm, r, spo2, glucose_level);
		// 				}

		// 				// Send data to Firebase
		// 				// sendToFirebase(bpm, spo2, glucose_level);
		// 			}

		// 			// Reset statistic
		// 			stat_red.reset();
		// 			stat_ir.reset();
		// 		}

		// 		crossed = false;
		// 		last_heartbeat = crossed_time;
		// 	}
		// }

		// last_diff = current_diff;

		float avg_glucose = 0;
		float avg_spo2 = 0;

		sampleAndAverageVitals(avg_glucose, avg_spo2);

		// Optionally display it right away
		u8g2.clearBuffer();
		drawStatusIcons();
		u8g2.setFont(u8g2_font_ncenB08_tr);
		u8g2.setCursor(0, 20);
		u8g2.print("Avg Glucose: ");
		u8g2.print(avg_glucose);
		u8g2.setCursor(0, 40);
		u8g2.print("Avg SpO2: ");
		u8g2.print(avg_spo2);
		u8g2.sendBuffer();

		delay(2000); // Display result for a moment
	} else {
		// If no finger detected, show message on OLED and Serial Monitor
		u8g2.clearBuffer();
		drawStatusIcons();
		u8g2.setFont(u8g2_font_ncenB08_tr);
		u8g2.setCursor(0, 20);
		u8g2.print("No finger detected");
		u8g2.setCursor(0, 40);
		u8g2.print("Genly tap the sensor");
		u8g2.sendBuffer(); // Update the OLED display
		Serial.println("No finger detected");
	}
}

void displayResults(int bpm, float r, float spo2, float glucose_level) {
	// Clear the screen buffer
	u8g2.clearBuffer();
	drawStatusIcons();

	// Display heart rate
	u8g2.setFont(u8g2_font_ncenB08_tr);
	u8g2.setCursor(0, 10);
	u8g2.print("Heart Rate: ");
	u8g2.print(bpm);
	u8g2.print(" bpm");
	Serial.print("Heart Rate: ");
	Serial.println(bpm);

	// Display SpO2
	u8g2.setCursor(0, 30);
	u8g2.print("SpO2: ");
	u8g2.print(spo2);
	u8g2.print(" %");
	Serial.print("SpO2: ");
	Serial.println(spo2);

	// Display Glucose Level
	u8g2.setCursor(0, 50);
	u8g2.print("Glucose: ");
	u8g2.print(glucose_level);
	u8g2.print(" mg/dL");
	Serial.print("Glucose: ");
	Serial.println(glucose_level);

	// Update the display
	u8g2.sendBuffer();
}

// // Function to send data to Firebase
// void sendToFirebase(int bpm, float spo2, float glucose_level) {
// 	String path = "/health_data/entry" + String(entryCount + 1);
// 	if (Firebase.setInt(firebaseData, path + "/bpm", bpm) &&
// 		Firebase.setInt(firebaseData, path + "/spo2", spo2) &&
// 		Firebase.setFloat(firebaseData, path + "/glucose", glucose_level)) {

// 		Serial.println("Data sent successfully to Firebase");
// 		// blinkCheckSymbol();  // Call the blink function after sending data successfully

// 		// Increment the entry count and update it in Firebase
// 		entryCount++;
// 		Firebase.setInt(firebaseData, "/health_data/count", entryCount);
// 	} else {
// 		Serial.print("Failed to send data to Firebase: ");
// 		Serial.println(firebaseData.errorReason());
// 	}
// }
