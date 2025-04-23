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
#include <time.h> // For timestamp functionality
#include <Arduino.h>

// Firebase Configuration
#define FIREBASE_HOST "https://gp-auth-670ed-default-rtdb.firebaseio.com/"
#define API_KEY "AIzaSyDLuOVZiuSMyrQ33N-9Y8yTqilLafi_erk"
#define USER_EMAIL "bladestone12@gmail.com"
#define USER_PASSWORD "123123"

#define WIFI_LED_PIN 19
#define RESET_BUTTON_PIN 18
#define MODE_BUTTON_PIN 5

// Battery Monitoring
const int BATTERY_PIN = 36; // GPIO36 (VP) on ESP32
const float R1 = 10000.0;	// 10kΩ
const float R2 = 8200.0;	// 8.2kΩ (10kΩ parallel to 47kΩ)
const float VOLTAGE_FACTOR = (R1 + R2) / R2;
const float MIN_VOLTAGE = 6.0; // Minimum battery voltage (empty)
const float MAX_VOLTAGE = 8.4; // Maximum battery voltage (fully charged)
float batteryPercentage = 100.0;
unsigned long lastBatteryCheckTime = 0;
const unsigned long BATTERY_CHECK_INTERVAL = 30000; // 30 seconds

// WiFi Reconnection Settings
const unsigned long WIFI_RECONNECT_INTERVAL = 30000;
unsigned long lastWifiReconnectAttempt = 0;

// Initialize Firebase
FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;

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
const float GLUCOSE_BASE = 90.0;
const float BPM_COEFF = 0.3;
const float SPO2_COEFF = -0.7;
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

void checkBattery()
{
	unsigned long currentMillis = millis();

	if (currentMillis - lastBatteryCheckTime >= BATTERY_CHECK_INTERVAL)
	{
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

void setup()
{
	pinMode(WIFI_LED_PIN, OUTPUT);
	digitalWrite(WIFI_LED_PIN, LOW);
	pinMode(BATTERY_PIN, INPUT);

	Serial.begin(9600);
	u8g2.begin();
	u8g2.setFont(u8g2_font_ncenB08_tr);
	u8g2.clearBuffer();
	u8g2.drawStr(30, 40, "GLUCOPULSE");
	u8g2.sendBuffer();
	delay(1500);

	pinMode(RESET_BUTTON_PIN, INPUT);
	pinMode(MODE_BUTTON_PIN, INPUT);

	// Start in offline mode
	u8g2.clearBuffer();
	drawStatusIcons();
	u8g2.setFont(u8g2_font_ncenB08_tr);
	u8g2.drawStr(20, 30, "OFFLINE MODE");
	u8g2.sendBuffer();
	delay(1000);

	if (sensor.begin() && sensor.setSamplingRate(kSamplingRate))
	{
		Serial.println("Sensor initialized");
	}
	else
	{
		Serial.println("Sensor not found");
		while (1)
			;
	}

	u8g2.clearBuffer();
	u8g2.sendBuffer();
}

void initializeNetwork()
{
	WiFiManager wifiManager;
	const unsigned long wifiTimeout = 15000;
	unsigned long startAttemptTime = millis();

	u8g2.clearBuffer();
	u8g2.setFont(u8g2_font_ncenB08_tr);
	u8g2.drawStr(0, 20, "Connecting to Wi-Fi...");
	u8g2.drawStr(0, 40, "Press to cancel");
	u8g2.sendBuffer();

	bool wifiConnected = false;

	while ((millis() - startAttemptTime) < wifiTimeout)
	{
		if (wifiManager.autoConnect("GlucoPulse_AP"))
		{
			wifiConnected = true;
			break;
		}

		if (digitalRead(MODE_BUTTON_PIN))
		{
			Serial.println("Connection attempt cancelled");
			break;
		}

		delay(500);
	}

	if (wifiConnected)
	{
		Serial.println("Connected to Wi-Fi!");
		digitalWrite(WIFI_LED_PIN, HIGH);
		isOnline = true;
		initializeFirebase();

		u8g2.clearBuffer();
		drawStatusIcons();
		u8g2.setFont(u8g2_font_ncenB08_tr);
		u8g2.drawStr(0, 30, "Wi-Fi Connected");
		u8g2.sendBuffer();
		delay(1000);
	}
	else
	{
		Serial.println("Wi-Fi connection failed");
		isOnline = false;
		digitalWrite(WIFI_LED_PIN, LOW);

		u8g2.clearBuffer();
		drawStatusIcons();
		u8g2.setFont(u8g2_font_ncenB08_tr);
		u8g2.drawStr(0, 30, "Wi-Fi Failed");
		u8g2.drawStr(0, 50, "Offline Mode");
		u8g2.sendBuffer();
		delay(2000);
	}
}

void initializeFirebase()
{
	config.host = FIREBASE_HOST;
	config.api_key = API_KEY;
	auth.user.email = USER_EMAIL;
	auth.user.password = USER_PASSWORD;

	Firebase.reconnectWiFi(true);
	Firebase.begin(&config, &auth);

	// Wait for authentication
	Serial.println("Waiting for Firebase authentication...");
	unsigned long authStartTime = millis();
	while ((millis() - authStartTime) < 10000)
	{ // Wait up to 10 seconds for auth
		if (Firebase.ready())
		{
			break;
		}
		delay(100);
	}

	if (!Firebase.ready())
	{
		Serial.println("Firebase authentication failed!");
		isOnline = false;
		digitalWrite(WIFI_LED_PIN, LOW);
		return;
	}

	// Configure NTP for Philippine Time (UTC+8)
	configTime(8 * 3600, 0, "pool.ntp.org", "time.nist.gov"); // UTC+8 offset (8 * 3600 seconds)

	if (Firebase.getInt(firebaseData, "/health_data/count"))
	{
		entryCount = firebaseData.intData();
		Serial.print("Current Firebase entry count: ");
		Serial.println(entryCount);
	}
	else
	{
		Serial.print("Failed to retrieve entry count: ");
		Serial.println(firebaseData.errorReason());
		entryCount = 0;
	}

	Serial.println("Firebase initialized");
}

void checkWiFiConnection()
{
	if (isOnline)
	{
		if (WiFi.status() != WL_CONNECTED)
		{
			unsigned long currentMillis = millis();

			if (currentMillis - lastWifiReconnectAttempt >= WIFI_RECONNECT_INTERVAL)
			{
				lastWifiReconnectAttempt = currentMillis;

				u8g2.clearBuffer();
				drawStatusIcons();
				u8g2.setFont(u8g2_font_ncenB08_tr);
				u8g2.drawStr(0, 30, "Wi-Fi Disconnected");
				u8g2.drawStr(0, 50, "Reconnecting...");
				u8g2.sendBuffer();

				Serial.println("WiFi disconnected! Attempting to reconnect...");

				WiFi.disconnect();
				if (WiFi.reconnect())
				{
					Serial.println("Reconnected to WiFi!");
					digitalWrite(WIFI_LED_PIN, HIGH);

					// Reinitialize Firebase after reconnecting
					initializeFirebase();

					u8g2.clearBuffer();
					drawStatusIcons();
					u8g2.setFont(u8g2_font_ncenB08_tr);
					u8g2.drawStr(0, 30, "Wi-Fi Reconnected");
					u8g2.sendBuffer();
					delay(1000);
				}
				else
				{
					Serial.println("Failed to reconnect, switching to offline mode");
					isOnline = false;
					digitalWrite(WIFI_LED_PIN, LOW);

					u8g2.clearBuffer();
					drawStatusIcons();
					u8g2.setFont(u8g2_font_ncenB08_tr);
					u8g2.drawStr(0, 30, "Wi-Fi Failed");
					u8g2.drawStr(0, 50, "Offline Mode");
					u8g2.sendBuffer();
					delay(1000);
				}
			}
		}
	}
}

void drawStatusIcons()
{
	checkBattery(); // Updates battery status when needed

	char batteryIcon;
	if (batteryPercentage > 75)
		batteryIcon = '4';
	else if (batteryPercentage > 50)
		batteryIcon = '3';
	else if (batteryPercentage > 25)
		batteryIcon = '2';
	else if (batteryPercentage > 10)
		batteryIcon = '1';
	else
		batteryIcon = '0';

	u8g2.setFont(u8g2_font_battery19_tn);
	u8g2.drawStr(116, 19, String(batteryIcon).c_str());

	if (isOnline && WiFi.status() == WL_CONNECTED && Firebase.ready())
	{
		u8g2.setFont(u8g2_font_open_iconic_all_1x_t);
		u8g2.drawGlyph(105, 8, 0x00f8);
		digitalWrite(WIFI_LED_PIN, HIGH);
	}
	else
	{
		u8g2.setFont(u8g2_font_open_iconic_all_1x_t);
		u8g2.drawGlyph(105, 8, 0x00c5);
		digitalWrite(WIFI_LED_PIN, LOW);
	}
}

void sampleAndAverageVitals(float &avg_glucose, float &avg_spo2)
{
	const int duration_ms = 5000;
	const int interval_ms = 100;
	const int num_samples = duration_ms / interval_ms;

	float glucose_sum = 0;
	float spo2_sum = 0;
	int valid_samples = 0;

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

	while (millis() - start_time < duration_ms)
	{
		auto sample = sensor.readSample(1000);
		float red = low_pass_filter_red.process(sample.red);
		float ir = low_pass_filter_ir.process(sample.ir);

		if (sample.red > kFingerThreshold)
		{
			stat_red.process(red);
			stat_ir.process(ir);

			if (stat_red.count() > 5 && stat_ir.count() > 5)
			{
				float rred = (stat_red.maximum() - stat_red.minimum()) / stat_red.average();
				float rir = (stat_ir.maximum() - stat_ir.minimum()) / stat_ir.average();

				if (rir > 0)
				{
					float r = rred / rir;

					float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;
					float bpm = 80;
					float glucose = GLUCOSE_BASE +	BPM_COEFF * (bpm - 72) + SPO2_COEFF * (spo2 - 98);
					glucose = constrain(glucose, MIN_GLUCOSE, MAX_GLUCOSE);

					glucose_sum += glucose;
					spo2_sum += spo2;
					valid_samples++;
				}
			}
		}

		delay(interval_ms);
	}

	if (valid_samples > 0)
	{
		avg_glucose = glucose_sum / valid_samples;
		avg_spo2 = spo2_sum / valid_samples;
	}
	else
	{
		avg_glucose = 0;
		avg_spo2 = 0;
	}

	Serial.print("Averaged Glucose: ");
	Serial.println(avg_glucose);
	Serial.print("Averaged SpO2: ");
	Serial.println(avg_spo2);

	if (isOnline && WiFi.status() == WL_CONNECTED && Firebase.ready())
	{
		sendToFirebase(avg_glucose, avg_spo2);
	}
}

void loop()
{
	checkWiFiConnection();
	checkBattery(); // Now checks every 30 seconds

	bool currentModeButtonState = digitalRead(MODE_BUTTON_PIN);

	if (currentModeButtonState && !lastModeButtonState && (millis() - lastDebounceTime > debounceDelay))
	{
		lastDebounceTime = millis();

		if (!isOnline)
		{
			u8g2.clearBuffer();
			drawStatusIcons();
			u8g2.setFont(u8g2_font_ncenB08_tr);
			u8g2.drawStr(0, 30, "Connecting to WiFi...");
			u8g2.sendBuffer();

			initializeNetwork();
		}
		else
		{
			isOnline = false;
			digitalWrite(WIFI_LED_PIN, LOW);
			WiFi.disconnect();

			u8g2.clearBuffer();
			drawStatusIcons();
			u8g2.setFont(u8g2_font_ncenB08_tr);
			u8g2.drawStr(20, 30, "OFFLINE MODE");
			u8g2.sendBuffer();
		}

		delay(1000);
	}
	lastModeButtonState = currentModeButtonState;

	if (digitalRead(RESET_BUTTON_PIN))
	{
		Serial.println("Reset button pressed!");

		u8g2.begin();
		u8g2.clearBuffer();
		u8g2.setFont(u8g2_font_ncenB08_tr);
		u8g2.drawStr(10, 30, "OLED Reset");
		u8g2.sendBuffer();

		if (sensor.begin() && sensor.setSamplingRate(kSamplingRate))
		{
			Serial.println("Sensor re-initialized");
		}
		else
		{
			Serial.println("Sensor re-init failed");
			u8g2.clearBuffer();
			u8g2.drawStr(0, 30, "Sensor Error");
			u8g2.sendBuffer();
			delay(2000);
		}

		low_pass_filter_red.reset();
		low_pass_filter_ir.reset();
		high_pass_filter.reset();
		differentiator.reset();
		averager_bpm.reset();
		averager_r.reset();
		averager_spo2.reset();
		stat_red.reset();
		stat_ir.reset();

		last_heartbeat = 0;
		last_diff = NAN;
		crossed = false;
		crossed_time = 0;
		finger_detected = false;
		finger_timestamp = millis();

		while (digitalRead(RESET_BUTTON_PIN))
		{
			delay(10);
		}

		Serial.println("Reset complete. Waiting for finger...");
		return;
	}

	auto sample = sensor.readSample(1000);

	if (sample.red > kFingerThreshold)
	{
		if (millis() - finger_timestamp > kFingerCooldownMs)
		{
			finger_detected = true;
		}
	}
	else
	{
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

	if (finger_detected)
	{
		float avg_glucose = 0;
		float avg_spo2 = 0;

		sampleAndAverageVitals(avg_glucose, avg_spo2);

		u8g2.clearBuffer();
		drawStatusIcons();
		u8g2.setFont(u8g2_font_ncenB08_tr);
		u8g2.setCursor(0, 20);
		u8g2.print("Avg SpO2: ");
		u8g2.print(avg_spo2);
		u8g2.setCursor(0, 40);
		u8g2.print("Avg Glucose: ");
		u8g2.print(avg_glucose);
		u8g2.sendBuffer();

		delay(5000);
	}
	else
	{
		u8g2.clearBuffer();
		drawStatusIcons();
		u8g2.setFont(u8g2_font_ncenB08_tr);
		u8g2.setCursor(0, 40);
		u8g2.print("No finger detected");
		u8g2.sendBuffer();
	}
}

void displayResults(int bpm, float r, float spo2, float glucose_level)
{
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

void sendToFirebase(float &avg_glucose, float &avg_spo2)
{
	if (isOnline && WiFi.status() == WL_CONNECTED && Firebase.ready())
	{
		String path = "/health_data/entry" + String(entryCount + 1);

		// Get current timestamp in Philippine Time (UTC+8)
		configTime(8 * 3600, 0, "pool.ntp.org", "time.nist.gov");
		time_t now;
		time(&now);
		while (now < 24 * 3600)
		{
			delay(100);
			time(&now); // Wait for valid time
		}
		char timestamp[20];
		strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", localtime(&now));

		if (Firebase.setInt(firebaseData, path + "/spo2", avg_spo2) &&
			Firebase.setFloat(firebaseData, path + "/glucose", avg_glucose) &&
			Firebase.setString(firebaseData, path + "/timestamp", timestamp))
		{

			Serial.println("Data sent successfully to Firebase");
			Serial.print("Philippine Time: ");
			Serial.println(timestamp);

			for (int i = 0; i < 2; i++)
			{
				digitalWrite(WIFI_LED_PIN, LOW);
				delay(200);
				digitalWrite(WIFI_LED_PIN, HIGH);
				delay(200);
			}

			entryCount++;
			Firebase.setInt(firebaseData, "/health_data/count", entryCount);
		}
		else
		{
			Serial.print("Failed to send data to Firebase: ");
			Serial.println(firebaseData.errorReason());

			// If token expired, try to reauthenticate
			if (firebaseData.errorReason() == "token is not ready (revoked or expired)")
			{
				Serial.println("Attempting to reauthenticate Firebase...");
				initializeFirebase();
			}
		}
	}
	else
	{
		Serial.println("Offline mode - Data not sent to Firebase");
	}
}