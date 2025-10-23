#include <WiFi.h>
#include <FirebaseESP32.h>


#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <time.h>

// Replace with your network credentials
#define WIFI_SSID "S"
#define WIFI_PASSWORD "12345678"

// Replace with your Firebase project credentials
#define API_KEY "AIzaSyABeppC4z0SrUmDDieiAH0W9BTAY233FPE"
#define DATABASE_URL "https://solar-dryer-d2d1c-default-rtdb.firebaseio.com/"

// NTP Server settings
#define NTP_SERVER "pool.ntp.org"
#define GMT_OFFSET_SEC 19800  // India is UTC+5:30 (5.5*60*60)
#define DAYLIGHT_OFFSET_SEC 0
#define BUZZER_PIN 25
// DHT Sensor
#define DHTPIN 15    // ESP32 GPIO pin where DHT is connected
#define DHTTYPE DHT22 // DHT 22 (AM2302)
DHT dht(DHTPIN, DHTTYPE);

#define C_DHTPIN 14
#define C_DHTTYPE DHT22
DHT cdht(C_DHTPIN, C_DHTTYPE);

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Variables to store sensor readings
float temperature = 0.0;
float humidity = 0.0;
float C_temperature = 0.0;
float C_humidity = 0.0;
char timestamp[25]; // Global timestamp buffer - moved here to global scope
String collectorShape="V_noholes";
int buzzerControl = -1;  
int automode=1;
#define FAN_IN1 26  // PWM pin
#define FAN_IN2 27  // Direction pin (keep LOW for this fan)
int fanSpeed = 0;   // 0–255 manual speed
float autoTempThreshold = 20.0; // °C

unsigned long previousMillis = 0;
const long interval = 60000;  // Read sensor every 2 seconds

// Function to get current timestamp in the format "YYYY-MM-DD HH:MM:SS"
void getTimestamp() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    strcpy(timestamp, "0000-00-00 00:00:00");
    return;
  }
  
  strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &timeinfo);
}

void fetchControls() {
  if (Firebase.getFloat(fbdo, "/solarDryer/control/thresholdTemp")) {
    autoTempThreshold = fbdo.floatData();
  }
  if (Firebase.getInt(fbdo, "/solarDryer/control/buzzer")) {
    buzzerControl = fbdo.intData();
  }
  if (Firebase.getInt(fbdo, "/solarDryer/control/autoMode")) {
    automode = fbdo.intData();
  }
}

void updateBuzzerLogic() {
  if (automode == 0) {
    // Manual mode
    if (buzzerControl == 1) {
      digitalWrite(BUZZER_PIN, HIGH);
      Serial.println("Buzzer ON (manual)");
    } else {
      digitalWrite(BUZZER_PIN, LOW);
      Serial.println("Buzzer OFF (manual)");
    }
  } else {
    // Auto mode
    if (temperature > autoTempThreshold) {
      digitalWrite(BUZZER_PIN, HIGH);
      Serial.println("Buzzer ON (auto)");

      // update Firebase so app sees ON
      Firebase.setInt(fbdo, "/solarDryer/control/buzzer", 1);
    } else {
      digitalWrite(BUZZER_PIN, LOW);
      Serial.println("Buzzer OFF (auto)");
      Firebase.setInt(fbdo, "/solarDryer/control/buzzer", 0);
    }
  }
}

void uploadHistory() {
  // 🔥 Get current timestamp in milliseconds
  unsigned long long timestamp = millis();  
  // (Better: use NTP time for real-world timestamp in ms)

  // Path like: /solarDryer/history/Cylinder_holes/1755622998811
  String basePath = "/solarDryer/history/" + collectorShape + "/" + String(timestamp);

  // Save as indexed values
  if (Firebase.setFloat(fbdo, basePath + "/0", temperature)) {
    Serial.println("Temperature history saved");
  }
  if (Firebase.setFloat(fbdo, basePath + "/1", humidity)) {
    Serial.println("Humidity history saved");
  }
  if (Firebase.setFloat(fbdo, basePath + "/2", C_temperature)) {
    Serial.println("Collector Temp history saved");
  }
  if (Firebase.setFloat(fbdo, basePath + "/3", C_humidity)) {
    Serial.println("Collector Humidity history saved");
  }
}


void setup() {
  Serial.begin(9600);
  delay(2000);
  Serial.println("Hello from ESP32");
  
  // Initialize DHT sensors

   pinMode(FAN_IN1, OUTPUT);
  pinMode(FAN_IN2, OUTPUT);
  digitalWrite(FAN_IN2, LOW); // fixed direction
  analogWrite(FAN_IN1, 0);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  dht.begin();
  delay(1000);
  cdht.begin();
  delay(1000);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.println("Connected to Wi-Fi");
  Serial.print("WiFi signal strength (RSSI): ");
  Serial.println(WiFi.RSSI());
  
  // Configure time from NTP server
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  Serial.println("Setting up time sync");
  delay(2000);
  
  // Get initial time
  getTimestamp();
  Serial.print("Current time: ");
  Serial.println(timestamp);
  
  Serial.println("Setting up Firebase config...");
  
  // Increase SSL buffer size
  fbdo.setBSSLBufferSize(4096, 1024);
  fbdo.setResponseSize(4096);
  
  // Firebase configuration
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  
  // Sign in with credentials
  auth.user.email = "shivnanzu@gmail.com";
  auth.user.password = "shiva_shiva";
  
  Serial.println("Starting Firebase...");
  
  // Initialize Firebase
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Firebase.setReadTimeout(fbdo, 1000 * 60); // 1 minute timeout
  Firebase.setwriteSizeLimit(fbdo, "tiny");
  
  Serial.println("Firebase initialized");

  // Retry connection if needed
  int retryCount = 0;
  while (!Firebase.ready() && retryCount < 10) {
    Serial.println("Retrying Firebase connection...");
    delay(1000);
    retryCount++;
  }

  if (Firebase.ready()) {
    Serial.println("Firebase is ready");
  } else {
    Serial.println("Firebase not ready after retries");
  }

  if (Firebase.getString(fbdo, "/solarDryer/latest/Collector_shape")) {
   collectorShape= fbdo.stringData();
  Serial.println("Collector shape: " + collectorShape);
} else {
  Serial.println("Failed to get collector shape");
  Serial.println(fbdo.errorReason());
}
  if (Firebase.getFloat(fbdo, "/solarDryer/control/thresholdTemp")) {
   autoTempThreshold= fbdo.floatData();
  Serial.println("Threshold Temperaure: " +String(autoTempThreshold) );
} else {
  Serial.println("Failed to get collector shape");
  Serial.println(fbdo.errorReason());
}
   

}

void loop() {
  unsigned long currentMillis = millis();
  
  // Only read sensor every 2 seconds
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    // Get current timestamp
    getTimestamp();
    Serial.print("Current time: ");
    Serial.println(timestamp);
    
    // Read sensor values
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();
    C_humidity = cdht.readHumidity();
    C_temperature = cdht.readTemperature();
    
    // Upload timestamp to Firebase
    if (Firebase.setString(fbdo, "/solarDryer/latest/timestamp", timestamp)) {
      Serial.println("Timestamp uploaded successfully");
    } else {
      Serial.println("Failed to upload timestamp");
      Serial.println(fbdo.errorReason());
    }
    
    // Check if readings failed for first sensor
    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Failed to read from DHT sensor!");
      
      // Try to reinitialize the sensor
      dht.begin();
      delay(1000);
    } else {
      Serial.printf("Temp: %.2f C, Humidity: %.2f %%\n", temperature, humidity);
      
      
      if (Firebase.setFloat(fbdo, "/solarDryer/latest/temperature", temperature)) {
        Serial.println("Temperature uploaded successfully");
      } else {
        Serial.println("Failed to upload temperature");
        Serial.println(fbdo.errorReason());
      }
      
      if (Firebase.setFloat(fbdo, "/solarDryer/latest/humidity", humidity)) {
        Serial.println("Humidity uploaded successfully");
      } else {
        Serial.println("Failed to upload humidity");
        Serial.println(fbdo.errorReason());
      }

      
    }

    // Check if readings failed for second sensor
    if (isnan(C_temperature) || isnan(C_humidity)) {
      Serial.println("Failed to read from Collector DHT sensor!");
      // Try to reinitialize the sensor
      cdht.begin();
      delay(1000);
    } else {
      Serial.printf("Sensor 2 - C_Temp: %.2f C, C_Humidity: %.2f %%\n", C_temperature, C_humidity);
      
      // Upload to Firebase with C_ endpoints
      if (Firebase.setFloat(fbdo, "/solarDryer/latest/Collector_temperature", C_temperature)) {
        Serial.println("C_temperature uploaded successfully");
      } else {
        Serial.println("Failed to upload C_temperature");
        Serial.println(fbdo.errorReason());
      }
      
      if (Firebase.setFloat(fbdo, "/solarDryer/latest/Collector_humidity", C_humidity)) {
        Serial.println("C_humidity uploaded successfully");
      } else {
        Serial.println("Failed to upload C_humidity");
        Serial.println(fbdo.errorReason());
      }
    }
    uploadHistory();
  }

  

  if (Firebase.getInt(fbdo, "/solarDryer/control/fanSpeed")) { // Flutter app sets this
    fanSpeed = fbdo.intData();
    fanSpeed = constrain(fanSpeed, 0, 255);
    Serial.printf("Fan speed from Firebase: %d\n", fanSpeed);
  } else {
    Serial.println("Failed to read fan speed: " + fbdo.errorReason());
  }

  // Temperature-based override
  if (temperature > autoTempThreshold) {
    analogWrite(FAN_IN1, 255); // full speed
    Serial.println("Auto mode: High temp, full speed");
  } else {
    analogWrite(FAN_IN1, fanSpeed); // manual speed
  }
  fetchControls();
 
  updateBuzzerLogic();



  delay(500); // Check fan speed twice a second
  
  // Other tasks can run here without blocking
  
}