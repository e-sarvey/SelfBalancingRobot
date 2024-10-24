#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_SSD1306.h>  // Include OLED library

// I2C and MPU6050 variables
const int MPU = 0x68;              // MPU6050 I2C address
float GyroX, GyroY, GyroZ;
float gyroAngleX, gyroAngleY, gyroanglex, GyroX_last, yaw;
float dt, tt, tt_last;
float mqttGyroX, mqttGyroY, mqttGyroZ; // For MQTT task
float loopFrequency = 0;
// Motor control variables
const int pwmResolution = 16; 
int in1 = 19;
int in2 = 18;
int in3 = 4;
int in4 = 2;
int duty_cycle = 0;
float error = 0;
float max_PWM_angle = 50;
int max_pwm = pow(2,pwmResolution);
int min_PWM = 0.2*max_pwm; // this might need to be rounded
float Kp = 1.0;    // Initial value for Kp
float Kp0 = (max_pwm-min_PWM)/max_PWM_angle; // normalization value for max PWM at 45 degrees
float Kd = 0;     // Initial value for Kd
float anglex = 0;
float roll = 0;  // Rotation about x-axis
// PWM-related definitions
const int pwmFreq = 1000;       // PWM frequency in Hz   
const int pwmChannel1_forward = 0;  // PWM channel for motor 1 forward
const int pwmChannel1_reverse = 1;  // PWM channel for motor 1 reverse
const int pwmChannel2_forward = 2;  // PWM channel for motor 2 forward
const int pwmChannel2_reverse = 3;  // PWM channel for motor 2 reverse
// Timing variables
unsigned long lastMQTTTransmission = 0;

// Wi-Fi and MQTT credentials and settings
const char* ssid = "Tufts_Robot";        // Wi-Fi SSID
const char* password = "";               // Wi-Fi Password
const char* mqtt_server = "10.243.82.33"; // MQTT Broker IP Address
const int mqtt_port = 1883;               // MQTT Broker Port

// Task settings
const int mqttTaskDelayMs = 20;           // MQTT task delay in milliseconds (50Hz)
const int i2cTaskDelayMs = 0.1;           // I2C task delay in milliseconds
const int mqttPublishIntervalMs = 100;     // MQTT data publishing interval (1 second)

// OLED display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

// MQTT client setup
WiFiClient espClient;
PubSubClient client(espClient);

// Task handles
TaskHandle_t MQTTTask;

// Structure to store gains and a flag for new updates
struct BalancingGains {
  float Kp;
  float Kd;
  bool newGainsAvailable;
};

BalancingGains gains = {0.6, 0.0, false};  // Initialize gains with Kp = 20 and Kd = 0
SemaphoreHandle_t gainsMutex;  // Mutex to protect shared data

// Gyro range options (in degrees per second)
#define MPU6050_RANGE_250_DEG  0b00  // ±250 dps
#define MPU6050_RANGE_500_DEG  0b01  // ±500 dps
#define MPU6050_RANGE_1000_DEG 0b10  // ±1000 dps
#define MPU6050_RANGE_2000_DEG 0b11  // ±2000 dps

// Accelerometer range options (in g)
#define MPU6050_RANGE_2_G  0b00  // ±2g
#define MPU6050_RANGE_4_G  0b01  // ±4g
#define MPU6050_RANGE_8_G  0b10  // ±8g
#define MPU6050_RANGE_16_G 0b11  // ±16g

// DLPF bandwidth options (in Hz)
#define MPU6050_BAND_260_HZ 0b000  // 260 Hz
#define MPU6050_BAND_184_HZ 0b001  // 184 Hz
#define MPU6050_BAND_94_HZ  0b010  // 94 Hz
#define MPU6050_BAND_44_HZ  0b011  // 44 Hz
#define MPU6050_BAND_21_HZ  0b100  // 21 Hz
#define MPU6050_BAND_10_HZ  0b101  // 10 Hz
#define MPU6050_BAND_5_HZ   0b110  // 5 Hz

// Function prototypes
void setGyroRange(uint8_t range);
void setAccelRange(uint8_t range);
void setDLPF(uint8_t bandwidth);
void setup_wifi();
void reconnect();
void MQTTTaskCode(void *pvParameters);
void i2cTask(void *pvParameters);
void motor1(int duty_cycle);
void motor2(int duty_cycle);
void mqttCallback(char* topic, byte* payload, unsigned int length);
void updateOLEDDisplay();

// Function to set the gyroscope range
void setGyroRange(uint8_t range) {
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(range << 3);  // Shift the range value into the correct position
  Wire.endTransmission(true);
}

// Function to set the accelerometer range
void setAccelRange(uint8_t range) {
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(range << 3);  // Shift the range value into the correct position
  Wire.endTransmission(true);
}

// Function to set the digital low-pass filter bandwidth
void setDLPF(uint8_t bandwidth) {
  Wire.beginTransmission(MPU);
  Wire.write(0x1A);  // CONFIG register
  Wire.write(bandwidth);  // Set the DLPF bandwidth
  Wire.endTransmission(true);
}

// Function to set up Wi-Fi connection
void setup_wifi() {
  delay(10);
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

// Function to reconnect to MQTT broker
void reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (client.connect("ESP32Gyro")) {
      Serial.println("Connected to MQTT broker");

      // Subscribe to the topic for receiving gains updates
      client.subscribe("ESP32/gains");

      // Show the current gains after connecting to MQTT
      updateOLEDDisplay();  // Ensure OLED is updated with gains
    } else {
      Serial.print("Failed to connect, rc=");
      Serial.print(client.state());
      Serial.println(". Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

// Motor control functions using PWM
void motor1(float anglex, int duty_cycle) {
  int pwmValue = abs(duty_cycle);  // Ensure duty cycle is positive for PWM
  if (anglex > 180.0) {
    ledcWrite(pwmChannel1_forward, pwmValue);  // Forward direction
    ledcWrite(pwmChannel1_reverse, 0);         // No reverse
  } else {
    ledcWrite(pwmChannel1_forward, 0);         // No forward
    ledcWrite(pwmChannel1_reverse, pwmValue);  // Reverse direction
  }
}

void motor2(float anglex, int duty_cycle) {
  int pwmValue = abs(duty_cycle);  // Ensure duty cycle is positive for PWM
  if (anglex > 180.0) {
    ledcWrite(pwmChannel2_forward, pwmValue);  // Forward direction
    ledcWrite(pwmChannel2_reverse, 0);         // No reverse
  } else {
    ledcWrite(pwmChannel2_forward, 0);         // No forward
    ledcWrite(pwmChannel2_reverse, pwmValue);  // Reverse direction
  }
}

// Function to update OLED display with gains
void updateOLEDDisplay() {
  if (xSemaphoreTake(gainsMutex, portMAX_DELAY)) {
    // Print to Serial for debugging
    Serial.print("Updating OLED Display - Kp: ");
    Serial.print(gains.Kp, 2);
    Serial.print(", Kd: ");
    Serial.println(gains.Kd, 2);

    display.clearDisplay();  // Clear the OLED screen
    display.display();
    
    // Display the current gains (Kp and Kd)
    display.setTextSize(1);
    display.setCursor(0,0);
    display.println("GAINS"); // Set cursor to start position for Kp
    display.print("Kp: ");
    display.println(gains.Kp, 2);  // Display Kp with 2 decimal places

    // Move to the next line for Kd
    display.print("Kd: ");
    display.print(gains.Kd, 2);  // Display Kd with 2 decimal places
    
    display.display();  // Push everything to the OLED screen
    
    xSemaphoreGive(gainsMutex);
  } else {
    Serial.println("Failed to take mutex for OLED update.");
  }
}

// MQTT callback function to process incoming messages
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, "ESP32/gains") == 0) {
    char payloadString[100];
    strncpy(payloadString, (char*)payload, length);
    payloadString[length] = '\0';  // Null-terminate the string

    char* start = strstr(payloadString, "[");
    char* end = strstr(payloadString, "]");

    if (start != NULL && end != NULL) {
      start++;
      float newKp, newKd;
      if (sscanf(start, "%f,%f", &newKp, &newKd) == 2) {
        // Update Kp and Kd using a mutex to prevent race conditions
        if (xSemaphoreTake(gainsMutex, portMAX_DELAY)) {
          gains.Kp = newKp;
          gains.Kd = newKd;
          gains.newGainsAvailable = true;  // Indicate new gains are ready to be displayed
          xSemaphoreGive(gainsMutex);
        }

        // Update OLED display when new gains are received
        updateOLEDDisplay();

        Serial.print("Updated Kp: ");
        Serial.print(newKp);
        Serial.print(" | Kd: ");
        Serial.println(newKd);
      }
    }
  }
}

// Task to send data via MQTT and process incoming messages
void MQTTTaskCode(void *pvParameters) {
  unsigned long lastPublishTime = 0;  // Time of last data publish

  while (true) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();  // Handle incoming MQTT messages

    unsigned long now = millis();

    // Publish data periodically
    if (now - lastPublishTime > mqttPublishIntervalMs) {
      lastPublishTime = now;

      // Take mutex to read current gains safely
      if (xSemaphoreTake(gainsMutex, portMAX_DELAY)) {
        // Create a JSON payload with angle estimate, gyro data, PWM, and current gains
        char msg[250];
        snprintf(msg, 250,
          "{\"anglex\": %.2f, \"gyroX\": %.2f, \"gyroY\": %.2f, \"gyroZ\": %.2f, \"PWM\": %d, \"Kp\": %.2f, \"Kd\": %.2f}",
          anglex, mqttGyroX, mqttGyroY, mqttGyroZ, duty_cycle, gains.Kp, gains.Kd);

        // Publish the message to the "ESP32/data" topic
        client.publish("ESP32/data", msg);

        // Release the mutex after reading gains
        xSemaphoreGive(gainsMutex);
      }
    }

    vTaskDelay(1);  // Yield to other tasks
  }
}

// Task for reading IMU data and controlling motors, pinned to core 1
void i2cTask(void *pvParameters) {
  while (true) {
    tt = millis();
    dt = tt - tt_last;

    // === Read gyroscope data === //
    Wire.beginTransmission(MPU);
    Wire.write(0x43);  // Gyro data first register address 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6);

    int16_t rawGyroX = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t rawGyroY = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t rawGyroZ = (int16_t)(Wire.read() << 8 | Wire.read());

    GyroY = rawGyroX / 131.0;
    GyroX = rawGyroY / 131.0;
    GyroZ = rawGyroZ / 131.0;

    // Calculate roll (angle about x-axis) from accelerometer
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // Accelerometer data first register address 0x3B
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6);

    int16_t rawAccY = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t rawAccX = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t rawAccZ = (int16_t)(Wire.read() << 8 | Wire.read());

    float AccX = rawAccX / 16384.0;
    float AccY = rawAccY / 16384.0;
    float AccZ = rawAccZ / 16384.0;

    gyroanglex = ((GyroX - GyroX_last) * (float)dt / 1000) * (180 / PI);
    roll = atan2(AccY, AccZ) + PI;
    roll = roll * (180 / PI);

    // Angle estimation using complementary filter
    anglex = (0.8 * (anglex - gyroanglex) + 0.2 * roll);
    // Control logic
    error = abs(180 - anglex);
    // Motor control logic in i2cTask
    duty_cycle = min_PWM + (gains.Kp * Kp0 * error) - (gains.Kd * abs(GyroX));

    // duty cycle saturation
    if (duty_cycle > max_pwm) {
      duty_cycle = max_pwm;
    }
    if (duty_cycle < -max_pwm) {
      duty_cycle = -max_pwm;
    }
    // Motor control
    motor1(anglex, duty_cycle);
    motor2(anglex, duty_cycle);

    // Update shared values for MQTT
    mqttGyroX = GyroX;
    mqttGyroY = GyroY;
    mqttGyroZ = GyroZ;

    // Calculate and print loop frequency
    loopFrequency = 1000.0 / dt;  // Convert period (ms) to frequency (Hz)
    // Serial.print("Loop frequency: ");
    // Serial.print(loopFrequency);
    // Serial.println(" Hz" );

    tt_last = tt;
    GyroX_last = GyroX;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // Set I2C clock to 400 kHz for faster communication

  // Initialize motor control pins for PWM
  ledcSetup(pwmChannel1_forward, pwmFreq, pwmResolution);
  ledcAttachPin(in1, pwmChannel1_forward);  // PWM for motor 1 forward

  ledcSetup(pwmChannel1_reverse, pwmFreq, pwmResolution);
  ledcAttachPin(in2, pwmChannel1_reverse);  // PWM for motor 1 reverse

  ledcSetup(pwmChannel2_forward, pwmFreq, pwmResolution);
  ledcAttachPin(in3, pwmChannel2_forward);  // PWM for motor 2 forward

  ledcSetup(pwmChannel2_reverse, pwmFreq, pwmResolution);
  ledcAttachPin(in4, pwmChannel2_reverse);  // PWM for motor 2 reverse

  // Initialize WiFi and MQTT
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);  // Set MQTT callback

  // Initialize the OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);  // Loop forever if the display fails to initialize
  }
  display.clearDisplay();  // Clear the display buffer
  display.display();
  display.setTextColor(WHITE);

  // Initialize the mutex
  gainsMutex = xSemaphoreCreateMutex();

  // Display the initial gains on the OLED at startup
  updateOLEDDisplay();
  Serial.println("OLED Configured");

  // Initialize the MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);  // Wake up MPU6050
  Wire.endTransmission(true);
  delay(1000);
  Serial.println("Configuring MPU6050");

  // Configure Gyro Range, Accelerometer Range, and Digital Low-Pass Filter
  setGyroRange(MPU6050_RANGE_1000_DEG);
  setAccelRange(MPU6050_RANGE_2_G);
  setDLPF(MPU6050_BAND_94_HZ);

  Serial.println("MPU6050 configured.");

  // Start the MQTT task with lower priority and pin it to core 0
  xTaskCreatePinnedToCore(MQTTTaskCode, "MQTT Task", 10000, NULL, 1, &MQTTTask, 0);

  // Start the I2C communication (main loop) task on core 1
  xTaskCreatePinnedToCore(i2cTask, "I2C Task", 10000, NULL, 2, NULL, 1);
  Serial.println("Threads Established");
}

void loop() {
  // Empty because i2cTask is running on core 1
}
