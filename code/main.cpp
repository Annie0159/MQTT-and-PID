/* Assignment 4 Starter Code
Libraries Needed:
FreeRTOS
ESP32Servo
PubSubClient

Please change the servo_topic to use your uark username

*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <ESP32Servo.h>
#include "esp_random.h"
#include "PubSubClient.h"
#include <math.h>

const char* username = "am442";  

// MQTT topics
char data_topic[64];
char cmd_topic[64];
char status_topic[64];

const char* broker = "broker.hivemq.com";
const int brokerPort = 1883;
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASSWORD ""
// Defining the WiFi channel speeds up the connection:
#define WIFI_CHANNEL 6

#define SERVO_UPDATE_MS 50          // 20 Hz update
#define NOISE_STDDEV    2.5f        // degrees

const int SERVO = 18;
Servo myservo;        // create servo object to control a servo

// Globals modified by tasks
float mqtt_setpoint = 90.0f;    // Comes from MQTT message handler
float servo_angle_true = 90.0f;    // Simulated actual servo angle
float pid_output = 0.0f;   // Student writes their PID output here

// PID globals
float kp = 0.50;
float ki = 0.10;
float kd = 0.20;

float last_error = 0.0f;
float integral = 0.0f;

// Generate normally distributed random noise using Box–Muller transform
static float random_normal(float mean, float stddev)
{
    float u1 = esp_random() / (float)UINT32_MAX;
    float u2 = esp_random() / (float)UINT32_MAX;
    float z0 = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI * u2);
    return mean + z0 * stddev;
}

// MQTT callback — runs whenever a subscribed topic receives a message
void mqtt_callback(char* topic, byte* payload, unsigned int length)
{
    // Incoming DATA drives setpoint
    // Convert payload to float
    char msg[32];
    memcpy(msg, payload, length);
    msg[length] = '\0';

    mqtt_setpoint = atof(msg);
    Serial.printf("MQTT Setpoint: %.2f\n", mqtt_setpoint);
}

// Configure MQTT topics using username
void mqtt_setup()
{
    mqtt.setServer(broker, brokerPort);
    mqtt.setCallback(mqtt_callback);

    sprintf(data_topic,   "uark/CSCE41104/%s/data", username);
    sprintf(cmd_topic,    "uark/CSCE41104/%s/commands", username);
    sprintf(status_topic, "uark/CSCE41104/%s/status", username);
}

// Connect to MQTT broker, retrying until successful
void mqtt_connect()
{
    while (!mqtt.connected()) {
        Serial.println("Connecting to MQTT...");

        String clientId = "esp32_";
        clientId += String(random(0xffff), HEX);

        if (mqtt.connect(clientId.c_str())) {
            Serial.println("Connected to MQTT.");

            // Subscribe for DATA messages
            mqtt.subscribe(data_topic);

            // Send START command
            mqtt.publish(cmd_topic, "START");
        }
        else {
            Serial.print("Failed, rc=");
            Serial.println(mqtt.state());
            delay(2000);
        }
    }
}

// Servo Task: Runs continuously
void servo_task(void* params)
{
    myservo.attach(SERVO, 500, 2400); // Attach servo with pulse bounds
    servo_angle_true = mqtt_setpoint; // Initialize with setpoint

    while (1) {
        float influence_mqtt = 0.10f * (mqtt_setpoint - servo_angle_true);
        float influence_pid  = 0.20f * pid_output;
        float noise          = random_normal(0.0f, NOISE_STDDEV);

        servo_angle_true += influence_mqtt + influence_pid + noise;

        // Clamp angle
        if (servo_angle_true < 0.0f) servo_angle_true = 0.0f;
        if (servo_angle_true > 180.0f) servo_angle_true = 180.0f;

        myservo.write(servo_angle_true);

        vTaskDelay(pdMS_TO_TICKS(SERVO_UPDATE_MS)); // Periodic update
    }
}

// Attempts to regulate servo_angle_true to 90
void pid_task(void* params)
{
    const TickType_t delayTicks = pdMS_TO_TICKS(50); // 20 Hz

    while (1) {
        float error = 90.0f - servo_angle_true; // Target = 90 degrees

        integral += error * 0.05f;   // dt = 0.05s
        float derivative = (error - last_error) / 0.05f;

        pid_output = kp * error + ki * integral + kd * derivative;

        last_error = error;

        vTaskDelay(delayTicks);
    }
}

// Publishes current servo state to MQTT
void status_task(void* params)
{
    const TickType_t delayTicks = pdMS_TO_TICKS(1000);
    char payload[64];

    while (1) {
        float currentAngle = servo_angle_true;
        float p_error = 90.0f - currentAngle;
        float d_error = (p_error - last_error) / 0.05f;

        snprintf(payload, sizeof(payload),
                 "%.2f,%.3f,%.3f,%.3f",
                 currentAngle, p_error, d_error, integral);
        // Send MQTT status message
        if (mqtt.connected()) {
            mqtt.publish(status_topic, payload);
            Serial.print("Status: ");
            Serial.println(payload);
        }
        vTaskDelay(delayTicks);
    }
}

//main
void setup()
{
    Serial.begin(115200);
    Serial.println("Starting Simulation...");

    // WiFi connection
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL);
    Serial.print("Connecting");

    while (WiFi.status() != WL_CONNECTED) {
        delay(100);
        Serial.print(".");
    }

    Serial.println("\nWiFi Connected!");
    mqtt_setup();
    mqtt_connect();

    // CREATE TASKS
    xTaskCreate(servo_task,   "Servo Task",   2000, NULL, 1, NULL);
    xTaskCreate(pid_task,     "PID Task",     4096, NULL, 1, NULL);
    xTaskCreate(status_task,  "Status Task",  8192, NULL, 1, NULL);
}

// Only ensures MQTT stays connected & processes incoming messages
void loop()
{
    if (!mqtt.connected()) {
        mqtt_connect();
    }
    mqtt.loop(); // Process MQTT
    delay(20);
}
