#include "Arduino.h"
#include "Adafruit_AHTX0.h"
#include "Wire.h"
#include "pin_defs.h"
#include "PID_v1.h"
#include "WiFi.h"
#include "PubSubClient.h"

TwoWire wire(0);

Adafruit_AHTX0 ahtx0 = Adafruit_AHTX0();

uint16_t pulse_counter = 0;
double last_rpm=0;

uint8_t target_fan_percentage = 64; // 0-255
double target_fan_percentage_pid = 64; // 0-255
double target_fan_rpm = 2220;

PID fanPID(&last_rpm, &target_fan_percentage_pid, &target_fan_rpm, 0.06, 0.02, 0, DIRECT);

typedef enum{
    CONTROL_MODE_PERCENTAGE=0,
    CONTROL_MODE_RPM=1,
} fan_control_mode_t;
fan_control_mode_t fan_control_mode = CONTROL_MODE_RPM;

void interrupt_fan_pulse() {
    // Handle fan pulse interrupt here
    pulse_counter++;
}

void setup() {
    Serial.begin(115200);
    wire.setPins(SDA_PIN, SCL_PIN);
    wire.begin();
    ahtx0.begin(&wire, 0, AHTX0_I2CADDR_DEFAULT);
    Serial.println("AHTX0 initialized successfully.");

    attachInterrupt(digitalPinToInterrupt(FAN_SPEED_PIN), interrupt_fan_pulse, RISING);
    pinMode(FAN_PWM_PIN, OUTPUT);
    // set to 0
    // digitalWrite(FAN_PWM_PIN, LOW);
    bool result=ledcAttach(FAN_PWM_PIN, 25000, 8); // Attach PWM to fan speed pin with 25kHz frequency and 8-bit resolution
    if(!result){
        Serial.println("Failed to attach PWM to fan speed pin.");
    }
    result=ledcWrite(FAN_PWM_PIN, 10); // Set initial fan speed to 0%
    if(!result){
        Serial.println("Failed to set initial fan speed.");
    }
    Serial.println("Fan PWM initialized successfully.");
    fanPID.SetMode(AUTOMATIC); // Set PID to automatic mode
}   

uint8_t loop_counter = 0;

void loop() {
    uint16_t pcounter = pulse_counter;
    if((loop_counter & 0x0f) == 0){
        // every 16 loops (16*1000ms = 16s)
        sensors_event_t humidity, temperature;
        ahtx0.getEvent(&humidity, &temperature);
        Serial.print("Temperature: ");
        Serial.print(temperature.temperature);
        Serial.println(" °C");
        Serial.print("Humidity: ");
        Serial.print(humidity.relative_humidity);
        Serial.println(" %");
    }else{
        uint16_t rpm=pcounter*30;
        last_rpm=static_cast<double>(rpm);
        Serial.print("RPM: ");
        Serial.println(rpm);
        if(fan_control_mode == CONTROL_MODE_PERCENTAGE){
            // set fan speed to target percentage
            ledcWrite(FAN_PWM_PIN, target_fan_percentage);
        }else if(fan_control_mode == CONTROL_MODE_RPM){
            // set fan speed to target RPM using PID control
            fanPID.Compute();
            uint8_t output_percentage=0;
            if(target_fan_percentage_pid > 255){
                output_percentage = 255;
            }else if(target_fan_percentage_pid < 0){
                output_percentage = 0;
            }else{
                output_percentage = static_cast<uint8_t>(target_fan_percentage_pid);
            }
            Serial.print("Target fan percentage (PID): ");
            Serial.println(output_percentage);
            ledcWrite(FAN_PWM_PIN, output_percentage);
        }
    }
    loop_counter++;
    pulse_counter = 0;
    delay(1000);
}