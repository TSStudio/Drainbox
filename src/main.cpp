#include "Arduino.h"
#include "Adafruit_AHTX0.h"
#include "Wire.h"
#include "pin_defs.h"

TwoWire wire(0);

Adafruit_AHTX0 ahtx0 = Adafruit_AHTX0();

uint16_t pulse_counter = 0;
uint8_t pwm_target_percentage = 0;

uint8_t fan_percentage_preset[4]= {25, 40, 20, 30};

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
    bool result=ledcAttach(FAN_PWM_PIN, 25000, 8); // Attach PWM to fan speed pin with 5kHz frequency and 8-bit resolution
    if(!result){
        Serial.println("Failed to attach PWM to fan speed pin.");
    }
    result=ledcWrite(FAN_PWM_PIN, 10); // Set initial fan speed to 0%
    if(!result){
        Serial.println("Failed to set initial fan speed.");
    }
    Serial.println("Fan PWM initialized successfully.");
}   

uint8_t loop_counter = 0;

void loop() {
    uint16_t pcounter = pulse_counter;
    Serial.printf("Loop counter: %d\n", loop_counter);
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
        ledcWrite(FAN_PWM_PIN, fan_percentage_preset[(loop_counter>>4) & 0x03]);
    }else{
        uint16_t rpm=pcounter*30;
        Serial.print("RPM: ");
        Serial.println(rpm);
    }
    loop_counter++;
    pulse_counter = 0;
    delay(1000);
}