#include <ESP32Servo.h>

#define NUM_MOTORS 8
int motor_pins[NUM_MOTORS] = {13,14,32,27,26,25}; 

Servo motors[NUM_MOTORS];
float pwm_signals[NUM_MOTORS] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

void setup() {
    Serial.begin(115200);

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].attach(motor_pins[i], 1000, 2000);
    }
}

void loop() {
    if (Serial.available()) {
        String linea = Serial.readStringUntil('\n');
        decodeSerial(linea);  
    }
    set_motors(pwm_signals);
}

void decodeSerial(String serial_line) {
    int start = 0, end = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
        end = serial_line.indexOf(';', start);
        if (end == -1) end = serial_line.length();
        pwm_signals[i] = serial_line.substring(start, end).toFloat();
        start = end + 1;
    }
}

void set_motors(float signals[]) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].writeMicroseconds(signals[i]);
    }
}
