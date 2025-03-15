#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "MS5837.h"

#define LEAK_SENSOR_PIN  3  // Sensor de fugas (pin digital)
#define PWM_FREQ 50          // Frecuencia para señales PWM de motores

#define IMU_ADDR 0x68  // Dirección I2C del MPU6050

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // PCA9685 en dirección 0x40

MS5837 bar02;  // Sensor de presión MS5837

float sensores[9];  // Array para almacenar datos de sensores
float pwm_signals[8]; // Array para señales PWM de motores

void setup() {
    Serial.begin(115200);
    Wire.begin();  // Iniciar I2C

    // Inicializar IMU (MPU6050)
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(0x6B); // Registro de encendido
    Wire.write(0x00); // Despertar el sensor
    Wire.endTransmission();

    // Iniciar PCA9685
    pwm.begin();
    pwm.setPWMFreq(PWM_FREQ);

    // Iniciar sensor de presión (Bar02)
    if (!bar02.init()) {
        while (1);
    }
    bar02.setModel(MS5837::MS5837_02BA);
    bar02.setFluidDensity(997); // Agua dulce (997 kg/m³) o salada (1029 kg/m³)

    // Configurar sensor de fuga como entrada
    pinMode(LEAK_SENSOR_PIN, INPUT);
}

void loop() {
    // Leer sensores
    read_sensors();
    imprimir_datos(sensores);

    // Leer comandos por Serial
    if (Serial.available()) {
        String respuesta = Serial.readStringUntil('\n');
        decodeSerial(respuesta);
        set_motors(pwm_signals);
    }
}

void read_sensors() {
    float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;

    Wire.beginTransmission(IMU_ADDR);
    Wire.write(0x3B); // Registro de datos (inicio en Accel X)
    Wire.endTransmission(false);
    Wire.requestFrom(IMU_ADDR, 14, true); // Leer 14 bytes

    if (Wire.available() == 14) {
        int16_t rawAccelX = Wire.read() << 8 | Wire.read();
        int16_t rawAccelY = Wire.read() << 8 | Wire.read();
        int16_t rawAccelZ = Wire.read() << 8 | Wire.read();
        Wire.read(); Wire.read(); // Ignorar temperatura
        int16_t rawGyroX = Wire.read() << 8 | Wire.read();
        int16_t rawGyroY = Wire.read() << 8 | Wire.read();
        int16_t rawGyroZ = Wire.read() << 8 | Wire.read();

        sensores[0] = rawAccelX / 16384.0; // Aceleración en G (±2g)
        sensores[1] = rawAccelY / 16384.0;
        sensores[2] = rawAccelZ / 16384.0;

        sensores[3] = rawGyroX / 131.0; // Giroscopio en °/s (±250°/s)
        sensores[4] = rawGyroY / 131.0;
        sensores[5] = rawGyroZ / 131.0;
    }
    else{for(int i = 0;i<6;i++)sensores[i] = 0;}


    // Leer Bar02
    bar02.read();
    sensores[6] = bar02.depth();       // Profundidad en metros
    sensores[7] = bar02.pressure();    // Presión en mbar
    sensores[8] = bar02.temperature(); // Temperatura en °C

    // Leer sensor de fuga
    sensores[9] = digitalRead(LEAK_SENSOR_PIN);
}

void imprimir_datos(float data[]) {
    /*for (int i = 0; i < 10; i++) {
        Serial.print(data[i]);
        Serial.print(";");
    }*/
    Serial.println(data[7]);
}

void decodeSerial(String serial_line) {
    int start = 0, end = 0;
    for (int i = 0; i < 8; i++) {
        end = serial_line.indexOf(';', start);
        if (end == -1) end = serial_line.length();
        pwm_signals[i] = serial_line.substring(start, end).toFloat();
        start = end + 1;
    }
}

void set_motors(float pwm_signals[]) {
    for (int i = 0; i < 8; i++) {
        int pwm_value = map(pwm_signals[i], 0, 255, 1000, 2000);
        pwm.setPWM(i, 0, pwm_value);
    }
}