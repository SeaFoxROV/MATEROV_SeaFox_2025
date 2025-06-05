#include <Wire.h>
#include <RTClib.h>
#include "MS5837.h"

// ------------------------------
// CONEXIONES:
// IMU (Ejemplo MPU6050): SDA en A4, SCL en A5
// Encoder:
//   - Azul: 5V
//   - Negro: GND
//   - Blanco: Output 1
//   - Rojo: Output 2
//   - Verde: Canal A -> D2
//   - Amarillo: Canal B -> D3
// Sensor Barométrico MS5837: SDA en A4, SCL en A5

#define IMU_ADDR 0x68     // Dirección I2C del IMU
#define ENCODER_A 2       // Canal A del encoder (cable verde)
#define ENCODER_B 3       // Canal B del encoder (cable amarillo)

volatile long pulseCount = 0; // Contador de pulsos del encoder
const int pulsesPerRevolution = 11; // Pulsos por revolución del encoder
const float gearReduction = 34.0; // Relación de reducción del motor
const float resolucion_real = 3.28; // Ajuste de resolución

RTC_DS3231 rtc;
MS5837 sensor;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    rtc.begin();
    rtc.adjust(DateTime(F(__DATE__),F(__TIME__)));

    // Configuración del encoder
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
    
    // Configuración del IMU (Ejemplo para MPU6050)
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission();
    
    // Inicialización del sensor barométrico
    sensor.setModel(MS5837::MS5837_02BA);
    sensor.init();
    sensor.setFluidDensity(997);
}

void loop() {
    // Obtener fecha y hora
    DateTime now = rtc.now();
    Serial.print(now.year()); Serial.print("/");
    Serial.print(now.month()); Serial.print("/");
    Serial.print(now.day()); Serial.print(" ");
    Serial.print(now.hour()); Serial.print(":");
    Serial.print(now.minute()); Serial.print(":");
    Serial.print(now.second()); Serial.print(" | ");
    
    // Leer ángulo de la IMU
    float imuAngle = leerIMU();
    Serial.print(imuAngle); Serial.print(" | ");
    
    // Calcular vueltas acumuladas en grados del encoder
    float totalDegrees = (pulseCount / (float)pulsesPerRevolution) * 360.0 / gearReduction / resolucion_real;
    Serial.print(totalDegrees / 360); Serial.print(" | ");
    
    // Leer datos del sensor barométrico
    sensor.read();
    Serial.print(sensor.depth());
    Serial.println();
    
    delay(1000);
}

float leerIMU() {
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(IMU_ADDR, 2, true);
    
    if (Wire.available() == 2) {
        int16_t accelX = Wire.read() << 8 | Wire.read();
        return accelX / 16384.0 * 90.0;
    }
    return 0.0;
}

void encoderISR() {
    int stateA = digitalRead(ENCODER_A);
    int stateB = digitalRead(ENCODER_B);
    
    if (stateA == stateB) {
        pulseCount++;
    } else {
        pulseCount--;
    }
}
