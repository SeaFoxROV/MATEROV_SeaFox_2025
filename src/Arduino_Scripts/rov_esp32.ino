// Rangos de pulsos (en microsegundos)
#define SERVO_MIN_PULSE   1000  // 0°
#define SERVO_MAX_PULSE   1720 // 360°

#define NUM_THRUSTERS 6
#define NUM_GRIPPER 3
#define NUM_SIGNALS 9

#define LEDC_TIMER_BITS   14   
#define LEDC_BASE_FREQ    50   

#include <ESP32Servo.h>


int thrusters_pins[6] = {9,12,15,3,5,13}; 
int gripper_pins[3] = {18, 19, 21};
                      //elbow, wrist, gripper
Servo motors[NUM_THRUSTERS];

float pwm_signals[9] = {1500, 1500, 1500,1500,1500,1500,1500,1500,1500};

void setup() {
  Serial.begin(115200);

  Serial.println("Hola");

  for (int i = 0; i < 6; i++) {
    motors[i].attach(thrusters_pins[i]);
    }
  Serial.println("Hola2"); 

  for (int ch = 10; ch < 13; ch++) {
    ledcSetup(ch, LEDC_BASE_FREQ, LEDC_TIMER_BITS);
    ledcAttachPin(gripper_pins[ch], ch);
  }
  
 
    Serial.println("Hola3"); 
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
    for (int i = 0; i < NUM_SIGNALS; i++) {
        end = serial_line.indexOf(';', start);
        if (end == -1) end = serial_line.length();
        pwm_signals[i] = serial_line.substring(start, end).toFloat();
        start = end + 1;
    }
}

void set_motors(float signals[]) {
    for (int i = 0; i < NUM_THRUSTERS; i++) {
      motors[i].writeMicroseconds(signals[i]);
    }
    for (int ch = 10; ch < 13; ch++) {
      uint32_t duty = signals[ch + NUM_THRUSTERS] * ((1 << 14) / (1000000.0 / 50));
      ledcWrite(ch, duty);
    }
    
}

