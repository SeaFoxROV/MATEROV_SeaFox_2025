// Cable azul del encoder 5v
// Cable negro del encoder GND
// Cable blanco output 1
// Cable rojo output 2

#define ENCODER_A 2 // Canal A del encoder (cable verde) 
#define ENCODER_B 3 // Canal B del encoder (cable amarillo) 

volatile long pulseCount = 0; // Contador de pulsos del encoder
const int pulsesPerRevolution = 11; // Pulsos por revolución del encoder (JGB 37-520)
const float gearReduction = 34.0; // Relación de reducción del motor
const float resolucion_real = 3.28;

void setup() {
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

  Serial.begin(9600); // Inicializar el monitor serial
}

void loop() {
  // Calcular vueltas acumuladas en grados
  float totalDegrees = (pulseCount / (float)pulsesPerRevolution) * 360.0 / gearReduction/resolucion_real;

  // Mostrar las vueltas acumuladas en el monitor serial
  Serial.print("Vueltas acumuladas (grados): ");
  Serial.println(totalDegrees/360);

  delay(1000); // Actualizar cada 500 ms
}

void encoderISR() {
  // Leer el estado del canal B para determinar la dirección
  int stateA = digitalRead(ENCODER_A);
  int stateB = digitalRead(ENCODER_B);

  if (stateA == stateB) {
    pulseCount++; // Incrementar si va en una dirección
  } else {
    pulseCount--; // Decrementar si va en la dirección opuesta
  }
}
