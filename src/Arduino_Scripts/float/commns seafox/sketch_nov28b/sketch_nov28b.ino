//receptor
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Pines CE y CSN
#define CE_PIN 48  // Pin CE conectado al pin 48 del Mega
#define CSN_PIN 49 // Pin CSN conectado al pin 49 del Mega

// Crear instancia del módulo RF24
RF24 radio(CSN_PIN, CE_PIN);

// Dirección del canal de comunicación
const byte address[6] = "00001";

void setup() {
  Serial.begin(9600);

  // Inicializar el módulo nRF24L01
  if (!radio.begin()) {
    Serial.println("Error al inicializar el nRF24L01. Verifica las conexiones.");
    while (1); // Detener ejecución en caso de error
  }

  radio.openReadingPipe(0, address); // Configurar dirección de recepción
  radio.setPALevel(RF24_PA_MAX);     // Configurar máxima potencia
  radio.setDataRate(RF24_250KBPS);  // (Opcional) Mejor estabilidad en entornos con interferencia
  radio.startListening();            // Configuración como receptor

  Serial.println("Receptor listo.");
}

void loop() {
  // Verificar si hay datos disponibles
  if (radio.available()) {
    char text[32] = ""; // Buffer para el mensaje recibido
    radio.read(&text, sizeof(text)); // Leer mensaje recibido
    Serial.println(text);            // Imprimir mensaje en el monitor serie
  }
}
