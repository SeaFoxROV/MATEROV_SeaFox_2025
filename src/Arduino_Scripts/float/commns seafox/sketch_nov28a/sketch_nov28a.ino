//transmisor
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Pines CE y CSN
#define CE_PIN 48  // Pin CE conectado al pin 48 del Mega
#define CSN_PIN 49 // Pin CSN conectado al pin 49 del Mega

// Configurar el módulo RF24
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001"; // Dirección del canal

void setup() {
  Serial.begin(9600);

  // Inicia el módulo RF24
  if (!radio.begin()) {
    Serial.println("Error al inicializar el nRF24L01. Verifica las conexiones.");
    while (1); // Detener si hay un error
  }

  radio.openWritingPipe(address);   // Establecer dirección del canal
  radio.setPALevel(RF24_PA_HIGH);   // Configurar potencia de transmisión
  radio.setDataRate(RF24_250KBPS); // (Opcional) Configurar tasa de datos
  radio.stopListening();            // Configuración como transmisor

  Serial.println("nRF24L01 configurado correctamente.");
}

void loop() {
  const char text[] = "Hola, receptor!";
  bool success = radio.write(&text, sizeof(text)); // Enviar mensaje
  if (success) {
    Serial.println("Mensaje enviado con éxito!");
  } else {
    Serial.println("Error al enviar el mensaje.");
  }
  delay(1000); // Enviar mensaje cada segundo
}

