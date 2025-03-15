import pygame
import serial
import time

# Configuracion puerto de arduino
arduino = serial.Serial("COM7", 115200, timeout=1)
time.sleep(2)  # Esperar a que el puerto se estabilice

# Inicializar pygame y el módulo de joystick
pygame.init()
pygame.joystick.init()

# Verificar si hay joysticks conectados
if pygame.joystick.get_count() == 0:
    print("No se detectó un joystick")
    exit()

# Inicializar el primer joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Usando joystick: {joystick.get_name()}")
print("Moviendo los joysticks...")

while True:
    pygame.event.pump()  # Actualizar eventos de pygame

    # Leer el valor del eje Y del joystick izquierdo
    left_y = int(joystick.get_axis(1) * 255)  # Eje Y del joystick izquierdo

    # Leer el valor del eje Y del joystick derecho

    # Crear la cadena de salida con los valores de ambos ejes
    output = f"{left_y};0;0;0;0;0;0;0;\n"
    arduino.write(output.encode())  # Enviar datos al Arduino por Serial
    print(f"Enviado: {output.strip()}")  # Imprimir en la consola para verificar