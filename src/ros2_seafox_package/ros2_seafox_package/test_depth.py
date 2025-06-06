import pyrealsense2 as rs
import numpy as np
import cv2

def main():
    # 1. Crear un pipeline
    pipeline = rs.pipeline()
    config = rs.config()

    # 2. Configurar para recibir solo la cámara de profundidad (p. ej. 640x480 @ 30 FPS)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    try:
        # 3. Iniciar la transmisión
        profile = pipeline.start(config)
        print("Pipeline de profundidad iniciado correctamente.")

        # 4. Esperar un par de frames para "calentar" la cámara
        for i in range(5):
            pipeline.wait_for_frames()

        # 5. Tomar un solo frame de profundidad
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            print("❌ No se obtuvo ningún frame de profundidad.")
            return

        # 6. Convertir el frame de profundidad a un array de NumPy para inspección
        depth_image = np.asanyarray(depth_frame.get_data())
        h, w = depth_image.shape
        print(f"✔ Depth frame recibido: resolución = {w}×{h}")

        # Opcional: mostrar la imagen de profundidad en escala de grises (normalizada)
        # Para visualizar bien, convertimos a 8 bits y aplicamos un colormap
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03),
            cv2.COLORMAP_JET
        )
        cv2.namedWindow('Depth Stream', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Depth Stream', depth_colormap)
        print("Presiona cualquier tecla en la ventana para finalizar.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    except Exception as e:
        print(f"❌ Error al iniciar o leer el pipeline: {e}")

    finally:
        # 7. Detener el pipeline para liberar la cámara
        pipeline.stop()
        print("Pipeline detenido.")

if __name__ == "__main__":
    main()
