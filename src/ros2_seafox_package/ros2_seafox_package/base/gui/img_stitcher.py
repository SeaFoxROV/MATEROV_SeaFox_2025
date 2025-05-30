import cv2
class ImageStitcher:
    def __init__(self, pano_conf_thresh=0.5):
        self.stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
        self.stitcher.setPanoConfidenceThresh(pano_conf_thresh)
        self.stitcher.setWaveCorrection(True)

    def preprocess_image(self, img):
        img = cv2.GaussianBlur(img, (5, 5), 0)
        return img

    def stitch_images(self, images):
        if not images:
            print("No estan cargando las imagenes")
            return None
        
        result = images[0]
        for i in range(len(images)-1):
            print(type(images[i]))
            cv2.imshow("ifjijeiew", images[i])

        for i in range(len(images)-1):
            if result is None or images[i] is None:
                print(f"Imagen inválida detectada en la posición {i}")
                return None
            status, result = self.stitcher.stitch([result, images[i]])

            if status != cv2.Stitcher_OK:
                print(f"Error al unir las imágenes {i} y {i + 1}. Estado: {status}")
                return None
            print(f"Imágenes {i + 1} unidas con éxito.")
        
        cv2.imshow('Fotoesfera', result)
        cv2.imwrite('imagenescamara/prueb01final.jpeg', result)
        cv2.waitKey(0)
