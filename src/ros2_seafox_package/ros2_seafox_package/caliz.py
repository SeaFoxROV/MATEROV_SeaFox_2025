import numpy as np
import pandas as pd
import cv2
import pytesseract

# Configuración de Tesseract para reconocer solo Y y N
tess_config = r'-c tessedit_char_whitelist=YN --psm 10'

class ComputerModel:
    def __init__(self):
        print('start')

    def read_table(self, image_path: str) -> (pd.DataFrame, list):
        """
        Lee una imagen de tabla con marcas Y/N y devuelve:
          - df: DataFrame con años como índice y columnas Region 1...N
          - data: lista de listas con strings 'Y'/'N'
        """
        # 1. Cargar y convertir a escala de grises
        img = cv2.imread(image_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # 2. Binarizar (invertido) para resaltar líneas
        bw = cv2.adaptiveThreshold(
            ~gray, 255,
            cv2.ADAPTIVE_THRESH_MEAN_C,
            cv2.THRESH_BINARY,
            15, -2
        )

        # 3. Detectar líneas horizontales
        horiz = bw.copy()
        horiz_sz = horiz.shape[1] // 15
        horiz_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (horiz_sz, 1))
        horiz = cv2.erode(horiz, horiz_kernel)
        horiz = cv2.dilate(horiz, horiz_kernel)

        # 4. Detectar líneas verticales
        vert = bw.copy()
        vert_sz = vert.shape[0] // 15
        vert_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, vert_sz))
        vert = cv2.erode(vert, vert_kernel)
        vert = cv2.dilate(vert, vert_kernel)

        # 5. Intersecciones de líneas para obtener nodos de celdas
        intersect = cv2.bitwise_and(horiz, vert)
        pts = cv2.findNonZero(intersect)
        if pts is None:
            raise RuntimeError("No se detectaron intersecciones de la cuadrícula.")
        pts = pts.reshape(-1, 2)
        xs = np.unique(pts[:, 0]); ys = np.unique(pts[:, 1])
        xs.sort(); ys.sort()

        # Filtrar líneas principales para evitar duplicados
        def filter_lines(arr, thresh=15):
            filtered = []
            prev = -thresh * 2
            for v in arr:
                if v - prev > thresh:
                    filtered.append(v)
                    prev = v
            return filtered
        xs = filter_lines(xs)
        ys = filter_lines(ys)

        # 6. Extraer celdas (omitimos encabezados fila0 y col0)
        data = []
        for i in range(1, len(ys) - 1):
            row_cells = []
            y1, y2 = ys[i] + 2, ys[i+1] - 2
            for j in range(1, len(xs) - 1):
                x1, x2 = xs[j] + 2, xs[j+1] - 2
                cell_img = gray[y1:y2, x1:x2]
                if cell_img.size == 0:
                    row_cells.append('')
                else:
                    text = pytesseract.image_to_string(cell_img, config=tess_config)
                    text = text.strip().upper()
                    row_cells.append(text if text in ['Y', 'N'] else '')
            data.append(row_cells)

        # 7. Construir DataFrame con años e regiones
        num_rows = len(data)
        num_cols = len(data[0]) if num_rows > 0 else 0
        years = list(range(2016, 2016 + num_rows))
        columns = [f"Region {k}" for k in range(1, num_cols + 1)]
        df = pd.DataFrame(data, index=years, columns=columns)
        return df, data

    def bin(self, data: list) -> list:
        """
        Convierte una lista de listas con 'Y'/'N' a lista de listas con 1/0.
        """
        binary = []
        for row in data:
            bin_row = [1 if cell == 'Y' else 0 for cell in row]
            binary.append(bin_row)
        return binary

if __name__ == '__main__':
    path = "src/ros2_seafox_package/ros2_seafox_package/imgs/table.png"
    comp = ComputerModel()
    df, data = comp.read_table(path)
    binary_table = comp.bin(data)
    print("Y/N Table:")
    print(df)
    print("\nBinary Table:")
    print(binary_table)
