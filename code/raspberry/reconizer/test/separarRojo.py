import cv2
import numpy as np

def rastrear_pilares():
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Definimos el "borrador mágico" una sola vez para ahorrar memoria
    kernel = np.ones((5,5), np.uint8)

    while True:
        ret, frame = cap.read()
        if not ret: break

        # 1. PERCEPCIÓN ÚNICA: Convertir a HSV (Lo hacemos solo 1 vez por fotograma)
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ==========================================
        # 2. CANAL ROJO
        # ==========================================
        rojo_bajo_1 = np.array([0, 50, 50])
        rojo_alto_1 = np.array([10, 255, 255])
        rojo_bajo_2 = np.array([170, 50, 50])
        rojo_alto_2 = np.array([179, 255, 255])

        mascara_r1 = cv2.inRange(hsv_frame, rojo_bajo_1, rojo_alto_1)
        mascara_r2 = cv2.inRange(hsv_frame, rojo_bajo_2, rojo_alto_2)
        mascara_roja = cv2.add(mascara_r1, mascara_r2)
        mascara_roja = cv2.morphologyEx(mascara_roja, cv2.MORPH_OPEN, kernel)

        # ==========================================
        # 3. CANAL VERDE
        # ==========================================
        # El verde en OpenCV está alrededor de Hue=60. (Rango seguro: 40 a 80)
        verde_bajo = np.array([40, 50, 50])
        verde_alto = np.array([80, 255, 255])
        
        mascara_verde = cv2.inRange(hsv_frame, verde_bajo, verde_alto)
        mascara_verde = cv2.morphologyEx(mascara_verde, cv2.MORPH_OPEN, kernel)

        # ==========================================
        # 4. FUNCIÓN INTERNA: Procesar Contornos
        # ==========================================
        def procesar_color(mascara, color_dibujo, nombre_objeto):
            contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Variables para guardar el objeto más grande encontrado
            max_area = 0
            mejor_centro_x = None

            for contorno in contornos:
                area = cv2.contourArea(contorno)
                # Solo procesamos si es grande y si es el MAYOR encontrado hasta ahora
                if area > 1000 and area > max_area:
                    max_area = area
                    x, y, w, h = cv2.boundingRect(contorno)
                    
                    # Dibujar Rectángulo y Punto Central
                    cv2.rectangle(frame, (x, y), (x + w, y + h), color_dibujo, 2)
                    mejor_centro_x = x + (w // 2)
                    centro_y = y + (h // 2)
                    cv2.circle(frame, (mejor_centro_x, centro_y), 5, color_dibujo, -1)
                    
                    # Etiqueta de texto
                    texto = f"{nombre_objeto} X:{mejor_centro_x}"
                    cv2.putText(frame, texto, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_dibujo, 2)
            
            return mejor_centro_x, max_area # Devolvemos datos clave para la lógica de conducción

        # ==========================================
        # 5. EJECUTAR BÚSQUEDA Y LÓGICA (El Cerebro)
        # ==========================================
        # Buscamos el rojo (dibujamos en BGR: Azul=0, Verde=0, Rojo=255)
        centro_rojo_x, area_roja = procesar_color(mascara_roja, (0, 0, 255), "PILAR ROJO")
        
        # Buscamos el verde (dibujamos en BGR: Azul=0, Verde=255, Rojo=0)
        centro_verde_x, area_verde = procesar_color(mascara_verde, (0, 255, 0), "PILAR VERDE")

        # --- Lógica Básica de Decisión ---
        if centro_rojo_x is not None:
            # Si vemos un pilar rojo, la WRO exige pasarlo por un lado específico (ej. dejarlo a la izquierda)
            print("Detectado ROJO. ¡Esquivando a la DERECHA!")
            # puerto.write(comando_girar_derecha) # (Futuro)
        elif centro_verde_x is not None:
            print("Detectado VERDE. ¡Esquivando a la IZQUIERDA!")
            # puerto.write(comando_girar_izquierda) # (Futuro)

        # Mostrar resultados
        cv2.imshow("Robot Vision", frame)
        
        # Para ver qué está pasando por dentro, mostramos ambas máscaras unidas temporalmente
        vista_debug = cv2.bitwise_or(mascara_roja, mascara_verde)
        cv2.imshow("Debug (Rojo + Verde)", vista_debug)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    rastrear_pilares()