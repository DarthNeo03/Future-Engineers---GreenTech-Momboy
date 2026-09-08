import cv2

def probar_camara(indice=0):
    # En Windows puede ayudar usar cv2.CAP_DSHOW en vez de solo el índice
    cap = cv2.VideoCapture(indice)

    if not cap.isOpened():
        print(f"No se pudo abrir la cámara con índice {indice}")
        return

    # Opcional: forzar resolución
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print("Cámara abierta. Presiona 'q' para salir, 's' para guardar una foto.")

    contador_fotos = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            print("No se pudo leer el frame. Saliendo...")
            break

        cv2.imshow("Prueba de camara USB", frame)

        tecla = cv2.waitKey(1) & 0xFF
        if tecla == ord('q'):
            break
        elif tecla == ord('s'):
            nombre = f"captura_{contador_fotos}.png"
            cv2.imwrite(nombre, frame)
            print(f"Foto guardada: {nombre}")
            contador_fotos += 1

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    probar_camara(0)  # Cambia el índice si tienes varias cámaras (0, 1, 2...)