# -*- coding: utf-8 -*-
import speech_recognition as sr

INF = 1000 # > 15 min
NULL_CHAR = "."
LOCATIONS = ["COCINA", "SALÓN", "BAÑO", "HABITACIÓN", "CUARTO"]

# Inicializar el reconocedor
recognizer = sr.Recognizer()
import cv2
import mediapipe as mp

result = 0
contador_tiempo = 0
#------------------------------------------------------------------------

import socket

HOST = "172.20.10.6" #"192.168.1.228"  #IP DEL SERVIDOR #HOST = "192.168.1.128"  #IP DEL SERVIDOR
PORT = 8003      #Puerto de envio => a que servidor nos vamos a conectar


# Utilizar el micrófono como fuente de audio
def listen():
    with sr.Microphone() as source:
        audio = recognizer.listen(source, timeout=INF, phrase_time_limit=4)

        try:
            # Usar Google Speech Recognition para transcribir el audio
            text = recognizer.recognize_google(audio, language='es-ES')
            print(text)
            return text
        except sr.UnknownValueError:
            # print("Google Speech Recognition no ha entendido lo que dijiste")
            return NULL_CHAR
        except sr.RequestError as e:
            #print("Error en la solicitud a Google Speech Recognition: {0}".format(e))
            return NULL_CHAR

if __name__ == "__main__": #main
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect((HOST, PORT)) #Se queda esperando...
    #una vez se establece la conexion se desbloquea

    while True:
        
        #Numero / estado enviado por el servidor => cliente maneja y decide
        tomadecision = int(client.recv(1024).decode()) #funcion blocking => se queda esperando a recibir una respuesta
        print(f"Respuesta enviada por el servidor: {tomadecision}")

        if tomadecision == 1: #speech
            op = -1
            print("Escuchando...")
            while op == -1:
                listened = listen()
                if (listened != NULL_CHAR):
                    listened = listened.upper()
                    for i in range(len(LOCATIONS)):
                        if LOCATIONS[i] in listened:
                            op = i
                            break
            op = str(op)
            client.send(op.encode())

        elif tomadecision == 2: #mediapipe
            mp_drawing = mp.solutions.drawing_utils #configuracion de mediapipe
            mp_hands = mp.solutions.hands

            cap = cv2.VideoCapture(0, cv2.CAP_DSHOW) #configuracion de la camara
            with mp_hands.Hands( #inicializar el modelo de mediapipe hands
                static_image_mode=False,
                max_num_hands=1,
                min_detection_confidence=0.5) as hands:

                while True:
                    ret, frame = cap.read()
                    if ret == False: #Si no se puede leer el frame por algun problema
                        break
                    
                    #Procesamiento de la imagen (frame)
                    height, width, _ = frame.shape
                    frame = cv2.flip(frame, 1)
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                    results = hands.process(frame_rgb)

                    #Dibujar landmarks y conexiones
                    if results.multi_hand_landmarks is not None:
                        for hand_landmarks in results.multi_hand_landmarks:
                            mp_drawing.draw_landmarks(
                                frame, hand_landmarks, mp_hands.HAND_CONNECTIONS,
                                mp_drawing.DrawingSpec(color=(0,0,255), thickness=3, circle_radius=5),
                                mp_drawing.DrawingSpec(color=(255,0,0), thickness=2, circle_radius=5))
                            
                            extremodedo = int(hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].y * height)
                            basededo = int(hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP].y * height)
                            diferencia = extremodedo - basededo

                            cv2.putText(frame, str(contador_tiempo), (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                            
                            # Verificar si el pulgar está señalando hacia arriba o hacia abajo
                            if diferencia < -40:
                                cv2.putText(frame, 'Pulgar hacia arriba', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                                contador_tiempo = contador_tiempo + 1
                                
                                if contador_tiempo > 80:
                                    result = str(1) #BIEN
                                    client.send(result.encode())
                                    cap.release()
                                    cv2.destroyAllWindows()
                                    result = 0
                                    contador_tiempo = 0
                                    break

                            elif diferencia > 40:
                                cv2.putText(frame, 'Pulgar hacia abajo', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                                contador_tiempo = contador_tiempo - 1
                                
                                if contador_tiempo < -80:
                                    result = str(2) #mal
                                    client.send(result.encode())
                                    cap.release()
                                    cv2.destroyAllWindows
                                    result = 0
                                    contador_tiempo = 0 
                                    break
                            else:
                                cv2.putText(frame, 'No detection', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (13, 127, 140), 2)
                                contador_tiempo = 0
                            
                    cv2.imshow('Frame',frame)
                    
                    if cv2.waitKey(1) & 0xFF == 27: #Si se pulsa la tecla ESC
                        break
            cap.release()
            cv2.destroyAllWindows()

#Bibliografia
#https://omes-va.com/mediapipe-hands-python/

#  Desktop\UNIVERSIDAD IV\ROBOTS MÓVILES\Proyecto
