import cv2
import pywhatkit
import time
import urllib.request
import numpy as np
import serial
from simple_facerec import SimpleFacerec

# Init
sfr = SimpleFacerec()
sfr.load_encoding_images("images/")

url = 'http://192.168.90.125:8000/video'
stream = urllib.request.urlopen(url)
bytes_buffer = b''

ser = serial.Serial('COM6', 9600)

unknown_detected = False
start_time = None
message_sent = False
servo_activated = False

def send_alert_message():
    try:
        pywhatkit.sendwhatmsg_instantly("+21699733400", "Alerte : Visage inconnu détecté !")
        print("Message WhatsApp envoyée.")
    except Exception as e:
        print(f"Erreur WhatsApp : {e}")

while True:
    bytes_buffer += stream.read(1024)
    a = bytes_buffer.find(b'\xff\xd8')
    b = bytes_buffer.find(b'\xff\xd9')

    if a != -1 and b != -1:
        jpg = bytes_buffer[a:b+2]
        bytes_buffer = bytes_buffer[b+2:]
        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

        if frame is None:
            continue

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        face_locations, face_names = sfr.detect_known_faces(frame_rgb)

        if 'Unknown' in face_names:
            if not unknown_detected:
                print("Visage inconnu détecté.")
                unknown_detected = True
                start_time = time.time()
                message_sent = False
                ser.write(b'1')  # Active le buzzer clignotant
            else:
                if (time.time() - start_time >= 5) and not message_sent:
                    send_alert_message()
                    message_sent = True
        else:
            if face_names:
                if not servo_activated:
                    print("Visage reconnu, ouverture de la porte.")
                    ser.write(b'S')  # Commande pour le servo
                    servo_activated = True
            unknown_detected = False
            start_time = None
            message_sent = False
            ser.write(b'0')  # Désactiver le buzzer

        for face_loc, name in zip(face_locations, face_names):
            y1, x2, y2, x1 = face_loc
            cv2.putText(frame, name, (x1, y1 - 10), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 200), 2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 200), 4)

        cv2.imshow("Raspberry Pi Camera Feed", frame)

    key = cv2.waitKey(1)
    if key == 32:  # Espace
        print("Arrêt manuel du buzzer.")
        ser.write(b'0')
        unknown_detected = False
        start_time = None
        message_sent = False

    if key == 27:  # Échap
        print("Fermeture du programme.")
        ser.write(b'0')
        break

cv2.destroyAllWindows()
