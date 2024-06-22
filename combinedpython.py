import threading
import time
import cv2
import cvzone
import math
import serial
from ultralytics import YOLO
import firebase_admin
from firebase_admin import credentials, db

first_time = True

# Serial object initiation
ser = serial.Serial('COM3', 9600, timeout=1)
time.sleep(2)

# Initialize Firebase Admin SDK
cred = credentials.Certificate("react-native-course-778b3-firebase-adminsdk-9n608-7b23871db3.json")  # Replace with your path
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://react-native-course-778b3-default-rtdb.firebaseio.com/'
})

# Global variables to store positions
X_pos, Y_pos, Z_pos = 0, 0, 0

# YOLO model and class names
model = YOLO("../YOLO-weights/newlibrarytrain.pt")
classNames = ['BIODEGRADABLE', 'CARDBOARD', 'METAL', 'PAPER', 'PLASTIC', 'cardboard']

def YOLO_processing(img):
    global first_time

    if first_time:
        first_time = False

    results = model(img, stream=True)
    max_conf = 0.75
    for r in results:
        boxes = r.boxes
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            w, h = x2 - x1, y2 - y1
            conf = (math.ceil(box.conf[0] * 100)) / 100
            cls = int(box.cls[0])
            if conf > 0.8:
                cvzone.cornerRect(img, (x1, y1, w, h))
                cvzone.putTextRect(img, f'{classNames[cls]} {conf}', (max(0, x1), max(35, y1)), scale=0.9, thickness=2)
                if conf >= max_conf:
                    max_conf = conf
                    center_of_box = x1 + (w // 2)
                    print(center_of_box)
                    cv2.line(img, (x1 + (w // 2), y1), (x1 + (w // 2), y1 + h), (255, 0, 0), 3)
                    if 340 < center_of_box <= 640:
                        cv2.putText(img, "box Right", (center_of_box + 100, y1 + (h // 2) + 50),
                                    cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
                        write_to_arduino(b'r')  # Sending 'r' for right
                        print("right")
                    elif 0 < center_of_box < 300:
                        cv2.putText(img, "box Left", (center_of_box + 100, y1 + (h // 2) + 50),
                                    cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
                        write_to_arduino(b'l')
                        print("left")
                    elif 300 < center_of_box < 340:
                        cv2.putText(img, "box center", (center_of_box + 100, y1 + (h // 2) + 50),
                                    cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
                        print("center")
                        write_to_arduino(b'c')

def handle_event(event):
    global X_pos, Y_pos, Z_pos, ser
    data_snapshot = ref.get()
    length = len(data_snapshot['Space'][0]['free'])
    X_pos = data_snapshot['Space'][0]['free'][length-1]['x']
    Y_pos = data_snapshot['Space'][0]['free'][length-1]['y']
    Z_pos = data_snapshot['Space'][0]['free'][length-1]['z']
    write_to_arduino(f"{X_pos},{Y_pos},{Z_pos}\n".encode())
    print(f'X pos: {X_pos}, Y pos: {Y_pos}, Z pos: {Z_pos}')

    robots = data_snapshot['Robots']
    for index, robot in enumerate(robots):
        condition = data_snapshot['Robots'][index]['condition']
        condition_ref = db.reference(f'/Robots/{index}/condition')
        now_handled = db.reference('/Orders/now/handled')
        if condition == 'f':
            X_pos_get = data_snapshot['Orders']['now'][0]['x']
            Y_pos_get = data_snapshot['Orders']['now'][0]['y']
            Z_pos_get = data_snapshot['Orders']['now'][0]['z']
            write_to_arduino(f"{X_pos_get},{Y_pos_get},{Z_pos_get}\n".encode())
            print("data sent to arduino")
            condition_ref.set('b')
            now_handled.set('True')
            print("condition and handled updated in database")

def database_comm():
    global ref
    ref=db.reference('/Warhouses/-NtDy2pwD9NWNPAK5Dgp')
    ref.listen(handle_event)

def read_from_arduino():
    global ser
    while True:
        if ser.in_waiting > 0:
            received_char = ser.read().decode('utf-8')
            if received_char == 'y':
                print("YOLO Processing initiated")
                while True:
                    success, img = cap.read()
                    if not success:
                        break
                    YOLO_processing(img)
                    cv2.imshow("output", img)
                    cv2.waitKey(1)
            elif received_char == 'd':
                print("Database Processing initiated")
                database_comm()

def write_to_arduino(data):
    write_thread = threading.Thread(target=_write_to_arduino, args=(data,))
    write_thread.start()

def _write_to_arduino(data):
    global ser
    ser.write(data)

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # for webcam
cap.set(3, 640)
cap.set(4, 480)
print("captured")

# Create and start the reading thread
read_thread = threading.Thread(target=read_from_arduino)
read_thread.start()

# print("before loop")
# while True:
#     success, img = cap.read()
#     print(success)
#     cv2.imshow("output", img)
#     cv2.waitKey(1)
