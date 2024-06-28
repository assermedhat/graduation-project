import threading
import time
import cv2
import cvzone
import math
import serial
from ultralytics import YOLO
import firebase_admin
from firebase_admin import credentials, db

# Shared variables and synchronization primitives
received_char = None
lock = threading.Lock()
stop_yolo = threading.Event()

# Function to read from serial in a separate thread
def read_serial():
    global received_char
    while True:
        char = ser.read().decode('utf-8')
        with lock:
            received_char = char
        if char == 'd':
            stop_yolo.set()  # Signal YOLO processing to stop
            break

first_time = True

# Serial object initiation
ser = serial.Serial('COM8', 9600, timeout=1)
time.sleep(2)

# Initialize Firebase Admin SDK
cred = credentials.Certificate("react-native-course-778b3-firebase-adminsdk-9n608-7b23871db3.json")  # Replace with your path
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://react-native-course-778b3-default-rtdb.firebaseio.com/'
})

# Global variables to store positions
X_pos, Y_pos, Z_pos = 0, 0, 0
done=1


# YOLO model and class names
model = YOLO("../YOLO-weights/newlibrarytrain.pt")
classNames = ['BIODEGRADABLE', 'CARDBOARD', 'METAL', 'PAPER', 'PLASTIC', 'cardboard']

def YOLO_processing():
    global first_time
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # for webcam
    cap.set(3, 640)
    cap.set(4, 480)
    print("captured")

    while not stop_yolo.is_set():
        success, img = cap.read()
        if not success:
            break

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
        cv2.imshow("output", img)
        cv2.waitKey(1)

    cap.release()
    cv2.destroyAllWindows()

def handle_event(event):
    global X_pos, Y_pos, Z_pos, ser
    data_snapshot = ref.get()

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
            print("order spaces data sent to arduino")
            condition_ref.set('b')
            now_handled.set('True')
            print("condition and handled updated in database")

def free_spaces(event):
    global X_pos, Y_pos, Z_pos, ser
    global done
    data_snapshot = ref.get()
    global done
    pick = db.reference('/Warhouses/-NtDy2pwD9NWNPAK5Dgp/Pick/')
    if (done == 1):
        done = 0
        done_flag = db.reference('/Warhouses/-NtDy2pwD9NWNPAK5Dgp/Pick/done')
        # done=data_snapshot['Pick']['done']
        length = len(data_snapshot['Space'][0]['free'])
        print("free spaces=", length)A
        # there will be a condition here if(broughtin==true) it will send these positions else it will check if the robot is free to make the orders
        X_pos = data_snapshot['Pick']['x']
        Y_pos = data_snapshot['Pick']['y']
        Z_pos = data_snapshot['Pick']['z']
        write_to_arduino(f"{X_pos},{Y_pos},{Z_pos}\n".encode())
        print("data sent to arduino")
        done_flag.set(True)
        print('X pos', X_pos)
        print('Y pos', Y_pos)
        print('Z pos', Z_pos)
def database_comm():
    global ref
    ref = db.reference('/Warhouses/-NtDy2pwD9NWNPAK5Dgp')
    ref.listen(handle_event)

def database_free():
    global ref
    ref = db.reference('/Warhouses/-NtDy2pwD9NWNPAK5Dgp')
    ref.listen(free_spaces)

def read_from_arduino():
    global ser
    while True:
        if ser.in_waiting > 0:
            received_char = ser.read().decode('utf-8')
            print(f"Received: {received_char}")
            if received_char == 'y':
                print("YOLO Processing initiated")
                stop_yolo.clear()  # Ensure the YOLO thread will run
                yolo_thread = threading.Thread(target=YOLO_processing)
                yolo_thread.start()
            elif received_char == 'd':
                print("pickup  Processing initiated")
                database_free()
            elif received_char == 'f':
                print("order processing initiated")
                database_comm()
            else:
                print("no character sent")

def write_to_arduino(data):
    with lock:
        ser.write(data)

# Create and start the reading thread
read_thread = threading.Thread(target=read_from_arduino)
read_thread.start()
