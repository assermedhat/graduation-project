from ultralytics import YOLO
import cv2
import cvzone
import math
import serial
import threading
import time
first_time = True

# ser = serial.Serial('COM8', 9600, timeout=1)   #serial object initiation


def YOLO_processing(img):
    global first_time
    # global img
    if first_time:
        first_time = False
        # thread = threading.Thread(target=firebase_processing)
        # thread.start()
        # print("thread started")
    results = model(img, stream=True)
    # loops across every frame to check BB and get its coordinates
    for r in results:
        boxes = r.boxes
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            w, h = x2 - x1, y2 - y1
            # print(x1,y1,w,h)
            # rectangle of opencv
            # cv2.rectangle(img,(x1,y1),(x2,y2),(0,255,0),3)
            # fancy reactangle of cvzone
            conf = (math.ceil(box.conf[0] * 100)) / 100
            max_conf=0.9
            cls = int(box.cls[0])
            # showing class name and conf on img
            if conf > 0.8:
                cvzone.cornerRect(img, (x1, y1, w, h))
                cvzone.putTextRect(img, f'{classNames[cls]} {conf}', (max(0, x1), max(35, y1)), scale=0.9, thickness=2)
            # showing classes name
                cls = box.cls[0]
            if conf >= max_conf:
                max_conf=conf
                center_of_box=x1 + (w // 2)
                print(center_of_box)
                cv2.line(img, (x1 + (w // 2), y1), (x1 + (w // 2), y1 + h), (255, 0, 0), 3)
                if (center_of_box) > 320 and (center_of_box) <= 640:
                    start_time = 0
                    cv2.putText(img, "box Right", (center_of_box + 100, y1 + (h // 2) + 50),
                                cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
                    # ser.write(b'R')  # Sending 'R' for right
                    print("right")
                elif (center_of_box) < 320 and (center_of_box) > 0:
                    cv2.putText(img, "box Left", (center_of_box + 100, y1 + (h // 2) + 50),
                                cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
                    # ser.write(b'L')
                    print("left")
                elif(center_of_box)==320:
                    cv2.putText(img, "box center", (center_of_box + 100, y1 + (h // 2) + 50),
                                cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
                    print("center")
                else:
                    print("stop")


                # getting confidence level of BB

                # showing classes name



# def firebase_processing():




# cap= cv2.VideoCapture("D:/codes/YOLOv8/chapter6-Webcam/cars.mp4") #for video
# cap= cv2.VideoCapture(0)
cap= cv2.VideoCapture(0,cv2.CAP_DSHOW) #for webcam
cap.set(3,640)
cap.set(4,480)
print("captured")
#creating the model with nano weights
model = YOLO("../YOLO-weights/newlibrarytrain.pt")


#coco dataset based on id number of detected object if id=0 then person detected if 1 bicycle and so on
classNames = ['BIODEGRADABLE', 'CARDBOARD', 'METAL', 'PAPER', 'PLASTIC', 'cardboard']


print("before loop")     
while True:
    global img
    success,img = cap.read()
    print(success)
    YOLO_processing(img)
    #getting the results and adding stream to true as it is more efficient

    print("before show")
    cv2.imshow("output",img)
    cv2.waitKey(1)
