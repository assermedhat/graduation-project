import firebase_admin
from firebase_admin import credentials, db
import serial
import time

#create serial object
arduino=serial.Serial('COM8',9600)
time.sleep(2)
# Initialize Firebase Admin SDK
cred = credentials.Certificate("react-native-course-778b3-firebase-adminsdk-9n608-7b23871db3.json")  # Replace with your path
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://react-native-course-778b3-default-rtdb.firebaseio.com/'
})

# Function to handle updates
def handle_event(event):
    print('Event:', event.event_type)  # The type of event (put, patch, delete)
    print('Path:', event.path)  # The path to the affected data
    print('Data:', event.data)  # The data that was changed
    data_snapshot = ref.get()
    length=len(data_snapshot['Space'][0]['free'])
    print("free spaces=",length)
    #there will be a condition here if(broughtin==true) it will send these positions else it will check if the robot is free to make the orders
    X_pos=data_snapshot['Space'][0]['free'][length-1]['x']
    Y_pos=data_snapshot['Space'][0]['free'][length-1]['y']
    Z_pos=data_snapshot['Space'][0]['free'][length-1]['z']
    arduino.write(f"{X_pos},{Y_pos},{Z_pos}\n".encode())
    print('X pos', X_pos)
    print('Y pos',Y_pos )
    print('Z pos',Z_pos )

    # num_robots = len(robots)
    robots = data_snapshot['Robots']
    for index, robot in enumerate(robots):
        # Do something with the index and the robot
        print(f"Robot at index {index}: {robot}")
        condition=data_snapshot['Robots'][index]['condition']
        condition_ref = db.reference('/Robots/condition')
        now_handled=db.reference('/Orders/now/handled')
        current_condition = condition_ref.get()
        if (condition=='f'):
            X_pos_get=data_snapshot['Orders']['now'][0]['x']
            Y_pos_get = data_snapshot['Orders']['now'][0]['y']
            Z_pos_get = data_snapshot['Orders']['now'][0]['z']
            arduino.write(f"{X_pos_get},{Y_pos_get},{Z_pos_get}\n".encode())
            print("data sent to arduino")
            condition_ref.set('b')
            now_handled.set('True')
            print("condition and handled updated in database")
            print("x to get from inventory=",X_pos_get)
            print("y to get from inventory=", Y_pos_get)
            print("z to get from inventory=", Z_pos_get)
            pass


# Listen for updates
ref = db.reference('/Warhouses/-NtDy2pwD9NWNPAK5Dgp')  # Replace with your database path
ref.listen(handle_event)
