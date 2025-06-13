import tkinter as tk
import serial
import serial.tools.list_ports

import cv2
import mediapipe as mp
import time
import numpy as np

# --- Root Window ---
root = tk.Tk()
root.title("Control Mode Selector")
root.geometry("400x450")
root.resizable(False, False)

# --- Global State ---
arduino = None
selected_port = tk.StringVar()
is_connected = tk.BooleanVar(value=False)

# --- Status Label ---
status_label = tk.Label(root, text="Not Connected", fg="red", font=("Arial", 12))
status_label.pack(pady=5)

# --- List Arduino-like Ports ---
def list_arduino_ports():
    arduino_ports = []
    ports = serial.tools.list_ports.comports()

    for port in ports:
        if (
            "Arduino" in port.description
            or "CH340" in port.description
            or (port.vid == 0x2341)  # Arduino official VID
            or (port.vid == 0x1A86)  # CH340 clone VID
        ):
            arduino_ports.append(port.device)

    return arduino_ports

def refresh_ports():
    ports = list_arduino_ports()
    menu = com_dropdown["menu"]
    menu.delete(0, "end")
    for port in ports:
        menu.add_command(label=port, command=lambda p=port: selected_port.set(p))
    if ports:
        selected_port.set(ports[0])
    else:
        selected_port.set("")
        status_label.config(text="No Arduino ports found", fg="red")

# --- Connect / Disconnect ---
def connect_serial():
    global arduino
    port = selected_port.get()
    if not port:
        status_label.config(text="No port selected", fg="red")
        return
    try:
        arduino = serial.Serial(port, 115200, timeout=1)
        is_connected.set(True)
        status_label.config(text=f"Connected to {port}", fg="green")
        enable_mode_buttons()
        connect_btn.config(state="disabled")
        disconnect_btn.config(state="normal")
        print(f"[CONNECTED] {port}")
    except Exception as e:
        status_label.config(text="Connection Failed", fg="red")
        print("[ERROR] Serial connection failed:", e)

def disconnect_serial():
    global arduino
    if arduino and arduino.is_open:
        arduino.close()
        print("[DISCONNECTED]")
    is_connected.set(False)
    disable_mode_buttons()
    status_label.config(text="Disconnected", fg="red")
    connect_btn.config(state="normal")
    disconnect_btn.config(state="disabled")

# --- Mode Buttons ---
def gesture_command_A():
    disable_mode_buttons()
    print("Disabled Buttons")
    print("[MODE]: Gesture Up")
    root.after(10,lambda: gesture_control(True))
    print("Enabling Buttons")
    root.after(100, enable_mode_buttons)

def gesture_command_B():
    disable_mode_buttons()
    print("Disabled Buttons")
    print("[MODE]: Gesture Down")
    root.after(10,lambda: gesture_control(False))
    print("Enabling Buttons")
    root.after(100, enable_mode_buttons)

def signal_command_C():
    disable_mode_buttons()
    print("Disabled Buttons")
    print("[MODE]: Signal")
    root.after(10,lambda: signal1())
    print("Enabling Buttons")
    root.after(100, enable_mode_buttons)

def signal_command_D():
    disable_mode_buttons()
    print("Disabled Buttons")
    print("[MODE]: Signal")
    root.after(10,lambda: signal2())
    print("Enabling Buttons")
    root.after(100, enable_mode_buttons)

def signal_command_E():
    disable_mode_buttons()
    print("Disabled Buttons")
    print("[MODE]: Signal")
    root.after(10,lambda: signal3())
    print("Enabling Buttons")
    root.after(100, enable_mode_buttons)


def disable_mode_buttons():
    for btn in [btn1, btn2, btn3, btn4, btn5]:
        btn.config(state="disabled")

def enable_mode_buttons():
    for btn in [btn1, btn2, btn3, btn4, btn5]:
        btn.config(state="normal")

# ------- Gesture Control Functions -------------------

def finger_howopen(fingertipind, landmarks,MoveUp):
    # Check if finger is open

    # If checking thumb
    if fingertipind==4:
        tip2root=2
        k=2.8
        b=25
    else: # if checking other finger
        tip2root=3
        k=2
        b=10

    handroot = landmarks.landmark[0]
    fingertip = landmarks.landmark[fingertipind]
    fingerroot = landmarks.landmark[fingertipind-tip2root]

    ftLoc=np.array([fingertip.x, fingertip.y])
    frLoc=np.array([fingerroot.x, fingerroot.y])
    hrLoc=np.array([handroot.x, handroot.y])

    # Calculate distance vectors between wrist to knuckle and knuckle to fingertip
    vector1 = ftLoc - frLoc
    vector2 = frLoc - hrLoc

    uv1 = vector1/np.linalg.norm(vector2)
    uv2 = vector2/np.linalg.norm(vector2)
    # uv2 = vector2/np.linalg.norm(vector1)
    # Calculate the dot product of the vectors
    r_finger= np.dot(uv1, uv2)
    mid = 90
    amp=70
    max_angle= mid + amp

    angle=mid + amp*(0.5 - r_finger*0.5)*k - b

    if angle<mid:
        angle=mid
    elif angle>max_angle:
        angle= max_angle

    if MoveUp==True:
        return angle
    else:
        return 180-angle

def get_finger_states(hand_landmarks,tips,MoveUp):
    """
    Returns 0 if finger is open, 1 if closed, for each of the 5 fingers.
    """
    finger_states = []
    for i in range(0, 5):
        finger_states.append(finger_howopen(tips[i], hand_landmarks,MoveUp))

    return finger_states

def gesture_control(MoveUp):
    # MediaPipe setup
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(static_image_mode=False,
                        max_num_hands=2,
                        min_detection_confidence=0.7,
                        min_tracking_confidence=0.5)
    mp_drawing = mp.solutions.drawing_utils

    # Define finger tip indices
    FINGER_TIPS = [8, 12, 16, 20]
    THUMB_TIP = 4

    tips = [4, 8, 12, 16, 20]
    # Start video capture
    cap = cv2.VideoCapture(0)

    printdata=90
    old_finger_data = [90, 90, 90, 90, 90]  # Default if no hand
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # Flip and convert the image
        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = hands.process(rgb)

        finger_data = [90, 90, 90, 90, 90]  # Default if no hand
        

        if result.multi_hand_landmarks and result.multi_handedness:
            for hand_landmarks, handedness in zip(result.multi_hand_landmarks, result.multi_handedness):
                if handedness.classification[0].label == 'Right':
                    finger_data = get_finger_states(hand_landmarks,tips,MoveUp)
                    mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                    break  # Only process one right hand

        
        r=1
        finger_data=[np.round(finger_data[i]*r + old_finger_data[i]*(1-r),2) for i in range(len(finger_data))]
        old_finger_data=finger_data
        
        # Send finger data to Arduino
        try:
            arduino.write((','.join(map(str, finger_data)) + '\n').encode())
        except Exception as e:
            print("Serial write error:", e)
        
        print((','.join(map(str, [finger_data])) + '\n').encode())


        # Show the camera feed
        cv2.imshow("Hand Tracking", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC key to quit
            break

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()

def signal1():
    finger_data = [90, 90, 90, 90, 90] 
    for dif in [50, 0, -50, 0]:
        for fingerNum in range(0, 4):
            finger_data[fingerNum]=90+dif
            try:
                arduino.write((','.join(map(str, finger_data)) + '\n').encode())
            except Exception as e:
                print("Serial write error:", e)
            time.sleep(0.5)  # Allow time for the connection to establish

def signal2():
    finger_data = [90, 90, 90, 90, 90] 
    for dif in [50, 0, -50, 0]:
        for fingerNum in range(0, 4):
            finger_data[fingerNum]=90+dif
        try:
            arduino.write((','.join(map(str, finger_data)) + '\n').encode())
        except Exception as e:
            print("Serial write error:", e)
        time.sleep(2)  # Allow time for the connection to establish

def signal3():
    finger_data = [90, 90, 90, 90, 90] 
    for dif in [-45, 45, -45, 45, 0]:
        for fingerNum in range(0, 4):
            finger_data[fingerNum]=90+dif
            try:
                arduino.write((','.join(map(str, finger_data)) + '\n').encode())
            except Exception as e:
                print("Serial write error:", e)
            time.sleep(0.4)  # Allow time for the connection to establish


# --- COM Port Dropdown ---
tk.Label(root, text="Select COM Port:", font=("Arial", 12)).pack()
com_dropdown = tk.OptionMenu(root, selected_port, "")
com_dropdown.pack(pady=5)

# --- Connect / Disconnect Buttons ---
connect_btn = tk.Button(root, text="Connect", font=("Arial", 12), command=connect_serial)
connect_btn.pack(pady=5)

disconnect_btn = tk.Button(root, text="Disconnect", font=("Arial", 12), command=disconnect_serial, state="disabled")
disconnect_btn.pack(pady=5)

refresh_btn = tk.Button(root, text="Refresh Ports", font=("Arial", 10), command=refresh_ports)
refresh_btn.pack(pady=2)

# --- Mode Buttons ---
btn1 = tk.Button(root, text="Gesture Up (esc to close)", font=("Arial", 14), command=gesture_command_A)
btn1.pack(pady=5)

btn2 = tk.Button(root, text="Gesture Down (esc to close)", font=("Arial", 14), command=gesture_command_B)
btn2.pack(pady=5)

btn3 = tk.Button(root, text="Signal 1", font=("Arial", 14), command=signal_command_C)
btn3.pack(pady=5)

btn4 = tk.Button(root, text="Signal 2", font=("Arial", 14), command=signal_command_D)
btn4.pack(pady=5)

btn5 = tk.Button(root, text="Signal 3", font=("Arial", 14), command=signal_command_E)
btn5.pack(pady=5)

# --- Initial Button States ---
disable_mode_buttons()
refresh_ports()

root.mainloop()
