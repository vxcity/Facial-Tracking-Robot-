import cv2
import serial
import time
import threading

# ---------- SETTINGS ----------
frame_width = 1920
frame_height = 1080
sensitivity = 25          # lower = more sensitive servo movement
arduino_port = '/dev/cu.usbmodem14101'  # adjust to your port
baud_rate = 9600        # match Arduino serial.begin()
use_arduino = True

# ---------- CAMERA ----------
video = cv2.VideoCapture(0, cv2.CAP_DSHOW)
video.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
video.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

# ---------- FACE DETECTOR ----------
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

# ---------- ARDUINO ----------
arduino = None

def try_connect():
    global arduino, use_arduino
    try:
        arduino = serial.Serial(arduino_port, baud_rate, timeout=1)
        time.sleep(2)
        print("Connected to Arduino on", arduino_port)
    except serial.serialutil.SerialException:
        print("Could not connect to Arduino. Running camera only.")
        use_arduino = False

try_connect()

# ---------- SERVO RESET ----------
def reset_position():
    if use_arduino:
        arduino.write(b"960 540\n")  # center
        print(" Reset servo to center") # my guess is that this is the reason it keeps moving 
        time.sleep(0.1)

reset_position()

# ---------- TRACKING LOOP ----------
center_x = frame_width // 2
center_y = frame_height // 2

while True:
    ret, frame = video.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(60, 60))

    if len(faces) > 0:
        # Pick the largest face (closest)
        (x, y, w, h) = max(faces, key=lambda f: f[2] * f[3])

        # Draw rectangle
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Calculate center of face
        face_x = x + w // 2
        face_y = y + h // 2

        # Send coordinates to Arduino
        if use_arduino:
            # Flip coordinates to match turret direction
            send_x = int(1920 - face_x)
            send_y = int(1080 - face_y)
            message = f"{send_x} {send_y}\n"
            arduino.write(message.encode())
            # small pause to prevent flooding
            time.sleep(0.05)
    else:
        # No face found — recenters slowly
        if use_arduino:
            arduino.write(b"960 540\n")

    # Display
    cv2.imshow("Face Tracker", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        reset_position()
        break

# ---------- CLEANUP ----------
video.release()
cv2.destroyAllWindows()
if use_arduino:
    arduino.close()
