import cv2
import serial
import time
import math

face_cascade = cv2.CascadeClassifier(
    '/usr/local/Cellar/opencv/4.0.1/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml')

temp = 99999
centering = True
port = '/dev/cu.usbmodem14101'
# loop runs if capturing has been initialized.
start = False
def face_dete(ard):
    global temp
    global start
    cap = cv2.VideoCapture(1)
    while 1:
        ret, clone = cap.read()
        img = cv2.resize(clone, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_NEAREST)
        img = cv2.flip(img, 0)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        face = face_cascade.detectMultiScale(gray, 1.3, 5)
        for (x, y, w, h) in face:
                center_x = int((x + w * 0.5))
                center_y = int((y + h * 0.5))
                radius = int((h + w) * 0.25)
                cv2.circle(img, (center_x, center_y), radius, (255, 255, 0), 2);
                print(temp)
                if (temp > 100000):
                    dif_x = (480 - center_x)
                    temp = math.pow(dif_x, 2)
                    com = int(dif_x).__str__() + "&"
                    print(com)
                    if ard == 0:
                        print("fail to send")
                    else:
                        ard.write(com.encode());
                        time.sleep(2)  # I shortened this to match the new value in your Arduino code
                        msg = ard.read(ard.inWaiting())  # read everything in the input buffer
                        print(msg)
                else:
                    print("face centered goto next part")
                    com = "999&"
                    if ard == 0:
                        print("fail to send")
                    else:
                        ard.write(com.encode());
                        time.sleep(5)  # I shortened this to match the new value in your Arduino code
                        start = False
                        temp = 99999
                        print("close")
                        # Close the window
                        cap.release()
                        # De-allocate any associated memory usage
                        cv2.destroyAllWindows()
                        return

        # Display an image in a window
        cv2.imshow('img', img)

        # Wait for Esc key to stop
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break

if __name__ == '__main__':
    ard = serial.Serial(port, 9600, timeout=5)
    time.sleep(2)
    start = False

    while 1:
        while(start == False):
            msg_s = ard.read(ard.inWaiting())  # read everything in the input buffer
            time.sleep(2)
            print(msg_s)
            if (len(msg_s) == 5):
                start = True
                face_dete(ard)

    ard.close()

