import cv2
import numpy as np
import serial
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
pins = [2, 3,4,14]

for pin in pins:
    GPIO.setup(pin, GPIO.OUT)

def get_limits(color):
    c = np.uint8([[color]])
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

    lowerLimit = hsvC[0][0][0] - 5, 100, 100
    upperLimit = hsvC[0][0][0] + 5, 255, 255

    lowerLimit = np.array(lowerLimit, dtype=np.uint8)
    upperLimit = np.array(upperLimit, dtype=np.uint8)

    return lowerLimit, upperLimit

yellow_lower, yellow_upper = get_limits((255, 0, 0))  # BGR values for yellow

cap = cv2.VideoCapture(0)


time.sleep(3)

print("Serial OK")

# Print "Coordinate" once
print("Coordinate (x , y)")
#GPIO.output(14, GPIO.HIGH)

try:
    GPIO.output(2, GPIO.LOW)
    GPIO.output(4, GPIO.LOW)
    GPIO.output(14, GPIO.LOW)
    GPIO.output(3, GPIO.LOW)
    while True:
   
        ret, frame = cap.read()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, yellow_lower, yellow_upper)
        mask_ = cv2.bitwise_and(frame, frame, mask=mask)
        bbox = cv2.boundingRect(mask)
        if bbox[2] > 0 and bbox[3] > 0:
            x1, y1, w, h = bbox

            # Calculate the centroid of the bounding box
            cx = x1 + w / 2
            cy = y1 + h / 2

            # Determine quadrant based on centroid
            if cx < frame.shape[1] / 2 and cy < frame.shape[0] / 2:
                quadrant = "Top Left"
                GPIO.output(4, GPIO.LOW)
                GPIO.output(3, GPIO.LOW)
                GPIO.output(14, GPIO.LOW)
                GPIO.output(2, GPIO.HIGH)

                pin=2
                print(f"Turning on GPIO {pin}")
                time.sleep(0.01)

                print(f"{quadrant}")
           
               
            elif cx >= frame.shape[1] / 2 and cy < frame.shape[0] / 2:
                quadrant = "Top Right"
                GPIO.output(2, GPIO.LOW)
                GPIO.output(4, GPIO.LOW)
                GPIO.output(14, GPIO.LOW)
                GPIO.output(3, GPIO.HIGH)

                pin=3
                print(f"Turning on GPIO {pin}")
                time.sleep(0.01)
                print(f"{quadrant}")

                print(f"Turning off GPIO {pin}")
            elif cx < frame.shape[1] / 2 and cy >= frame.shape[0] / 2:
                quadrant = "Bottom Left"
                GPIO.output(2, GPIO.LOW)
                GPIO.output(3, GPIO.LOW)
                GPIO.output(14, GPIO.LOW)
                GPIO.output(4, GPIO.HIGH)

                pin=4
                print(f"Turning on GPIO {pin}")
                time.sleep(0.01)

                print(f"Turning off GPIO {pin}")
                print(f"{quadrant}")
           
            elif cx >= frame.shape[1] / 2 and cy >= frame.shape[0] / 2:
                quadrant = "Bottom right"
                GPIO.output(2, GPIO.LOW)
                GPIO.output(3, GPIO.LOW)
                GPIO.output(4, GPIO.LOW)
                GPIO.output(14, GPIO.HIGH)

                pin=14
                print(f"Turning on GPIO {pin}")
                time.sleep(0.01)

                print(f"Turning off GPIO {pin}")
                print(f"{quadrant}")
           
            else:
                quadrant = "No Signal ..."
                GPIO.output(2, GPIO.LOW)
                GPIO.output(3, GPIO.LOW)
                GPIO.output(4, GPIO.LOW)
                GPIO.output(14, GPIO.LOW)
         
                print(f"{quadrant}")

            # Print the updated coordinates and quadrant
            #print("\rCoordinate :({}, {}), Quadrant: {}".format(cx, cy, quadrant), end='', flush=True)

            # Draw a circle at the centroid for visualization with black color
            cv2.circle(frame, (int(cx), int(cy)), 3, (0, 0, 0), +1)

            # Draw bounding box around the detected object
            cv2.rectangle(frame, (x1, y1), (x1+w, y1+h), (0, 255, 255), 2)



        # Display the frame with bounding box and centroid
        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\nClosing serial communication.")
    ser.close()
    cap.release()
    cv2.destroyAllWindows()
