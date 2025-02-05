"""Simple Reolink backup camera demo script.
"""

import cv2

# Open RTSP stream
capture = cv2.VideoCapture('rtsp://admin:@192.168.69.41:554/h264Preview_01_sub')

while (capture.isOpened()):
    ret, frame = cap.read()
    cv2.imshow('frame', frame)
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break

capture.release()

cv2.destroyAllWindows()
