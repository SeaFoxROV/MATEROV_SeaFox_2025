import cv2

# Open the default camera
cam = cv2.VideoCapture(8)

# Get the default frame width and height
frame_width = int(640)
frame_height = int(480)


while True:
    ret, frame = cam.read()



    # Display the captured frame
    cv2.imshow('Camera', frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break

# Release the capture and writer objects
cam.release()
cv2.destroyAllWindows()
