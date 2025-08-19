import cv2, os
import time, sys

try:
    label = sys.argv[1]
except IndexError:
    print("Please Include a Label after the filename")

# Initialize the webcam (0 is the default camera)
cap = cv2.VideoCapture(0)

# Check if the webcam is opened correctly
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Capture the stream for 5 seconds
start_time = time.time()
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    if not ret:
        print("Error: Failed to capture image.")
        break
    
    # Display the resulting frame
    cv2.imshow('Webcam Stream', frame)
    
    # Check if 5 seconds have passed
    if time.time() - start_time > 3:
        # Save the current frame as an image
        path = os.path.join(os.getcwd(),"hand_signs", label)
        if not os.path.isdir(path):
            os.makedirs(path)
        files = os.listdir(path)
        i = 0
        exists = os.path.exists(os.path.join(path,f"{i}.jpeg"))
        while exists:
            i += 1
            exists = os.path.exists(os.path.join(path,f"{i}.jpeg"))
        cv2.imwrite(os.path.join(path, f"{i}.jpeg"), frame)
        print(f"Image saved as {i}.jpeg.")
        start_time = time.time()

    
    # Exit the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture object and close the display window
cap.release()
cv2.destroyAllWindows()