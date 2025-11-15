import cv2
print(cv2.getBuildInformation())

gst_pipeline = (
	"nvarguscamerasrc sensor_id=0"
	"video/x-raw(memory:NVMM), width=1920. height=1080, framerate=30/01 !"
	"nvvidconv !"
	"videoconvert ! "
	"video/x-raw, format=BGR ! appsink"
)
   
# Open a connection to the webcam (0 is the default camera)
cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

# Check if the webcam is opened correctly
if not cap.isOpened():
    print("Could not open webcam")
    exit()

# Read one frame from the webcam
ret, frame = cap.read()

# If the frame was captured successfully, ret will be True
if ret:
    # Save the captured frame as an image
    cv2.imwrite("captured_image.jpg", frame)
    print("Image saved as captured_image.jpg")
else:
    print("Failed to capture image")

# Release the webcam
cap.release()

# Close all OpenCV windows
cv2.destroyAllWindows()
