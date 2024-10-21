import cv2
import numpy as np

# Load an image
image = cv2.imread("testImage.png")

# Setup BlobDetector
params = cv2.SimpleBlobDetector_Params()

# Filter by Area
params.filterByArea = True
params.minArea = 150
params.maxArea = 2000

# Filter by Circularity
params.filterByCircularity = True
params.minCircularity = 0.2

# Filter by Convexity
params.filterByConvexity = False

# Filter by Inertia
params.filterByInertia = True
params.minInertiaRatio = 0.8

# Distance Between Blobs
params.minDistBetweenBlobs = 10

# Create a detector with the parameters
detector = cv2.SimpleBlobDetector_create(params)

# Copy the image for overlay
overlay = image.copy()

# Detect blobs in the image
keypoints = detector.detect(image)

print(f"Number of keypoints detected: {len(keypoints)}")

# Print Coordinates
for i, k in enumerate(keypoints):
    print(f"Point {i+1} is at x = {k.pt[0]/2:.2f}; y = {k.pt[1]/2:.2f}")

# Draw detected blobs
for k in keypoints:
    cv2.circle(overlay, (int(k.pt[0]), int(k.pt[1])), int(k.size/2), (0, 0, 255), -1)
    cv2.line(overlay, (int(k.pt[0])-20, int(k.pt[1])), (int(k.pt[0])+20, int(k.pt[1])), (0,0,0), 3)
    cv2.line(overlay, (int(k.pt[0]), int(k.pt[1])-20), (int(k.pt[0]), int(k.pt[1])+20), (0,0,0), 3)

# Adjust opacity for the overlay
opacity = 0.5
cv2.addWeighted(overlay, opacity, image, 1 - opacity, 0, image)

# Resize the image to fit the window if needed
image = cv2.resize(image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_CUBIC)

# Show the result
cv2.imshow("Output", image)

# Wait for a key press to close the window
cv2.waitKey(0)

cv2.destroyAllWindows()
