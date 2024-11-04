import cv2
import numpy as np

# Load an image
image = cv2.imread("testPhotos/centre1.png")

# Get the dimensions of the original image
original_height, original_width = image.shape[:2]
center_x, center_y = original_width / 2, original_height / 2
print(f"Width and Height are: {original_width}, {original_height}")    

# Setup BlobDetector
params = cv2.SimpleBlobDetector_Params()

# Filter by Area
params.filterByArea = True
params.minArea = 10
params.maxArea = 1000

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

# Calculate the distance to the center and find the closest blob
closest_keypoint = None
min_distance = float('inf')

for i, k in enumerate(keypoints):
    # Calculate distance from the center
    distance = np.sqrt((k.pt[0] - center_x) ** 2 + (k.pt[1] - center_y) ** 2)
    print(f"Point {i+1} is at x = {k.pt[0]:.2f}, y = {k.pt[1]:.2f}, Distance to center: {distance:.2f}")

    # Update closest point if distance is smaller
    if distance < min_distance:
        min_distance = distance
        closest_keypoint = k

# Draw the closest blob if found
if closest_keypoint:
    cv2.circle(overlay, (int(closest_keypoint.pt[0]), int(closest_keypoint.pt[1])), int(closest_keypoint.size / 2), (0, 0, 255), -1)
    cv2.line(overlay, (int(closest_keypoint.pt[0]) - 20, int(closest_keypoint.pt[1])), (int(closest_keypoint.pt[0]) + 20, int(closest_keypoint.pt[1])), (0, 0, 0), 3)
    cv2.line(overlay, (int(closest_keypoint.pt[0]), int(closest_keypoint.pt[1]) - 20), (int(closest_keypoint.pt[0]), int(closest_keypoint.pt[1]) + 20), (0, 0, 0), 3)

    print(f"Closest point is at x = {closest_keypoint.pt[0]:.2f}, y = {closest_keypoint.pt[1]:.2f}, Distance to center: {min_distance:.2f}")

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