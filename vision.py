from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import cv2 
import numpy as np

def process_vision_sensor_image(image, your_threshold_value=127, circularity_threshold_lower=0.8, circularity_threshold_upper=1.2):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, thresholded = cv2.threshold(gray, your_threshold_value, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(thresholded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        center = (int(x), int(y))
        radius = int(radius)
        cv2.circle(image, center, radius, (0, 255, 0), 2)

        area = cv2.contourArea(cnt)
        circularity = 4 * np.pi * (area / (radius * radius))

        if circularity_threshold_lower < circularity < circularity_threshold_upper:
            return True  # Cylinder is round
        else:
            return False  # Cylinder is defective

    return False  # No cylinder detected


client = RemoteAPIClient()
sim = client.require('sim')

# Replace 'your_vision_sensor_handle' with the actual handle of your vision sensor
#vision_sensor_handle = 'Vision_sensor'
vision_sensor_name = 'Vision_sensor'  # Name of your vision sensor in CoppeliaSim


sim.setStepping(True)

sim.startSimulation()
# while (t := sim.getSimulationTime()) < 3:
#     print(f'Simulation time: {t:.2f} [s]')
#     sim.step()

while (t := sim.getSimulationTime()) < 5:
    print(f'Simulation time: {t:.2f} [s]')
    # Get image data from vision sensor
    #vision_sensor_handle = sim.getObjectHandle(vision_sensor_name)
    vision_sensor_handle = sim.getObjectHandle(vision_sensor_name)
    print("Vision Sensor Handle:", vision_sensor_handle)
    
    imageBuffer = sim.getVisionSensorImage(vision_sensor_handle, 0, 0, 0, 0, 0)
    print("Size of imageBuffer:", len(imageBuffer))
    
    num_pixels = 196608 // 3  # Dividing by 3 channels for RGB
    height = int(num_pixels ** 0.5)  # Assuming a square image for simplicity
    width = num_pixels // height
    print("Possible Resolution:", width, height)

    resolution = [width, height]  # Replace 'width' and 'height' with the actual values

    try:
        # Attempt to reshape assuming an RGB image
        image = np.array(imageBuffer, dtype=np.uint8).reshape(height, width, 3)
        image = np.flip(image, 0)  # Flip the image vertically
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)  # Convert to BGR format for OpenCV
    except ValueError as e:
        print("Error reshaping RGB image:", e)
        image = None

    if image is not None:
        # Process the image
        is_cylinder_correct = process_vision_sensor_image(image)
        # Set signal in CoppeliaSim
        sim.setIntegerSignal('cylinder_status', 1 if is_cylinder_correct else 0)
        # Display the processed image
        cv2.imshow("Vision Sensor Image", image)
        #cv2.waitKey(1)

    # Step simulation
    sim.step()

cv2.destroyAllWindows()
sim.stopSimulation()

