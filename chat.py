from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import cv2 
import numpy as np

def process_vision_sensor_image(image, your_threshold_value=127, circularity_threshold_lower=0.8, circularity_threshold_upper=1.2):
    # Image processing steps as you defined
    # ...

client = RemoteAPIClient()
sim = client.require('sim')

vision_sensor_name = 'Vision_sensor'  # Name of your vision sensor in CoppeliaSim

sim.setStepping(True)
sim.startSimulation()

while True:
    # Get the handle of the vision sensor
    vision_sensor_handle = sim.getObject(vision_sensor_name)

    # Get image data from vision sensor
    retCode, resolution, image = sim.getVisionSensorImage(vision_sensor_handle, False)

    if retCode == 0:  # Check for success
        # Convert the image from CoppeliaSim's format to a format OpenCV can use
        image = np.array(image, dtype=np.uint8)
        image.resize([resolution[1], resolution[0], 3])
        image = np.flip(image, 0)  # Vertical flip
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # Process the image
        is_cylinder_correct = process_vision_sensor_image(image)

        # Set signal in CoppeliaSim
        sim.setIntegerSignal('cylinder_status', 1 if is_cylinder_correct else 0)

        # Optionally, display the processed image
        cv2.imshow("Vision Sensor Image", image)
        cv2.waitKey(1)

    # Step simulation
    sim.step()

cv2.destroyAllWindows()
sim.stopSimulation()
