import pyzed.sl as sl
import math
import numpy as np
import sys
import rospy
from std_msgs.msg import Float64

def main():
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use ULTRA depth mode
    init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use millimeter units for depth measurements

    # Open the camera
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:  # Ensure the camera has opened successfully
        print("Camera Open : " + repr(status) + ". Exit program.")
        exit()

    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = sl.RuntimeParameters()

    image = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()

    rospy.init_node("depth", anonymous=True)
    pub = rospy.Publisher("depth_sensor", Float64, queue_size=10)
    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            # A new image is available if grab() returns SUCCESS
            if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                # Retrieve left image
                zed.retrieve_image(image, sl.VIEW.LEFT)
                # Retrieve depth map. Depth is aligned on the left image
                zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
                # Retrieve colored point cloud. Point cloud is aligned on the left image
                zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

                # Get distance value in mm at the center of the image
                x = round(image.get_width() / 2)
                y = round(image.get_height() / 2)
                err, point_cloud_value = point_cloud.get_value(x, y)

                if err == sl.ERROR_CODE.SUCCESS:
                    if all(math.isfinite(coord) for coord in point_cloud_value[:3]):
                        distance = math.sqrt(
                            point_cloud_value[0]**2 +
                            point_cloud_value[1]**2 +
                            point_cloud_value[2]**2
                        )
                        pub.publish(Float64(distance))
            rate.sleep()
    finally:
        # Close the camera
        zed.close()

if __name__ == "__main__":
    main()
