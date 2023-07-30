# Object_detection_pkg

Library:
-	sudo apt-get install python3-pcl
-	sudo apt-get install python-opencv

Type of messages:
-	traffic_light_status: DepthAndColor custom_msg 
-	point_cloud_position_of_cars: PointCloud2

Nodes
-	rosrun object_detection_pkg cars_detection.py
-	rosrun object_detection_pkg traffic_light_status.py

Topics
-	rostopic echo traffic_light_status
-	rostopic echo point_cloud_position_of_cars


