# Version 1.0
1. Finish basic object detection function with yolo_v5.
2. - Subscriber: ```/camera/color/image_raw```
   - Publisher : ```/object_detection/image_anchor```
3. Reference: [yolo_v5 github](https://github.com/ultralytics/yolov5),
              [coco-2017 class description](https://tech.amikelive.com/node-718/what-object-categories-labels-are-in-coco-dataset/)
### TODO
Solve the real-time problem when the task is heavy(print the stamp and get_time() to see)
***
# Version 1.1
### Change
1. Update ```opencv_vis``` flag for open and close the function of window visualization
2. Add word labels on the detection results
3. It can give the center of detected object as ```Marker``` now
4. Update README.md format   
5. Get intrinsic matrix and distortion parameters automatically
#### Subscriber
* /camera/color/image_raw
* /camera/aligned_depth_to_color/image_raw
* /camera/aligned_depth_to_color/camera_info 
#### Publisher
* /object_detection/image_anchor
* /object_detection/markers
### Tips
1. The topic ```/camera/aligned_depth_to_color/image_raw``` is published on the ```camera_color_optical_frame``` rather than the ```camera_aligned_depth_to_color_frame```
### TODO
Beautify the window visualization when ```opencv_vis==True```
***
# Version 1.2
### Change
1. Add some annotations for easier reading
2. Add parser for roslaunch using
3. Bias can be transfered from command line(left-top to right-bottom)
### TODO
Learn more about ROS service, the object_detection may be witten as service