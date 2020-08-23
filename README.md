# Plant-Leaf-Position-Estimation-with-Computer-Vision
This repository contains code described in the article 'Plant Leaf Position Estimation with Computer Vision', pending review.


*Robot_Control.py* is a python program designed to be run on a Raspberry Pi connected to a USB camera and Arduino. The code sends positional commands to the Arduino, which controls the motors of a linear three-axis robot. The USB Camera is connected to the robots arm. the code follows the processes below.

1. Calibrate the robot by moving each axis until a limit switch is pressed
2. Move the cameras position to seven different positions over a plant
3. Save an image at each location, and call on code to run an object detection neural network
4. group each detection from the neural network by origin leaf
5. convert the position of each leaf in every image into an estimated 3D position
6. Move the robot arm to any predicted positions
7. Reset

*Object_detection_image_dir_to_txt.py* is a python program which is called by *Robot_Control.py*, and runs a directory of images through the neural network "ssd_mobilenet_v2_quantized_coco". Any detections are saved to a text file. The neural network was trained following this Tutorial: https://github.com/EdjeElectronics/TensorFlow-Object-Detection-API-Tutorial-Train-Multiple-Objects-Windows-10. The trained network was then transferred to the Raspberry Pi using this tutorial: https://github.com/EdjeElectronics/TensorFlow-Object-Detection-on-the-Raspberry-Pi . *Object_detection_image_dir_to_txt.py* is an adaptation of the code *Object_detection_picamera.py* written by https://github.com/EdjeElectronics. Adaptations were made so the code classifies all images in a specified directory, and all outputs are saved to a text file.

*Motor_Control* is an Arduino code which controls the motion of three stepper motors. Position commands are received over serial, which the Arduino interprets and executes. Every motion a motor makes is set to have safe acceleration and deceleration. The Arduino returns commands over serial to indicate when a command has been received, completed, or failed. a failed command would be caused by a limit switch being pressed.

*labelmap.pbtxt* is the label map of the neural network. The network is only trained to recognise Cowpea (Vigna unguiculata) leaves.

*inference_graph_ssd_mobilenet* contains the saved inference graph trained with the Tensorflow API which was used to detect leaves.

*Test_Data.csv* includes the actual and predicted positions of 30 different detections the robot made in millimetres. Conclusions made from this have been outlined in the article.
