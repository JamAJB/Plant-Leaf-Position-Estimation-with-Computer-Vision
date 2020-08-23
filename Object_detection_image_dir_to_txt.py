######## Picamera Object Detection Using Tensorflow Classifier #########
#
# Author: Evan Juras
# Date: 4/15/18
## Some of the code is copied from Google's example at
## https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb
## and some is copied from Dat Tran's example at
## https://github.com/datitran/object_detector_app/blob/master/object_detection_app.py

# Adapted by James Beadle to classify all images in a directory and save the output to a text file 
# Date: 10/5/20 (UK Date)
# this file is called by Robot_Control.py
## Code used to count the number of images in a directory is taken from this github page
## https://stackoverflow.com/questions/1320731/count-number-of-files-with-certain-extension-in-python

import os
import cv2
import time
import numpy as np
import tensorflow as tf
import sys


def classifyDirectory(imageDirectory):
    # python file must be in object_detection folder
    sys.path.append('..')
    from utils import label_map_util
    from utils import visualization_utils as vis_util
    # Name of model directory
    MODEL_NAME = 'inference_graph_ssd_mobilenet'   
    # current working directory
    DETECTION_PATH = os.path.dirname(os.path.abspath(__file__))
    # image directory
    imageDir = imageDirectory
    # Path to frozen detection graph
    PATH_TO_CKPT = os.path.join(DETECTION_PATH,MODEL_NAME,'frozen_inference_graph.pb')
    # Path to label map file
    PATH_TO_LABELS = os.path.join(DETECTION_PATH,'data','labelmap.pbtxt')
    # Number of classes the object detector can identify
    NUM_CLASSES = 1

    ## Load the label map.
    label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
    categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
    category_index = label_map_util.create_category_index(categories)
    # Load the Tensorflow model into memory.
    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
        sess = tf.Session(graph=detection_graph)

    # Define input and output tensors
    # Input tensor is the image
    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
    # Output tensors are the detection boxes, scores, classes, number
    detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
    detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
    detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
    num_detections = detection_graph.get_tensor_by_name('num_detections:0')
    
    file = open(imageDir + "/" + "classifications" + ".txt","w")
    fileData = []
    pictureCount = 0
    #find the number of images in the directory
    for root, dirs, files in os.walk(imageDir):
        for filename in files:    
            if filename.endswith('.jpg'):
                pictureCount += 1

    #it is assumed that all images are called 0.jpg, 1.jpg... as this is what Robot_Control.py saves them as
    for imagenum in range(pictureCount):
        fileData.append([])
        image = cv2.imread(imageDir + '/' + str(imagenum) + '.jpg')
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_expanded = np.expand_dims(image_rgb, axis=0)
        #perform the classifications
        (boxes, scores, classes, num) = sess.run(
            [detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: image_expanded})

        #save all detections with high scores
        classifications = []
        for i in range(len(scores[0])):
            if (scores[0][i] > 0.6):
                classifications.append(boxes[0][i].tolist())
        fileData[imagenum] = classifications
    file.writelines(str(fileData))
    file.close()
    return True

#if this program is to be run on its own    
if __name__ == "__main__":
    classifyDirectory(#Add directory of images#)

