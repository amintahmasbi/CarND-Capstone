from styx_msgs.msg import TrafficLight

import rospy
import tensorflow as tf
import numpy as np
import cv2
import os
import sys
import tarfile
import zipfile
import six.moves.urllib as urllib


class TLClassifier(object):
    # Frozen inference graph files. NOTE: change the path to where you saved the models.
    SSD_GRAPH_FILE = 'ssd_mobilenet_v1_coco_11_06_2017'
    SSD_V2_GRAPH_FILE = 'ssd_inception_v2_coco_11_06_2017'
    RFCN_GRAPH_FILE = 'rfcn_resnet101_coco_11_06_2017'
    FASTER_RCNN_GRAPH_FILE = 'faster_rcnn_inception_resnet_v2_atrous_coco_11_06_2017'
    NUM_CLASSES = 90 # 'traffic light' is No. 10
    LOWER_RED = [100, 15, 17] #RGB
    UPPER_RED = [255, 56, 50] #RGB

    def __init__(self):
        # Load classifier
        # This is needed since the model is stored in the current folder.
        sys.path.append(".")
        # print(sys.path)

        PATH_TO_CKPT = self.DownloadFrozenModel()

        detection_graph = self.load_graph(PATH_TO_CKPT)

        # The input placeholder for the image.
        # `get_tensor_by_name` returns the Tensor with the associated name in the Graph.
        self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')

        # The classification of the object (integer id).
        self.detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

        self.sess = tf.Session(graph=detection_graph)


    def DownloadFrozenModel(self):
        # What model to download.
        MODEL_NAME = self.SSD_V2_GRAPH_FILE
        MODEL_FILE = MODEL_NAME + '.tar.gz'
        DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/'
        GRAPH_FILE = 'frozen_inference_graph.pb'
        # Path to frozen detection graph. This is the actual model that is used for the object detection.
        PATH_TO_CKPT = os.path.join('.','light_classification', MODEL_NAME, GRAPH_FILE)

        # List of the strings that is used to add correct label for each box.
        # PATH_TO_LABELS = os.path.join('.','light_classification','labels', 'mscoco_label_map.pbtxt')

        if not os.path.exists(PATH_TO_CKPT):
            rospy.logwarn('Downloading the frozen graph from tensorflow website... (~500 MBs)')
            opener = urllib.request.URLopener()
            opener.retrieve(DOWNLOAD_BASE + MODEL_FILE, MODEL_FILE)
            tar_file = tarfile.open(MODEL_FILE)
            for file in tar_file.getmembers():
                file_name = os.path.basename(file.name)
                if GRAPH_FILE in file_name:
                    tar_file.extract(file, os.getcwd())
        # else :
            # rospy.logwarn('Found graph file')
        return PATH_TO_CKPT


    #
    # Utility funcs
    #

    def filter_boxes(self, min_score, boxes, scores, classes):
        """Return boxes with a confidence >= `min_score`"""
        n = len(classes)
        idxs = []
        for i in range(n):
            if scores[i] >= min_score:
                idxs.append(i)

        filtered_boxes = boxes[idxs, ...]
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]
        return filtered_boxes, filtered_scores, filtered_classes

    def to_image_coords(self, boxes, height, width):
        """
        The original box coordinate output is normalized, i.e [0, 1].

        This converts it back to the original coordinate based on the image
        size.
        """
        box_coords = np.zeros_like(boxes)
        box_coords[:, 0] = boxes[:, 0] * height
        box_coords[:, 1] = boxes[:, 1] * width
        box_coords[:, 2] = boxes[:, 2] * height
        box_coords[:, 3] = boxes[:, 3] * width

        return box_coords

    def load_graph(self, graph_file):
        """Loads a frozen inference graph"""
        graph = tf.Graph()
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        return graph

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Implement light color prediction
        # Actual detection.
        image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)

        image_np = np.expand_dims(np.asarray(image, dtype=np.uint8), 0)

        (boxes, scores, classes) = self.sess.run([self.detection_boxes,
            self.detection_scores, self.detection_classes],
            feed_dict={self.image_tensor: image_np})

        # Remove unnecessary dimensions
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes)

        confidence_cutoff = 0.8
        # Filter boxes with a confidence score less than `confidence_cutoff`
        boxes, scores, classes = self.filter_boxes(confidence_cutoff, boxes, scores, classes)

        # The current box coordinates are normalized to a range between 0 and 1.
        # This converts the coordinates actual location on the image.
        height, width = image_np.shape[1:3] # Note: numpy height and weidth are inverse compared to opencv
        # rospy.logdebug(width)
        box_coords = self.to_image_coords(boxes, height, width)

        color_state = TrafficLight.UNKNOWN
        # masked_image = np.squeeze(image_np)

        for idx, detected_item in enumerate(classes):
            if detected_item == 10:
                box = box_coords[idx, :] # ymin, xmin, ymax, xmax = box
                box = [int(x) for x in box]
                sqeezed_image = np.squeeze(image_np)
                # rospy.logdebug(box)
                cropped_image = sqeezed_image[ box[0]:box[2], box[1]:box[3], :]
                # color_state, masked_image = self.get_color(cropped_image)
                color_state = self.get_color(cropped_image)
                # rospy.logdebug(color_state)

        # return color_state, masked_image
        return color_state

    def get_color(self, image):
        """Identify current color of Traffic Light

        :image (numpy array):  image contating traffic light
        :returns: ID of traffic light color

        """
        # TODO: HSV implementation for better detection
        # hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        # create NumPy arrays from the boundaries
        # lower = np.expand_dims(np.expand_dims(np.asarray(self.LOWER_RED, dtype=np.uint8), 0), 0)
        # upper = np.expand_dims(np.expand_dims(np.asarray(self.UPPER_RED, dtype=np.uint8), 0), 0)

        lower = np.asarray(self.LOWER_RED, dtype=np.uint8)
        upper = np.asarray(self.UPPER_RED, dtype=np.uint8)

        # lower_hsv = cv2.cvtColor(lower, cv2.COLOR_RGB2HSV)
        # upper_hsv = cv2.cvtColor(upper, cv2.COLOR_RGB2HSV)
        # find the colors within the specified boundaries and apply
        # the mask
        # mask = cv2.inRange(hsv, np.squeeze(lower_hsv), np.squeeze(upper_hsv))
        mask = cv2.inRange(image, lower, upper)

        # rospy.logwarn(np.sum(mask))
        masked_image = cv2.bitwise_and(image, image, mask = mask)

        if np.sum(mask) < 20:
            # return TrafficLight.UNKNOWN, masked_image
            return TrafficLight.UNKNOWN
        else:
            # return TrafficLight.RED, masked_image
            return TrafficLight.RED

    # def uniform_image(self, image):
        # return all(value == image[0] for value in image)
