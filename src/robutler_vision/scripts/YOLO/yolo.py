#!/usr/bin/env python3

# import required packages
import cv2
import argparse
import numpy as np

class yolo:
    def __init__(self, config_file, weights_file, classes_file):
        self.model, self.net, self.classes = self.load_model(config_file, weights_file, classes_file)
        self.min_confidence = 0.45

    def load_model(self, config_file, weights_file, classes_file):
        print("[INFO] loading network...")

        # load the model from disk
        net = cv2.dnn.readNet(weights_file, config_file)

        # set neural network parameters
        model = cv2.dnn_DetectionModel(net)
        model.setInputParams(size=(416, 416), scale=1/255)

        # load the classes from disk
        with open(classes_file, 'r') as f:
            classes = [line.strip() for line in f.readlines()]

        print("[INFO] network loaded successfully...")
        return model, net, classes

    def detect(self, image):    
            print("[INFO] detecting objects...")
        
            # generate different colors for different classes 
            COLORS = np.random.uniform(0, 255, size=(len(self.classes), 3))
        
            # detect objects in the input image
            class_ids, confidences, boxes = self.model.detect(image, confThreshold=0.1, nmsThreshold=0.2)

            people = []

            # draw bounding box on the detected object with class name
            for (class_id, confidence, box) in zip(class_ids, confidences, boxes):
                if confidence < self.min_confidence:
                    continue
                class_id = class_id[0]
                if self.classes[class_id] == "person":
                    people.append((class_id, confidence, box))
                    continue
                label = "%s : %f" % (self.classes[class_id], confidence)
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                left, top, width, height = box
                top = max(top, labelSize[1])
                image = self.draw_bounding_box(image, class_id, confidence, left, top, left + width, top + height, COLORS, self.classes)
            
            CLOSE_THRESHOLD = 300
            for person in people:
                left, top, width, height = person[2]
                close_count = 1
                for person2 in people:
                    left2, top2, width2, height2 = person2[2]
                    if left == left2 and top == top2:
                        continue
                    if left2-CLOSE_THRESHOLD < left < left2+CLOSE_THRESHOLD \
                        and top2-CLOSE_THRESHOLD < top < top2+CLOSE_THRESHOLD:
                        close_count += 1
                if close_count < 3:
                    image = self.draw_bounding_box(image, person[0], person[1], left, top, left + width, top + height, COLORS, self.classes)
        
            return image

    # function to draw bounding box on the detected object with class name
    def draw_bounding_box(self, img, class_id, confidence, x, y, x_plus_w, y_plus_h, COLORS, label):

        label = "%s : %f" % (self.classes[class_id], confidence)

        color = COLORS[class_id]

        cv2.rectangle(img, (x,y), (x_plus_w,y_plus_h), color, 2)

        cv2.putText(img, label, (x-10,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        return img
