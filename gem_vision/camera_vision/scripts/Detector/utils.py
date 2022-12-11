import cv2
import numpy as np
import argparse

# Draw a prediction box with confidence and title
def draw_prediction(frame, classes, classId, conf, left, top, right, bottom):
    # Draw a bounding box.
    cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0))
    # Assign confidence to label
    label = '%.2f' % conf
    # Print a label of class.
    if classes:
        assert(classId < len(classes))
        label = '%s: %s' % (classes[classId], label)
    labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    top = max(top, labelSize[1])
    cv2.rectangle(frame, (left, top - labelSize[1]), (left + labelSize[0], top + baseLine), (255, 255, 255), cv2.FILLED)
    cv2.putText(frame, label, (left, top), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
    return frame

# Process frame, eliminating boxes with low confidence scores and applying non-max suppression
def process_frame(frame, outs, classes, CONF_THRESHOLD, NMS_THRESHOLD, target): 
    detected_list = []
    # Get the width and height of the image
    frame_height = frame.shape[0]
    frame_width = frame.shape[1]
    # Network produces output blob with a shape NxC where N is a number of
    # detected objects and C is a number of classes + 4 where the first 4
    # numbers are [center_x, center_y, width, height]
    classIds = []
    confidences = []
    boxes = []
    bbx_frame = frame
    for out in outs:
        for detection in out:
            scores = detection[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]
            if confidence > CONF_THRESHOLD:
                # Scale the detected coordinates back to the frame's original width and height
                center_x = int(detection[0] * frame_width)
                center_y = int(detection[1] * frame_height)
                width = int(detection[2] * frame_width)
                height = int(detection[3] * frame_height)
                left = int(center_x - width / 2)
                top = int(center_y - height / 2)
                # Save the classId, confidence and bounding box for later use
                classIds.append(classId)
                confidences.append(float(confidence))
                boxes.append([left, top, width, height])
    # Apply non-max suppression
    indices = cv2.dnn.NMSBoxes(boxes, confidences, CONF_THRESHOLD, NMS_THRESHOLD)
    # CV2 4.2.0 's output is a 2-D list, convert it to 1-D lilst
    indices = [ind for index in indices for ind in index]
    for i in indices:
        confidence = confidences[i]
        box = boxes[i]
        left = box[0]
        top = box[1]
        width = box[2]
        height = box[3]
        # if  classes[classIds[i]] == target:
        detected_list.append([left, top, left + width, top + height, classIds[i], confidence])
        bbx_frame = draw_prediction(frame, classes, classIds[i], confidences[i], left, top, left + width, top + height)
    return detected_list, bbx_frame