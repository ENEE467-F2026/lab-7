# A custom message type to store color and class labels for each object detected by the Yolo object detection routine
# each bbox is a list: center-x, center-y, width, height
# confidence score on (0,1) for each detection

ObjectUId object_uid
float32[4] bbox 
float32 conf 

