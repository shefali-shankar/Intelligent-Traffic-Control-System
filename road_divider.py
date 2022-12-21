
# import the necessary packages
import numpy as np
import argparse
import imutils
import serial
import time
import cv2
import os
import cvlib as cv
from cvlib.object_detection import draw_bbox
from PIL import Image
ap = argparse.ArgumentParser()
ap.add_argument("-c", "--confidence", type=float, default=0.5,
	help="minimum probability to filter weak detections")
ap.add_argument("-t", "--threshold", type=float, default=0.3,
	help="threshold when applyong non-maxima suppression")
args = vars(ap.parse_args())

# load the COCO class labels (80 different class labels) our YOLO model was trained on
labelsPath = os.path.sep.join(["object-coco\coco.names"])
LABELS = open(labelsPath).read().strip().split("\n")
print(len(LABELS))
print(LABELS)
# initialize a list of colors to represent each possible class label
np.random.seed(42)
COLORS = np.random.randint(0, 255, size=(len(LABELS), 3),
	dtype="uint8")

data = serial.Serial(
  
   port='COM3',  #ttyAMA0 ttyS0
   baudrate = 9600, #9000 bits per sec
   parity=serial.PARITY_NONE,
   stopbits=serial.STOPBITS_ONE,
   bytesize=serial.EIGHTBITS,
   timeout=1
)

print("Loading ...................")
net = cv2.dnn.readNetFromDarknet('object-coco\yolov3.cfg', 'object-coco\yolov3.weights')
print(net)
ln = net.getLayerNames() #YOLO neural network layer(255 components)
print(ln)
print(len(ln))#254 - components, 106 conv layers5
ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()] #indices of output layer to find how far network must feedforward
print(ln) # 3 output layers - ['yolo_82', 'yolo_94', 'yolo_106']

#vs = cv2.VideoCapture('C:\\Users\\shefa\\Desktop\\Vehicle Detection\\videos\\overpass.mp4')
#vs = cv2.VideoCapture('C:\\Users\\shefa\\Desktop\\Vehicle Detection\\Vehicle Detection\\videos\\car2.jpeg')
#vs= cv2.VideoCapture(0)
writer = None
(W, H) = (None, None) #Width,Height

try:
	prop = cv2.cv.CV_CAP_PROP_FRAME_COUNT if imutils.is_cv2() \
		else cv2.CAP_PROP_FRAME_COUNT
	total = int(vs.get(prop))
	print("[INFO] Total frames in video {}".format(total))

# an error occurred while trying to determine the total
# number of frames in the video file
except:
	print("[INFO] Could not determine no. of frames in video")
	#print("[INFO] no approx. completion time can be provided")
	total = -1
countRight=0
countLeft=0
sample=0

##coord=[[240,0],[240,640]]
#coord=[[600,252],[900,252],[631,512],[952,512]]
##while True:

#(grabbed, frame )= vs.read() #Live image
frame=cv2.imread('C:\\Users\\shefa\\Desktop\\Vehicle Detection\\Vehicle Detection\\videos\\car3.jpeg') #Stored Image
height,width,_=frame.shape
print(height,width)

frame1 = frame[0:480,320:640]#right [0:x,y/2:y] x-height
frame2 = frame[0:480,0:320]#left


#frame1 = frame[0:306,204:408] 
#frame2 = frame[0:306,0:204]

# if the frame dimensions are empty, grab them
if W is None or H is None:
        (H, W) = frame1.shape[:2] #H=480, W=320
        #print(H)
        #print(W)

# construct a blob from the input frame and then perform a forward
# pass of the YOLO object detector, giving us our bounding boxes
# and associated probabilities
blob = cv2.dnn.blobFromImage(frame1, 1 / 255.0, (416, 416), swapRB=True, crop=False)

#Calculate Network response
net.setInput(blob) 
start = time.time()
layerOutputs = net.forward(ln) #Vectors of length 85
print(layerOutputs[0].shape) #(507, 85) - 507 bounding boxes
print(layerOutputs[1].shape)
print(layerOutputs[2].shape)
end = time.time()

# initialize our lists of detected bounding boxes, confidences,
# and class IDs, respectively
boxes = []
confidences = []
classIDs = []

# loop over each of the layer outputs
for output in layerOutputs:
        # loop over each of the detections
        for detection in output: #detection -> object bounding box detected
                # extract the class ID and confidence (i.e., probability) of the current object detection                
                scores = detection[5:] #ignoring first 5 since they are x,y,w, h values
                classID = np.argmax(scores) #index of obj(coco class) having highest confidence value
                confidence = scores[classID] #confidence value of obj

                # filter out weak predictions by ensuring the detected probability is greater than the minimum probability                
                if confidence > args["confidence"]:
                        # scale the bounding box coordinates back relative to
                        # the size of the image, YOLO returns the center (x, y)-coordinates of 
                        # the bounding box followed by the boxes' width and height                        
                        box = detection[0:4] * np.array([W, H, W, H]) #multiplying detection decimal values by actual w, h values to get them to appropriate pixel value
                        (centerX, centerY, width, height) = box.astype("int") #convert to int since pixel is int

                        # use the center (x, y)-coordinates to derive the bottom and and left corner of the bounding box                        
                        x = int(centerX - (width / 2))
                        y = int(centerY - (height / 2))

                        # update our list of bounding box coordinates,
                        # confidences, and class IDs
                        boxes.append([x, y, int(width), int(height)])
                        confidences.append(float(confidence))
                        classIDs.append(classID)

# apply non-maxima suppression to suppress weak, overlapping bounding boxes, and selecting those which have confidence > args[confidence]
idxs = cv2.dnn.NMSBoxes(boxes, confidences, args["confidence"],
        args["threshold"]) # threshold (nms threshold) = % of bbox to be kept from those above the confidence - lower the value , lower no of bounding values kept for an object

# ensure at least one detection exists
if len(idxs) > 0:
        # loop over the indexes we are keeping
        for i in idxs.flatten(): #flatten collapses 2D array to 1D. eg [[1,2],[3,4]] -> [1,2,3,4]
                # extract the bounding box coordinates
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])

                # draw a bounding box rectangle and label on the frame
                color = [int(c) for c in COLORS[classIDs[i]]]
                cv2.rectangle(frame1, (x, y), (x + w, y + h), color, 2) # 2 is thickness
                text = "{}: {:.4f}".format(LABELS[classIDs[i]],
                        confidences[i])
##                        print(text)
                text1="{}".format(LABELS[classIDs[i]])
                print(text1)
                if (text1 == 'car' or text1 == 'motorbike' or text1 == 'bus' or text1 == 'truck'):
                        bbox, label, conf = cv.detect_common_objects(frame1)
                        #output_image = draw_bbox(frame, bbox, label, conf)
                        countRight+=1

                print(countRight)
                
# if the frame dimensions are empty, grab them
if W is None or H is None:
        (H, W) = frame2.shape[:2]

# construct a blob from the input frame and then perform a forward
# pass of the YOLO object detector, giving us our bounding boxes
# and associated probabilities
blob = cv2.dnn.blobFromImage(frame2, 1 / 255.0, (416, 416),
        swapRB=True, crop=False)
net.setInput(blob)
start = time.time()
layerOutputs = net.forward(ln)
end = time.time()

# initialize our lists of detected bounding boxes, confidences,
# and class IDs, respectively
boxes = []
confidences = []
classIDs = []

# loop over each of the layer outputs
for output in layerOutputs:
        # loop over each of the detections
        for detection in output:
                # extract the class ID and confidence (i.e., probability)
                # of the current object detection
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]

                # filter out weak predictions by ensuring the detected
                # probability is greater than the minimum probability
                if confidence > args["confidence"]:
                        # scale the bounding box coordinates back relative to
                        # the size of the image, keeping in mind that YOLO
                        # actually returns the center (x, y)-coordinates of
                        # the bounding box followed by the boxes' width and
                        # height
                        box = detection[0:4] * np.array([W, H, W, H])
                        (centerX, centerY, width, height) = box.astype("int")

                        # use the center (x, y)-coordinates to derive the top
                        # and and left corner of the bounding box
                        x = int(centerX - (width / 2))
                        y = int(centerY - (height / 2))

                        # update our list of bounding box coordinates,
                        # confidences, and class IDs
                        boxes.append([x, y, int(width), int(height)])
                        confidences.append(float(confidence))
                        classIDs.append(classID)

# apply non-maxima suppression to suppress weak, overlapping
# bounding boxes
idxs = cv2.dnn.NMSBoxes(boxes, confidences, args["confidence"],
        args["threshold"])

# ensure at least one detection exists
if len(idxs) > 0:
        # loop over the indexes we are keeping
        for i in idxs.flatten():
                # extract the bounding box coordinates
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])

                # draw a bounding box rectangle and label on the frame
                color = [int(c) for c in COLORS[classIDs[i]]]
                cv2.rectangle(frame2, (x, y), (x + w, y + h), color, 2)
                text = "{}: {:.4f}".format(LABELS[classIDs[i]],
                        confidences[i])
##                        print(text)
                text1="{}".format(LABELS[classIDs[i]])
                print(text1)
                if (text1 == 'car' or text1 == 'motorbike' or text1 == 'bus' or text1 == 'truck'):
                        bbox, label, conf = cv.detect_common_objects(frame2)
                        #output_image = draw_bbox(frame, bbox, label, conf)
                        countLeft+=1

                print(countLeft)
                
                #cv2.line(frame, (315,4), (315,470), (0, 0, 255), 2) #Vertical left line
                cv2.line(frame, (315,4), (315,470), (0, 0, 255), 2)
                cv2.putText(frame, "Right vehicle count: "+str(countRight), (320, 90), cv2.FONT_HERSHEY_SIMPLEX,0.7,(0, 255, 0), 2)
                cv2.putText(frame, "Left vehicle count: "+str(countLeft), (50, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,255, 0), 2)
                

print("Right vehicle count: "+str(countRight))
print("Left vehicle count: "+str(countLeft))
threshold=2
if(countRight>threshold and countLeft>threshold):
        
        if(countRight>countLeft):
                data.write(str.encode('A'))
                print("Right side has higher traffic density. Divider moving to left side")
        elif(countLeft>countRight):
                data.write(str.encode('B'))
                print("Left side has higher traffic density. Divider moving to right side")
        else:
                print("Equal traffic density, divder doesnt move.")
else:
        print("Divider doesnt move")        
##        cv2.imshow('frmae1',frame1)
##        cv2.imshow('frmae2',frame2)
cv2.imshow('Vehicle Detection and count',frame)
##if cv2.waitKey(1) & 0xFF == ord('q'):
##        Object.close()
##        break

#vs.release()
