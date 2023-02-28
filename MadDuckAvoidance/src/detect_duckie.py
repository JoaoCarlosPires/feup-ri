## function to determine if a rubber duck was detected in an image.
import cv2
import numpy as np
from PIL import Image

cap = cv2.VideoCapture(0)
whT = 320
confThreshold = 0.5
nmsThreshold = 0.3

classesFile = "cfg/duckie.names"
classNames = []
with open(classesFile,'rt') as f:
    classNames = f.read().rstrip('n').split('n')
print(classNames)
print(len(classNames))

modelConfiguration = "cfg/yolov3-duckie.cfg"
modelWeights = "darknet/duckie_backup/yolov3-duckie_final.weights"

net = cv2.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_DEFAULT)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

def findObjects(img):
    # np_img = np.array(img)
    blob = cv2.dnn.blobFromImage(img, 1/255,(whT,whT),[0,0,0],crop=False)
    net.setInput(blob)
    layerNames = net.getLayerNames()
    outputNames = [layerNames[i-1] for i in net.getUnconnectedOutLayers()]
    outputs = net.forward(outputNames)

    hT, wT, cT = img.shape
    bbox = []
    classIds = []
    confs = []

    for output in outputs:
        for det in output:
            scores = det[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]
            if confidence > confThreshold:
                w,h = int(det[2]* wT), int(det[3]*hT)
                x,y = int((det[0]*wT)-w/2), int((det[1]*hT)-h/2)
                bbox.append([x,y,w,h])
                classIds.append(classId)
                confs.append(float(confidence))

    indices = cv2.dnn.NMSBoxes(bbox, confs,confThreshold,nmsThreshold)
    found_ducks = []
    print(bbox)
    for i in indices:
        box = bbox[i]
        box.append(confs[i])
        found_ducks.append(box)
        x,y,w,h = box[0], box[1], box[2], box[3]
        cv2.rectangle(img, (x,y),(x+w,y+h),(255,0,255),2) #image, top left corner, bottom right corner, color, thickness
        cv2.putText(img,f'{classNames[classIds[i]].upper()} {int(confs[i]*100)}%',
                    (x,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(255,0,255),2)
    return found_ducks #list of box coordinates: [x, y, w, h, confidence] (x,y) if top left corner


# #success, img = cap.read()
# img = Image.open("duck_frames/frame_001027.png")
# np_img = np.array(img)
# ducks = findObjects(np_img)

# if(len(ducks) != 0):
#     #duckie found
#     cv2.imshow('Image', np_img)
#     cv2.waitKey(0)

