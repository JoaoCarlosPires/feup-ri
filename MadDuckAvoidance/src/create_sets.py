##converts the annotations to the yolov3 format and divides the images in a train and test set.
import json
from os import listdir
files = [f for f in listdir("C:/Users/Lisa/UPorto/Robotics/Project/duckiestuff-master/duck_frames")]
print(files[0])


## loading all annotations
with open('duck_annotations_all.json') as f: 
	data = json.load(f)

def convert(size, box):
    dw = 1./size[0]
    dh = 1./size[1]
    x = (box[0] + box[1])/2.0
    y = (box[2] + box[3])/2.0
    w = box[1] - box[0]
    h = box[3] - box[2]
    x = x*dw
    w = w*dw
    y = y*dh
    h = h*dh
    return (x,y,w,h)

w = 640.
h = 480.
i = 0
test = ""
train = ""
for file in files:
    ann = data[file]
    if not ann:
        print("help")
        continue

    for a in ann:
        xmin = a['bbox'][0]
        ymin = a['bbox'][1]
        xmax = a['bbox'][0] + a['bbox'][2]
        ymax = a['bbox'][1] + a['bbox'][3]
        b = (xmin, xmax, ymin, ymax)

        bb = convert((w,h), b)

        f = open("duck_frames/%s.txt" %file[0:-4], "a")
        f.write(str(0) + " " + " ".join([str(a) for a in bb]) + '\n')
        f.close()

    if i%3:
        train += "C:/Users/Lisa/UPorto/Robotics/Project/duckiestuff-master/duck_frames/%s\n" %file
    else:
        test += "C:/Users/Lisa/UPorto/Robotics/Project/duckiestuff-master/duck_frames/%s\n" %file
    i += 1
    
ftest = open("testdata.txt", "a")
ftest.write(test)
ftest.close()

ftrain = open("traindata.txt", "a")
ftrain.write(train)
ftrain.close()


