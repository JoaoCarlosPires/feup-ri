# MadDuck Avoidance

Intelligent Robotics, M.EIC, FEUP, 2022-2023

**Authors**

- Julián Ferreira, up202202340
- Lisa Sonck, up202202272
- Joao Pires, up201806079

Code has been tested for Ubuntu 20.04 (Focal Fossa) with Ros Noetic. The training of the Yolov3 model happened on a Windows 10 system.

**Dependencies**

*For the obstacle detection, it is necessary to have installed the following packages:*
- Darknet
- NVIDIA Cuda Toolkit
- Cuda
- Cudnn
- python-opencv
- python3

*For the obstacle avoidance, it is necessary to have installed the following packages:*
- ROS and rospy
- Duckietown shell
- Duckietown environment
- pandas
- pyglet

**Source codes**

1. `create_sets.py`
2. `view_box.py`
3. `detect_duckie.py`
4. `madduck_avoidance.py`

These files contain the executable code in this project. `create_sets.py` alters the format of the annotation per image to the format of Yolov3 and writes them to a text file with the same name as the corresponding image. It also creates `testdata.txt` and `traindata.txt`, which contain the paths to the images used in training and testing the Yolov3 model. `view_box.py` allows to see the the Yolov3 annotations on the image. `detect_duckie.py` contains the logic to use the trained model on an image and return if a duck was detected. `madduck_avoidance.py` contains all the logic to run the project. It opens the duckietown simulator with the view of the robots camera and you can manually move the robot. After pressing SPACE the robot will run from itself and detect and avoid duckies.

**Directory Organization**

- MadDuckAvoidance
    - cfg
    - duck_frames

All source code is inside the MadDuckAvoidance folder. The cfg folder contains the customization of the Yolov3 model, together with the names of the detected classes and the parameters for executing the model in duckie.data. The duck_frames folder contains all used images, together with their annotations in an equally named text file.

**Executable**

First download darknet (https://medium.com/geekculture/yolov4-darknet-installation-and-usage-on-your-system-windows-linux-8dec2cea6e81) and make it.
You will now have a folder 'darknet' with the executable inside. Copy the complete darknet folder into the MadDuckAvoidance folder.

Next make an empty folder inside the copied darknet folder and name it 'duckie_backup'. Download the trained weights from https://drive.google.com/drive/folders/1iOtEv69aJzh7Kf8MG1B9RedCnq1lo5d6?usp=sharing and place them inside this new folder.

The following step is to download the DuckieTown Simulator. First ensure that you have numpy version 1.19.5. If not, errors will occur. Then execute following commands:
>pip install testresources \
>pip install py-multihash==0.2.0 \
>pip install Pillow=8.3.2 \
>pip install pyglet==1.5.11 \
>pip3 install duckietown-gym-daffy 

Next go inside a command prompt to the MadDuckAvoidance folder and execute:
- chmod +x madduck_avoidance.py
- ./madduck_avoidance.py

By doing this the project can be tested. After execution the duckietown simulator is displayed with the camera of the duckiebot

**How to test it?**

At first, when the project is launched you can control manually the robot by using the arrow keys to put it in the desired position. Once you have it where you want, if you press the SPACE button the robot is going to move autonomously avoiding the duckies he finds in the road.

In the terminal where you launched the program you can view the data of the obstacles the duckiebot detects with the following structure:
    [x, y, width, height] setting (x,y) in the top left corner

To finish the program you have to click the button ESC, which will automatically close the simulator and stop the execution.

**Object Detection**

The object detection training happend on a Windows 10 system.

For the object detection the dataset of Soroush Saryazdi and Dhaivat Bhatt (https://github.com/saryazdi/Duckietown-Object-Detection-LFV/blob/master/DuckietownObjectDetectionDataset.md) was used together with the Yolov3 algorithm (https://pjreddie.com/darknet/yolo/). The training happened with darknet. Darknet can be downloaded through https://medium.com/geekculture/yolov4-darknet-installation-and-usage-on-your-system-windows-linux-8dec2cea6e81.

The dataset was altered for our specific use. Only the images that contain a rubber duck were kept and the annotations were reformatted to the standard format for Yolov3: <class> <x> <y> <width> <height>, where (x,y) is the center of the surrounding box and the width and height its dimensions. The class is always equal to 0, since we only train to detect one object. Besides this are the images also split into two sets: the training set and the testset. The training set is used in the training and the test set is used for validating later on. 
These steps are implemented in the create_sets.py.

To recreate these steps the data from the dataset of Saryazdi and Bhatt needs to be downloaded together with the annotations. Afterwards per picture needs to be checked if it contains a duckie. If positive, the picture is copied to the folder duck_frames. The annotations are altered in the sense that only the annotations for the duckies are kept and all others removed and rename this file to `duck_annotations_all.json` and place it inside the MadDuckAvoidance folder. After doing this you have to alter the exact paths in the `create_sets.py` to the location of your duck_frames folder. 
Now you can execute `create_sets.py` with the command: ```python create_sets.py``` inside a python environment.

The Yolov3 model is also altered to our specific needs and can be found in cfg/yolov3-duckie.cfg. To make the training more efficient, the pretrained weights from the darknet53 model are used. These can be downloaded from https://pjreddie.com/darknet/yolo/ and needs to be placed inside the darknet folder. To train the model the following command needs to be executed inside the darknet folder (Make sure you have a duckie_backup folder inside the darknet folder.):
>darknet.exe detector train cfg/duckie.data ../cfg/yolov3-duckie.cfg darknet53.conv.74
  
These will train the weights specifically to detect rubber ducks. The final weights are placed inside the duckie_backup folder.
  
To validate the results next command can be run: 
>darknet.exe detector map cfg/duckie.data ../cfg/yolov3-duckie.cfg duckie_backup/yolov3-duckie_final.weights.

This will calculate the average precision of the model, based on the test images.
  
Our trained weights can be downloaded from https://drive.google.com/drive/folders/1iOtEv69aJzh7Kf8MG1B9RedCnq1lo5d6?usp=sharing.

