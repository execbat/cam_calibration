#!/usr/bin/env python
# coding: utf-8
"""
Object Detection From TF1 Saved Model
=====================================
"""

# %%
# This demo will take you through the steps of running an "out-of-the-box" TensorFlow 1 compatible
# detection model on a collection of images. More specifically, in this example we will be using
# the `Saved Model Format <https://www.tensorflow.org/guide/saved_model>`__ to load the model.

# %%
# Download the test images
# ~~~~~~~~~~~~~~~~~~~~~~~~
# First we will download the images that we will use throughout this tutorial. The code snippet
# shown bellow will download the test images from the `TensorFlow Model Garden <https://github.com/tensorflow/models/tree/master/research/object_detection/test_images>`_
# and save them inside the ``data/images`` folder.
import os
import sys
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'    # Suppress TensorFlow logging (1)
#import pathlib
import tensorflow as tf

tf.get_logger().setLevel('ERROR')           # Suppress TensorFlow logging (2)

# Enable GPU dynamic memory allocation
gpus = tf.config.experimental.list_physical_devices('GPU')
for gpu in gpus:
    tf.config.experimental.set_memory_growth(gpu, True)




# %%
# Download the model
# ~~~~~~~~~~~~~~~~~~
# The code snippet shown below is used to download the pre-trained object detection model we shall
# use to perform inference. The particular detection algorithm we will use is the
# `SSD MobileNet v2`. More models can be found in the `TensorFlow 1 Detection Model Zoo <https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf1_detection_zoo.md>`_.
# To use a different model you will need the URL name of the specific model. This can be done as
# follows:
#
# 1. Right click on the `Model name` of the model you would like to use;
# 2. Click on `Copy link address` to copy the download link of the model;
# 3. Paste the link in a text editor of your choice. You should observe a link similar to ``download.tensorflow.org/models/object_detection/XXXXXXXXX.tar.gz``;
# 4. Copy the ``XXXXXXXXX`` part of the link and use it to replace the value of the ``MODEL_NAME`` variable in the code shown below;
#
# For example, the download link for the model used below is: ``download.tensorflow.org/models/object_detection/ssd_mobilenet_v2_coco_2018_03_29.tar.gz``

# Download and extract model
#def download_model(model_name):
#    base_url = 'http://download.tensorflow.org/models/object_detection/'
#    model_file = model_name + '.tar.gz'
#    model_dir = tf.keras.utils.get_file(fname=model_name,
#                                        origin=base_url + model_file,
#                                        untar=True)
#    return str(model_dir)

#MODEL_NAME = 'ssd_mobilenet_v2_coco_2018_03_29'
#PATH_TO_MODEL_DIR = download_model(MODEL_NAME)

# %%
# Download the labels
# ~~~~~~~~~~~~~~~~~~~
# The coode snippet shown below is used to download the labels file (.pbtxt) which contains a list
# of strings used to add the correct label to each detection (e.g. person). Since the pre-trained
# model we will use has been trained on the COCO dataset, we will need to download the labels file
# corresponding to this dataset, named ``mscoco_label_map.pbtxt``. A full list of the labels files
# included in the TensorFlow Models Garden can be found `here <https://github.com/tensorflow/models/tree/master/research/object_detection/data>`__.

# Download labels file
#def download_labels(filename):
#    base_url = 'https://raw.githubusercontent.com/tensorflow/models/master/research/object_detection/data/'
#    label_dir = tf.keras.utils.get_file(fname=filename,
#                                        origin=base_url + filename,
#                                        untar=False)
#    label_dir = pathlib.Path(label_dir)
#    return str(label_dir)

LABEL_FILENAME = 'teat_label_map.pbtxt'
PATH_TO_LABELS = './teat_label_map.pbtxt'#download_labels(LABEL_FILENAME)

# %%
# Load the model
# ~~~~~~~~~~~~~~
# Next we load the downloaded model
import time
import datetime
from utils import label_map_util
#from utils import visualization_utils as viz_utils

PATH_TO_SAVED_MODEL = '/home/oem/obj_det/saved_model9'#PATH_TO_MODEL_DIR + "/saved_model"

#print('Loading model...')
start_time = time.time()

# Load saved model and build the detection function
model = tf.saved_model.load(PATH_TO_SAVED_MODEL)
detect_fn = model.signatures['serving_default']

end_time = time.time()
elapsed_time = end_time - start_time
print('Done! Took {} seconds'.format(elapsed_time))

# %%
# Load label map data (for plotting)
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Label maps correspond index numbers to category names, so that when our convolution network
# predicts `5`, we know that this corresponds to `airplane`.  Here we use internal utility
# functions, but anything that returns a dictionary mapping integers to appropriate string labels
# would be fine.

category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS,
                                                                    use_display_name=True)

# %%
# Putting everything together
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~
# The code shown below loads an image, runs it through the detection model and visualizes the
# detection results, including the keypoints.
#
# Note that this will take a long time (several minutes) the first time you run this code due to
# tf.function's trace-compilation --- on subsequent runs (e.g. on new images), things will be
# faster.
#
# Here are some simple things to try out if you are curious:
#
# * Modify some of the input images and see if detection still works. Some simple things to try out here (just uncomment the relevant portions of code) include flipping the image horizontally, or converting to grayscale (note that we still expect the input image to have 3 channels).
# * Print out `detections['detection_boxes']` and try to match the box locations to the boxes in the image.  Notice that coordinates are given in normalized form (i.e., in the interval [0, 1]).
# * Set ``min_score_thresh`` to other values (between 0 and 1) to allow more detections in or to filter out more detections.
import numpy as np
from PIL import Image
#import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings('ignore')   # Suppress Matplotlib warnings

def load_image_into_numpy_array(path):
    """Load an image from file into a numpy array.

    Puts image into numpy array to feed into tensorflow graph.
    Note that by convention we put it into a numpy array with shape
    (height, width, channels), where channels=3 for RGB.

    Args:
      path: the file path to the image

    Returns:
      uint8 numpy array with shape (img_height, img_width, 3)
    """
    return np.array(Image.open(path))
def live_image_into_numpy_array(frame1, frame2, frame3):
    """Load an image from file into a numpy array.

    Puts image into numpy array to feed into tensorflow graph.
    Note that by convention we put it into a numpy array with shape
    (height, width, channels), where channels=3 for RGB.

    Args:
      path: the file path to the image

    Returns:
      uint8 numpy array with shape (img_height, img_width, 3)
    """
    out = np.uint8(np.dstack([frame1, frame2, frame3]))
    return out

#import cv2
import numpy as np
import math
import socket
import io
import xml.etree.ElementTree as xml

#from ifm3dpy import O3RCamera, FrameGrabber, ImageBuffer
#import cv2
import argparse
#import matplotlib as mpl
#import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D
#import numpy as np
#from scipy.spatial.transform import Rotation as R
#import _thread
import time, datetime
import select
import os
#try:
#    import open3d as o3d
#    OPEN3D_AVAILABLE = True
#except ModuleNotFoundError:
#    OPEN3D_AVAILABLE = False

#image_choices = ["jpeg", "distance", "amplitude"]
#if OPEN3D_AVAILABLE:
#    image_choices += ["xyz"]
#if OPEN3D_AVAILABLE:
#    image_choices += ["xyz"]
#    
#buf = ImageBuffer()

UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPClientSocket.bind(('127.0.0.1', 59157)) 
#UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
#UDPClientSocket.bind(('192.168.127.10', 59157))  # Инициализирует ip-адрес и порт.
#UDPClientSocket.bind(('127.0.0.1', 59152))  # Инициализирует ip-адрес и порт.

##client_address=('127.0.0.1', 59152)

#print(UDPClientSocket)



###UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
###UDPServerSocket.bind(('192.168.1.10', 59152))  # Инициализирует ip-адрес и порт.

## Receive-UDP-data event-loop begins her
#UDPClientSocket.setblocking(True)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#s.bind(('',59155))
s.setblocking(0)
print('UDP server is turning on')

def SendData(Transmit,OpenedSocket, target_address):
    sent = OpenedSocket.sendto(Transmit, target_address)


def extract_xml(bytes_parcel, find = 'all'): # 'find' could be also the list of key-words
    res_dict = dict()
    #print(   bytes_parcel)
    try: # checkking if we can parse input data to ElementTree
        tree = xml.parse(io.BytesIO(bytes_parcel)) # xml.etree.ElementTree.ElementTree

    except TypeError:
        print('incorrect type of input data')

    else:
        try: # checking 'find'
            find = find.lower()
            

        except AttributeError: #operating with list
            root = tree.getroot()
            
            #print('cant make lowercase')
            for i_tag in find:
                elem = tree.find(i_tag)                
                try: # if incorrect key_word in list
                    if elem.text != None:
                        res_dict[elem.tag] = elem.text    # if element contains text
                    else:
                        res_dict[elem.tag] = elem.attrib  # if element contains dict
                except AttributeError:
                    print('Data do not contain "{}" key-word'.format(i_tag))


        else: #checking ALL
            if find == 'all':
                root = tree.getroot()
                if root.text != None:        
                    res_dict[root.tag] = root.text    # if element contains text
                else:
                    res_dict[root.tag] = root.attrib  # if element contains dict

                for elem in root:
                    if elem.text != None:        
                        res_dict[elem.tag] = elem.text    # if element contains text
                    else:
                        res_dict[elem.tag] = elem.attrib  # if element contains dict
            else:
                raise AttributeError('Use "all" or the list of keywords')            

    return res_dict



# Creates structure for ambedding into the xml package, which will be send to robot
def create_xml(sent_mess_list):
    root = xml.Element("empty")
    for num, elem in enumerate(sent_mess_list):
        key = list(elem.keys())[0]
        value = elem[key]
        if type(value) == dict: 
            if num == 0: 
                root = xml.Element(key , elem[key])
            else:
                element_tree = xml.SubElement(root, key , elem[key])
                
        elif type(value) == str:
            if num == 0:
                root = xml.Element(key)
                root.text = value 
            else:
                element_tree = xml.SubElement(root, key)
                element_tree.text = value                 
        else     :
            root = xml.Element(key)
    tree = xml.ElementTree(root)    
    DecodedTree = xml.tostring(root, encoding='utf8', method='xml', short_empty_elements='true', xml_declaration=False)
    
    return(DecodedTree)

def SendData(Transmit,OpenedSocket, target_address):
    sent = OpenedSocket.sendto(Transmit, target_address)

hand_hist = None
traverse_points = np.zeros([4, 2], dtype=int) #4 points with 3 values, x, y and
max_age=20
traverse_points_age = np.zeros([4, 1], dtype=int) 
total_rectangle = 9
hand_rect_one_x = None
hand_rect_one_y = None

hand_rect_two_x = None
hand_rect_two_y = None

def process_teats(far_points, pt):
            
            #print("Centroid : " + str(cnt_centroid) + ", farthest Point : " + str(far_points))
            
            cam_pos = list()
            teat_cntr=0
            teats = np.zeros( shape=( 4 , 3 ) )

            ###print("far_points", far_points)
            if False:#far_points != None:# and (far_points <= 100).any()and (far_points >= 20).any():
                far_points=sorted(far_points, key=lambda x: x[0], reverse=True)
                #print("farthest Point sorted: " + str(far_points))
                far_points_history=far_points
                ###print("far_points_history", far_points_history)
                for i in range(len(far_points)):
                    
                    far_point=tuple(far_points[i])
                    
                    #print(far_points[i])
                   # cv2.circle(frame1, far_point, 5, [255, 128, 128], -1)
                    #print("traverse_point : " + str(cnt_centroid) + ", farthest Point : " + str(far_point))
                    if np.any(far_points[i]):
                        
                        ###print("iiiii "+ str(i)+ str(far_point))
                        for j in range(len(traverse_points)):
                        
                        #for traverse_point in traverse_points:
                            traverse_point=tuple(traverse_points[j])
                           ### print("jjj "+ str(j)+ str(traverse_point)+ "age " + str(traverse_points_age[j]))
                            #traverse_point_age=tuple(traverse_points_age[j])
                            for k in range(len(traverse_points)):
                                if j == k:
                                    break
                                ###print("kkk "+ str(j)+ str(traverse_point)+ "age " + str(traverse_points_age[j]))
                                if np.allclose(traverse_points_age[k], traverse_points_age[j], rtol=0.1, atol=0.1, equal_nan=False) or traverse_points_age[k]>max_age:
                                    traverse_points[k]=(0, 0)
                                    traverse_points_age[k]=0
                                    print("delete old tp")
                            
        #                    if len(traverse_points) < 4:
        #                        np.append(traverse_points, far_point)
                            if np.allclose(far_point, traverse_point, rtol=0.1, atol=0.1, equal_nan=False):
        #                        far_points[i], far_points[j] = far_points[j], far_points[i] 
                                traverse_points[j]=far_point
                                traverse_points_age[j]=0
                                print("close")
                                #np.delete(traverse_points, traverse_point)
                                #np.append(traverse_points, far_point)
                                break
                            elif not np.any(traverse_points[j]) or traverse_points_age[j]>max_age:
                                traverse_points[j]=far_point
                                traverse_points_age[j]=0
                                print("fill empty tp")
                                #np.delete(traverse_points, traverse_point)
                                #np.append(traverse_points, far_point)
                                break
                            
                                #np.delete(traverse_points, traverse_point)
                                #np.append(traverse_points, far_point)
                                #break
                            
                                
                            #                        elif j == len(traverse_points):
                            #                            traverse_points.pop(0)
                            #                            np.append(traverse_points, far_point)
                                traverse_points_age[j]+=1
                                h=16
                                w=16
                                y1 = traverse_points[j][1] + 6
                                y2 = traverse_points[j][1] - (h)
                                x1 = traverse_points[j][0] - int(w/2)
                                x2 = traverse_points[j][0] + int(w/2)
                                print(x1)
                                # Draw the bounding box on the frame
#                                cv2.rectangle(
#                                    frame1, (x1,y1),
#                                    (x2,y2), (0,255,0),
#                                    2
#                                )
#                                gray = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
#                                to_canny = cv2.GaussianBlur(gray, (5, 5), 3)
#                                
#                                edges = cv2.Canny(to_canny, 0, 127)
#                                print (to_canny)
#                                print (edges)
#                                # Since the edges are only a 2-channel frame,
#                                # we can overlay it on to each channel in the
#                                # original frame
#                                frame1[y1:y2, x1:x2, 0] = edges[y1:y2, x1:x2]
#                                frame1[y1:y2, x1:x2, 1] = edges[y1:y2, x1:x2]
#                                frame1[y1:y2, x1:x2, 2] = edges[y1:y2, x1:x2]



                    #draw_circles(frame1, traverse_points)
                    
                    
                    a=int(far_point[1])
#                    if (far_point[0]<cnt_centroid[0]):
#                        b=int(far_point[0])
#                    else:
                    b=int(far_point[0])
                    x=pt[a, b][0]
                    y=pt[a, b][1]
                    z=pt[a, b][2]
                    print("index: "+str(i))
                    print("x "+str(x))
                    print("y "+str(y))
                    print("z "+str(z))


                    if x!=0 and y!=0 and z!=0:
                        teats[teat_cntr] = np.array([x,y,z])
                        teat_cntr=teat_cntr+1
                    #wd_dict={'Digout':{'o1': str(wd_bit)}}
           #######         cam_dict={'XYZ'+str(i+1): {'X': str(x), 'Y': str(y),'Z': str(z)}}
                    #cam_pos.append(cam_dict)
                  #####  cam_pos.append(cam_dict)
            if (teat_cntr>=2):
                udder_midpt= np.sum(teats, axis=0) / teat_cntr
                print("udder_midpt", udder_midpt)
                print("teat_cntr", teat_cntr, teats.shape[0], teats)
#                x=udder_midpt[0]
#                if(teat_cntr==1)
#                    x=
                y=udder_midpt[1]
                z=udder_midpt[2]
                cam_dict={'XYZ1': {'X': str(x), 'Y': str(y),'Z': str(z)}}
                #cam_dict={'XYZ1': {'X': str(x), 'Y': str(y),'Z': str(z)},'XYZa': {'X': str(teats[0][000000000000000000000]), 'Y': str(teats[0][1]),'Z': str(teats[0][2])}},'XYZb': {'X': str(teats[1][0]), 'Y': str(teats[1][1]),'Z': str(teats[1][2])}, 'XYZc': {'X': str(teats[2][0]), 'Y': str(teats[2][1]),'Z': str(teats[2][2])}, 'XYZd': {'X': str(teats[3][0]), 'Y': str(teats[3][1]),'Z': str(teats[3][2])}}
                cam_pos.append(cam_dict)
                

            elif (teat_cntr==1):
                x=teats[0][0]
    #                if(teat_cntr==1)
    #                    x=
                y=teats[0][1]
                z=teats[0][2]
                cam_dict={'XYZ1': {'X': str(x), 'Y': str(y),'Z': str(z)}}
                cam_pos.append(cam_dict)
#            else
#                cam_dict={'XYZ1': {'X': str(1), 'Y': str(2),'Z': str(3)}}
#                cam_pos.append(cam_dict)
#            x=50
 #           y=10
  #          z=500
   #         cam_dict={'XYZ1': {'X': str(x), 'Y': str(y),'Z': str(z)}}
    #        cam_pos.append(cam_dict)
           
            if(len(cam_pos) != 0):
                cam_dict={'XYZa': {'X': str(teats[0][0]), 'Y': str(teats[0][1]), 'Z': str(teats[0][2])}}
                cam_pos.append(cam_dict)
                cam_dict={'XYZb': {'X': str(teats[1][0]), 'Y': str(teats[1][1]), 'Z': str(teats[1][2])}}
                cam_pos.append(cam_dict)
                cam_dict={'XYZc': {'X': str(teats[2][0]), 'Y': str(teats[2][1]), 'Z': str(teats[2][2])}}
                cam_pos.append(cam_dict)
                cam_dict={'XYZd': {'X': str(teats[3][0]), 'Y': str(teats[3][1]), 'Z': str(teats[3][2])}}

                cam_pos.append(cam_dict)
                print(cam_pos)
                telegram = create_xml(cam_pos)
                    
                UDPClientSocket.sendto(telegram, ('127.0.0.1', 59152))
                print("sent", telegram)            
#        telegram = create_xml(cam_pos)
#                
#        UDPClientSocket.sendto(telegram, ('127.0.0.1', 59152))


  
        #key = cv2.waitKey(200)
import ifm3dpy
from ifm3dpy.device import O3D
from ifm3dpy.framegrabber import FrameGrabber, buffer_id
#import cv2
import argparse

#matplotlib notebook
#import ifm3dpy
#import matplotlib as mpl
#import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D
#import numpy as np
##from scipy.spatial.transform import Rotation as R

#try:
#    import open3d as o3d
#    OPEN3D_AVAILABLE = True
#except ModuleNotFoundError:
#    OPEN3D_AVAILABLE = False


    # Render the scene
#    fig = plt.figure()
#    ax = fig.add_subplot(111, projection='3d')
#    ax.scatter(
#        x_computed,
#        y_computed,
#        z_computed,
#        s = 0.1,
#        marker = ',',
#        c = x_computed)




#def display_2d(fg, buf, getter, title, index):
    
        

 
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcic-port", help="The pcic port from which images should be received", type=int,
                        required=False, default=50010)
#    parser.add_argument("--image", help="The image to received (default: distance)", type=str,
#                        choices=image_choices, required=False, default="amplitude")
    parser.add_argument("--ip", help="IP address of the sensor (default: 192.168.178.224)",
                        type=str, required=False, default="192.168.178.224")
    parser.add_argument("--xmlrpc-port", help="XMLRPC port of the sensor (default: 50010)",
                        type=int, required=False, default=50010)
    parser.add_argument("--index", help="results file index",
                        type=str, required=False, default=0)                    
    args = parser.parse_args()

   # getter = globals()["get_" + args.image]
        
    cam = O3D(ip=args.ip, xmlrpc_port=args.xmlrpc_port)
    fg = FrameGrabber(cam, pcic_port=args.xmlrpc_port)

    fg.start(
        [buffer_id.AMPLITUDE_IMAGE, buffer_id.RADIAL_DISTANCE_IMAGE, buffer_id.CONFIDENCE_IMAGE, buffer_id.XYZ]
    )
    title = "O3D Port {}".format(str(args.pcic_port))
    
    index=0
    subindex=0
#    if args.image == "xyz":
#        display_3d(fg, buf, getter, title)
#    else:
#        display_2d(fg, buf, getter, title, args.index)


    shoot=0
    prev_shoot=0
    data = 'Nothing received'
    #  while True:
    #     newestData = None

    #     keepReceiving = True
    #     while keepReceiving:
    #        try:
    #           data, fromAddr = sock.recvfrom(2048)
    #           if data:
    #              newestData = data
    #        except socket.error as why:
    #           if why.args[0] == EWOULDBLOCK:
    #              keepReceiving = False
    #           else:
    #              raise why

    #     if (newestData):
            # code to handle/parse (newestData) here
    index=1
    subindex=0
    
    today = datetime.date.today()  
    datestamp=today.strftime("%Y_%m_%d")
    timestamp=today.strftime("%H_%M_%S")
    ampm=today.strftime("%p")
    print(datestamp)
    os.makedirs('/mnt/tagmii1/'+datestamp+'/'+ampm, exist_ok=True) 
    
    
    path='/mnt/tagmii1/'+datestamp+'/'+ampm+'/'
    isExist = os.path.exists(path+str(index))
    while isExist:
        index=index+1
        path='/mnt/tagmii1/'+datestamp+'/'+ampm+'/'+str(index)
        isExist = os.path.exists(path)
        print("index is there ", index)
    #todaystr = today.isoformat()   
    #os.makedirs(
    folder_count=0
    for folders in os.listdir(path):  # loop over all files
        if os.path.isdir(os.path.join(path, folders)):  # if it's a directory
            folder_count += 1  # increment counter

    
    index=max(folder_count, index)
    print("starting index is ", index)
    os.makedirs('/mnt/tagmii1/'+datestamp+'/'+ampm, exist_ok=True) 
    
    
    global hand_hist
    is_hand_hist_created = False
#    capture = cv2.VideoCapture(0)

#    while capture.isOpened():
#        pressed_key = cv2.waitKey(1)
#        _, frame = capture.read()
#        frame = cv2.flip(frame, 1)

   
    idx=1
    s=1
    cam_was_down=0
    
    while True:
        key=None
        ok=0
        if(cam_was_down):
            print("waiting for camera to restart")
            time.sleep(3)
            ip='192.168.178.224'

            response = os.popen(f"ping -c 1 {ip}").read()

            if "1 received" not in response:

                print(f"Camera DOWW {ip} Ping Unsuccessful")
    
                continue


            cam = O3D(ip=args.ip, xmlrpc_port=args.xmlrpc_port)
            fg = FrameGrabber(cam, pcic_port=args.xmlrpc_port)

            fg.start( [buffer_id.AMPLITUDE_IMAGE, buffer_id.RADIAL_DISTANCE_IMAGE, buffer_id.CONFIDENCE_IMAGE, buffer_id.XYZ])
            cam_was_down=0            
       # fg = FrameGrabber(cam, pcic_port=args.xmlrpc_port)
        #fg.stop()
       # fg.start( [buffer_id.AMPLITUDE_IMAGE, buffer_id.RADIAL_DISTANCE_IMAGE, buffer_id.CONFIDENCE_IMAGE, buffer_id.XYZ] )
        #cv2.waitKey(-1) #wait until any key is pressed
       
         #     keepReceiving = True
    #     while keepReceiving:
    #        try:
        #data, fromAddr = sock.recvfrom(2048)
       ######################
       #        telegram = create_xml([])
#        print(telegram)
#        UDPClientSocket.sendto(telegram, client_address)
#        try:
#            bytesAddressPair = UDPClientSocket.recvfrom(4096)

#            ReceivedMessage = bytesAddressPair[0]
#            address = bytesAddressPair[1]
#            clientMsg = "Message from Client:{}".format(ReceivedMessage)
#    #        clientIP = "Client IP Address:{}".format(address)
#            print(clientMsg)
#            #print(clientIP)

#            #Receivetree = xml.parse(io.BytesIO(ReceivedMessage))

#    #        Received_root = Receivetree.getroot()
#    #        for child in Received_root:
#    #            print(child.tag, child.attrib)
#    #        
#            received_dict = extract_xml(ReceivedMessage, find='all')
#                
#            print("received:", received_dict)
#            
        
       # result = select.select([s],[],[])
        sent_mess_list = list()
        sent_mess_list.append({'ImAlive' : {'ImAlive' : 'ImAlive'}})
       # sent_mess_list.append({'Screenshot_request': '1'})
        telegram = create_xml(sent_mess_list)
             
        UDPClientSocket.sendto(telegram, ('127.0.0.1', 59155))
        #print (telegram)


#        msg = result[0][0].recv(bufferSize) 
        ##disablefornw## 


        
       # msg =UDPClientSocket.recv(4096) 

        mess_list = list()
        mess_list.append({'Sen' : {'Type' : 'ImNotFree'}})
                #                sent_mess_list.append({'XYZ1': camPosReceived1})
                #                sent_mess_list.append({'XYZ2': camPosReceived2})
                #                sent_mess_list.append({'XYZ3': camPosReceived3})
                #                sent_mess_list.append({'XYZ4': camPosReceived4})
        mess_list.append({'Screenshot': shoot})
        msg = create_xml(sent_mess_list)



        ip='192.168.178.224'

        response = os.popen(f"ping -c 1 {ip}").read()

        if "1 received" not in response:

             print(f"Camera DOWN {ip} Ping Unsuccessful")
             cam_was_down=1   
             continue
        #trigger_mode = cam.to_json()["ifm3d"]["Apps"][0]["TriggerMode"]

       # if trigger_mode == "1":
        #    print("Camera is in Continuous Trigger Mode")
        #elif trigger_mode == "2":
         #   print("Camera is in Software Trigger Mode")
        # Software Trigger the camera
        try:
            fg.sw_trigger()
        except:
            cam_was_down=1
        #print (msg)   
        received_dict = extract_xml(msg, find='all')
 
      
        if( 'Screenshot' in received_dict.keys()):
           
            Shoot=extract_xml(msg, find=['Screenshot'])
            #print("Shoot is ", Shoot)
            shoot=Shoot['Screenshot']
        #           if data:
        #              newestData = data
        #        except socket.error as why:
        #           if why.args[0] == EWOULDBLOCK:
        #              keepReceiving = False
        #           else:
        #              raise why
#        except:
#                    continue
       
       #################
        # Start timer#
        #print("ok to here")
        cam_pos = list()
        start_time = time.time()
        
        ip='192.168.178.224'
        
        #response = os.popen(f"ping -c 1 {ip}").read()

        #if "1 received" not in response:

         #    print(f"Camera DOWW {ip} Ping Unsuccessful")
    
          #   continue
        #fg.on_error()
        
       

        try:
#            fg.sw_trigger() 
#            fg.start([buffer_id.AMPLITUDE_IMAGE, buffer_id.RADIAL_DISTANCE_IMAGE, buffer_id.CONFIDENCE_IMAGE, buffer_id.XYZ])
            [ok, frame]=fg.wait_for_frame().wait_for(150)
            print("trying to get frame, success:", ok)
        except:
            print('could not connect to camera')
            cam_was_down=1
            continue
        else:
            print("fg status:", fg.is_running())
 #           continue
 #       fg.on_error(continue)
#        [ok, frame]=fg.wait_for_frame().wait_for(5)
        if ok:
            #print("OK!!", frame.timestamps()
            tmestamps=frame.timestamps()
            #continue
#        while not  :
#            continue
        if not ok:
            continue
            #raise RuntimeError("Timeout while waiting for a frame.")
        #frame1 = getter(buf)
        #NORM_AMPLITUDE_IMAGE, buffer_id.RADIAL_DISTANCE_IMAGE, buffer_id.XYZ]  )
         
      ####live script
        
        a=np.uint8(frame.get_buffer(buffer_id.AMPLITUDE_IMAGE))
       # frame1=frame.get_buffer(buffer_id.NORM_AMPLITUDE_IMAGE)
        frame1=np.uint8(frame.get_buffer(buffer_id.RADIAL_DISTANCE_IMAGE))
        #a=frame.get_buffer(buffer_id.NORM_AMPLITUDE_IMAGE)
        rows = a.shape[1]
        cols = a.shape[0]
#        print(rows)
#        print(cols)
#        print(a.max()) 
        
        for x in range(0, cols - 1):
            for y in range(0, rows -1):
                #print(a[x,y])
                pixel_value=a[x,y]
                if pixel_value>=16200:
                    a[x,y]=0
                if x>=215:
                    a[x,y]=0
#        print(a.max())     
        # Define the desired range
        array=a
        a, b = 0, 255

        # Min and max of the array
        array_min = np.min(array)
        array_max = np.max(array)

        # Normalize to the range [a, b]
        frame2 = a + ( (array - array_min) * (b - a) / (array_max - array_min) )
        #frame2=cv2.normalize(a, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        
        frame3=np.uint8(frame.get_buffer(buffer_id.CONFIDENCE_IMAGE))

        #print(frame2.shape)
        pt=frame.get_buffer(buffer_id.XYZ)
#        print("pt", np.max(pt))
        '''
        ###from file script
#        pth=
#        print(pth)
        root="/home/niall/Pictures/listons/2023_12_05/AM/"
        while (len(list(pathlib.Path(root+str(s)).glob("*_i"+str(s)+"_s"+str(idx)+"_amp.png")))==0) and s<1000:
            s=s+1
            #break
        
        pth=list(pathlib.Path(root+str(s)).glob("*_i"+str(s)+"_s"+str(idx)+"_amp.png"))[0]
            #idx=1i doesnt reset need to fox i recording program
        frame1=np.uint8(Image.open(pth))
        frame2=np.uint8(Image.open(str(pth).replace("amp", "dist")))
        frame3=np.uint8(Image.open(str(pth).replace("amp", "conf")))
        pt=np.load(str(pth).replace("amp.png", "pcd2.npy"))
        np.set_printoptions(threshold=sys.maxsize)
#        print(pt.shape)
#        print(pt)
        pt = pt.reshape((176,132,3))
        
        idx=idx+1
#        print(pt.shape)
#        print(pt)
        '''
        
        
        
         ###savevscript
        
        if(int(shoot) != prev_shoot  ):
            prev_shoot=int(shoot)
            if(int(shoot) == 1 ):
                index=index+1
                subindex=0
                print ("Index is", index)
                today = datetime.date.today()  
                datestamp=today.strftime("%Y_%m_%d")
                timestamp=today.strftime("%Y_%m_%d_%H_%M_%S")
                #todaystr = today.isoformat()   
                #os.makedirs(
                os.makedirs('/mnt/tagmii1/'+datestamp+'/'+ampm+'/'+str(index), exist_ok=True)
            
       
        #filestr="/media/niall/tagmii1/" + str(idx)
        filestr='/mnt/tagmii1/'+datestamp+'/'+ampm+'/'+str(index)+'/'+timestamp+'_i'+str(index)+'_s'+str(subindex)
  
  #uncooment here to save images   
         f1=frame1
         f2=frame2
         f3=frame3
         np.save(filestr+'_f1', f1)
         np.save(filestr+'_f2', f2)
         np.save(filestr+'_f3', f3)
         
         ptw = pt.reshape(pt.shape[:-3] + (-1, 3))
         np.save(filestr+'_pcd2', ptw)
#        im1 = Image.fromarray(frame1)
#            
#        im1.save(filestr+'_amp.png')
#        im2 = Image.fromarray(frame2)
#        im2.save(filestr+'_dist.png')
#        im3 = Image.fromarray(frame3)
#        im3.save(filestr+'_conf.png')
#        ptw = pt.reshape(pt.shape[:-3] + (-1, 3))
#        np.save(filestr+'_pcd2', ptw)
        idx=idx+1
#        frame1=cv2.normalize(frame.get_buffer(buffer_id.NORM_AMPLITUDE_IMAGE), None, 0,256, cv2.NORM_MINMAX, cv2.CV_8U)
#       # frame1=frame.get_buffer(buffer_id.NORM_AMPLITUDE_IMAGE)
#        frame2=cv2.normalize(frame.get_buffer(buffer_id.RADIAL_DISTANCE_IMAGE), None, 0,256, cv2.NORM_MINMAX, cv2.CV_8U)
#       frame3=cv2.normalize(frame.get_buffer(buffer_id.CONFIDENCE_IMAGE), None, 0,256, cv2.NORM_MINMAX, cv2.CV_8U)
#        pt=frame.get_buffer(buffer_id.XYZ)
        # Adjusts the brightness by adding 10 to each pixel value 
        brightness = -0 
        # Adjusts the contrast by scaling the pixel values by 2.3 
        contrast = 0.6
       # frame1 = cv2.addWeighted(frame1, contrast, np.zeros(frame1.shape, frame1.dtype), 0, brightness) 
        #frame1 = cv2.addWeighted(frame2, 0.6, np.zeros(frame2.shape, frame2.dtype), 0, 0) 
        frame1=frame2*0.8
        #frame3 = cv2.addWeighted(frame3, -1.0, np.zeros(frame3.shape, frame3.dtype), 0, 60)   
        
        
        c = frame3# np.array([1, 4, 2, 5, 7, 4, 2, 5, 6, 7, 7, 2, 5])
        mapping = np.arange(c.max() + 1)
        map_from = np.array([45, 48, 56,  61])
        map_to = np.array([15, 60, 180, 180])
        mapping[map_from] = map_to
        frame3=np.uint8(mapping[c])
        #pressed_key = cv2.waitKey(1)
        #cv2.imshow("teat_detector", frame1)
        image_np = live_image_into_numpy_array(frame1, frame3, frame2)#bgr
         # Things to try:231
    # Flip horizontally
        # image_np = np.fliplr(image_np).copy()

        # Convert image to grayscale
        # image_np = np.tile(
        #     np.mean(image_np, 2, keepdims=True), (1, 1, 3)).astype(np.uint8)
        
        # The input needs to be a tensor, convert it using `tf.convert_to_tensor`.
        input_tensor = tf.convert_to_tensor(image_np)
        # The model expects a batch of images, so add an axis with `tf.newaxis`.
        input_tensor = input_tensor[tf.newaxis, ...]

        detections = detect_fn(input_tensor)

        # All outputs are batches tensors.
        # Convert to numpy arrays, and take index [0] to remove the batch dimension.
        # We're only interested in the first num_detections.
        num_detections = int(detections.pop('num_detections'))
        detections = {key: value[0, :num_detections].numpy()
                      for key, value in detections.items()}
        detections['num_detections'] = num_detections
        

        # detection_classes should be ints.
        detections['detection_classes'] = detections['detection_classes'].astype(np.int64)
       # print(int(frame1.shape[1]))
        cam_pos = list()
        defects=list()
        boxes=detections['detection_boxes'] 
        if(len(boxes)!=0):
            #print (boxes)
            indices = np.argsort(boxes[:,0])
            #print(indices)
    #Then you can use these indices to sort the boxes, scores, and classed, as follows:

    #sorted_scores = scores[0][indices]
            sorted_boxes = boxes#[0][indices]
            #detections['detection_classes'] = detections['detection_classes'][0] [indices]
    #If you wanted to, for example, sort by xmax instead, you'd use np.argsort(boxes[0][:,2]). You can play around with using 0-3 to sort by xmin, ymin, xmax, and ymax.
            #print(int(frame1.shape[1]))
            width = int(frame1.shape[1])
            height= int(frame1.shape[0])
            #print(pt)
            if(len(boxes)!=0):
                #print (detections)
                indices = np.argsort(boxes[:,0])
                #print(indices)
        #Then you can use these indices to sort the boxes, scores, and classed, as follows:

        #sorted_scores = scores[0][indices]
                sorted_boxes = boxes#[0][indices]
                #detections['detection_classes'] = detections['detection_classes'][0] [indices]
        #If you wanted to, for example, sort by xmax instead, you'd use np.argsort(boxes[0][:,2]). You can play around with using 0-3 to sort by xmin, ymin, xmax, and ymax.
                #print(int(frame1.shape[1]))
                width = int(frame1.shape[1])
                height= int(frame1.shape[0])
                #print(pt)
                for i in range(len(boxes)):
                   if detections['detection_classes'][i]<=2:
                       
                       ymin = int((boxes[i,0]*height))
                       xmin = int((boxes[i,1]*width))
                       ymax = int((boxes[i,2]*height))
                       xmax = int((boxes[i,3]*width))
    #                   xmin = (int(boxes[i,0]*width))+4
    #                   ymin = (int(boxes[i,1]*height))
    #                   xmax = (int(boxes[i,2]*width))
    #                   ymax = (int(boxes[i,3]*height))
                      ### print(xmin, xmax, ymin, ymax)
                       a=int(xmin+(xmax-xmin)/2)
                       b=int(ymin+((ymax-ymin)/2))
                       
                       
                       x=pt[b, a][0]
                       y=pt[b, a][1]
                       z=pt[b, a][2]
                     #j  print(x,y,z)
                       c=0
                       
    #                   for alpha in range(xmin, xmax):
    #                    for beta in range(ymin, ymax):
                       #3d    
                       while(x==0 and c<5):
                        b=b-1
                        c=c+1
                        x=pt[b, a][0]
                        y=pt[b, a][1]
                        z=pt[b, a][2]
                       if a>70 and a <105:# and b >25 and b<70 and ymax-ymin<22 and xmax-xmin <18 and z<950 and z>550:
                           #print("sending", boxes, detections['detection_classes'][i], a, b, x, y, z)
                           cam_dict={'XYZ'+str(i+1): {'X': str(x), 'Y': str(y),'Z': str(z)}}
                                                     #cam_pos.append(cam_dict)
                           cam_pos.append(cam_dict)
                           defects.append([a, b])
                           #break
                       else:
                             detections['detection_scores'][i]=0
    #                        
                        
                   
#                   print("sending", boxes, detections['detection_classes'][i], a, b, x, y, z)
#                   cam_dict={'XYZ'+str(i+1): {'X': str(x), 'Y': str(y),'Z': str(z)}}
#                                             #cam_pos.append(cam_dict)
#                   cam_pos.append(cam_dict)
                   #break
        
#                   print("sending", boxes, detections['detection_classes'][i], a, b, x, y, z)
#                   cam_dict={'XYZ'+str(i+1): {'X': str(x), 'Y': str(y),'Z': str(z)}}
#                                             #cam_pos.append(cam_dict)
#                   cam_pos.append(cam_dict)
#        # bit for sending telegram of obj results
#        if(len(cam_pos) != 0):
#            print(cam_pos)
#            telegram = create_xml(cam_pos)
#                
#            UDPClientSocket.sendto(telegram, ('127.0.0.1', 59152))
#            print("sent", telegram)
        #print(defects)
        process_teats(defects[:4], pt)
        #print("got to here")
        image_np_with_detections = image_np.copy()

#       # viz_utils.visualize_boxes_and_labels_on_image_array(
#              image_np_with_detections,
#              detections['detection_boxes'],
#              detections['detection_classes'],
#              detections['detection_scores'],
#              category_index,
#              use_normalized_coordinates=True,
#              max_boxes_to_draw=8,
#              min_score_thresh=.25,
#              agnostic_mode=False)

        #plt.figure()
#        cv2.imshow("teat_detector", image_np_with_detections)
#        #plt.savefig('books_read.png')
        print('Done')
#        plt.show()
        #pressed_key = cv2.waitKey(1)
#        if pressed_key & 0xFF == ord('z'):
            #print("tracking started")
#        is_hand_hist_created = True
#        hand_hist = hand_histogram(frame1)
        if(shoot != prev_shoot):
            index=index+1
            prev_shoot=shoot
            
            today = datetime.date.today()  
            datestamp=today.strftime("%Y_%m_%d")
            timestamp=today.strftime("%Y_%m_%d_%H_%M_%S")
            #todaystr = today.isoformat()   
            #os.makedirs(
            os.makedirs('/mnt/tagmii1/'+datestamp, exist_ok=True) 
        ###hand_method
        '''
        if is_hand_hist_created:
            manage_image_opr(frame1, frame2, pt,  hand_hist)

        else:
            frame = draw_rect(frame1)
            is_hand_hist_created = True
            hand_hist = hand_histogram(frame1)
#            
        '''
       # cv2.imshow("teat_detector", frame1)#rescale_frame(
        #print(pt)
#        if(shoot):
#            
#            subindex=subindex+1
#            
#           # os.mkdir(todaystr)
#            cv2.imwrite('/home/niall/ifm3d/data/'+datestamp+'/'+timestamp+'_i'+str(index)+'_s'+str(subindex)+'_amp.png', frame1)
#            cv2.imwrite('/home/niall/ifm3d/data/'+datestamp+'/'+timestamp+'_i'+str(index)+'_s'+str(subindex)+'_dist.png', frame2)
#            pt = pt.reshape(pt.shape[:-3] + (-1, 3))
#            print(pt.shape)
#            pcd = o3d.geometry.PointCloud()
#            pcd.points = o3d.utility.Vector3dVector(pt)
#            #o3d.io.write_point_cloud("./data.ply", pcd)
#            #o3d.visualization.draw_geometries([pcd])
#            o3d.io.write_point_cloud('/home/niall/ifm3d/data/'+datestamp+'/'+timestamp+'_i'+str(index)+'_s'+str(subindex)+'_pcd.pcd', pcd)
#        
#            
#            cv2.imwrite('/home/niall/ifm3d/data/'+datestamp+'/'+timestamp+'_i'+str(index)+'_s'+str(subindex)+'_detections.png', frame1)
##        
#        if pressed_key == 27:
#            break
#        
        
        # End timer
        end_time = time.time()

#        # Calculate elapsed time
        elapsed_time = end_time - start_time

       # print("Elapsed time is ", elapsed_time)

   # capture.release()



if __name__ == '__main__':
    main()
'''
for image_path in IMAGE_PATHS:

    print('Running inference for {}... '.format(image_path), end='')

    image_np = load_image_into_numpy_array(image_path)

    # Things to try:
    # Flip horizontally
    # image_np = np.fliplr(image_np).copy()

    # Convert image to grayscale
    # image_np = np.tile(
    #     np.mean(image_np, 2, keepdims=True), (1, 1, 3)).astype(np.uint8)

    # The input needs to be a tensor, convert it using `tf.convert_to_tensor`.
    input_tensor = tf.convert_to_tensor(image_np)
    # The model expects a batch of images, so add an axis with `tf.newaxis`.
    input_tensor = input_tensor[tf.newaxis, ...]

    detections = detect_fn(input_tensor)

    # All outputs are batches tensors.
    # Convert to numpy arrays, and take index [0] to remove the batch dimension.
    # We're only interested in the first num_detections.
    num_detections = int(detections.pop('num_detections'))
    detections = {key: value[0, :num_detections].numpy()
                  for key, value in detections.items()}
    detections['num_detections'] = num_detections

    # detection_classes should be ints.
    detections['detection_classes'] = detections['detection_classes'].astype(np.int64)

    image_np_with_detections = image_np.copy()

    viz_utils.visualize_boxes_and_labels_on_image_array(
          image_np_with_detections,
          detections['detection_boxes'],
          detections['detection_classes'],
          detections['detection_scores'],
          category_index,
          use_normalized_coordinates=True,
          max_boxes_to_draw=200,
          min_score_thresh=.30,
          agnostic_mode=False)

    plt.figure()
    plt.imshow(image_np_with_detections)
    plt.savefig('books_read.png')
    print('Done')
plt.show()
'''
# sphinx_gallery_thumbnail_number = 2
def sort_points(pts):
    centroid = np.sum(pts, axis=0) / pts.shape[0]
    vector_from_centroid = pts - centroid
    vector_angle = np.arctan2(vector_from_centroid[:, 1], vector_from_centroid[:, 0]) 
    sort_order = np.argsort(vector_angle) # Find the indices that give a sorted vector_angle array

    # Apply sort_order to original pts array. 
    # Also returning centroid and angles so I can plot it for illustration. 
    return (pts[sort_order, :], centroid, vector_angle[sort_order])
