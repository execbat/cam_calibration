#!/usr/bin/env python3
#import cv2
import numpy as np
import math
import socket
import io
import xml.etree.ElementTree as xml

import ifm3dpy
from ifm3dpy.device import O3D
from ifm3dpy.framegrabber import FrameGrabber, buffer_id

#from ifm3dpy import O3DCamera, FrameGrabber, ImageBuffer
#import cv2
import argparse
#from mpl_toolkits.mplot3d import Axes3D
#import numpy as np
from scipy.spatial.transform import Rotation as R
import _thread
import time, datetime
import select
import os

from PIL import Image

import cv2
import argparse

#matplotlib notebook
#import ifm3dpy
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.spatial.transform import Rotation as R
from apriltag.scripts import apriltag

#O3D.get(["/device/clock/sntp/availableServers"])
#    # Output of above line: {'device': {'clock': {'sntp': {'availableServers': []}}}}
#O3D.set({'device': {'clock': {'sntp': {'availableServers': ["192.168.178.68"]}}}})
#O3D.get(["/device/clock/sntp/availableServers"])
#    # Output of above line: {'device': {'clock': {'sntp': {'availableServers': ['192.168.0.110']}}}}
#O3D.get(["/device/clock/currentTime"])
#    # Output of above line: {'device': {'clock': {'currentTime': 1676037142019902053}}}
#print(datetime.datetime.utcfromtimestamp(O3D.get(["/device/clock/currentTime"])["device"]["clock"]["currentTime"] // 1000000000))
#    #print(datetime.datetime(2023, 2, 10, 14, 2, 12) # System time with NTP connection)

#import ntplib
from datetime import datetime, date, timezone
#c = ntplib.NTPClient()
## Provide the respective ntp server ip in below function
#response = c.request('ie.pool.ntp.org', version=3)
#response.offset
#print (datetime.fromtimestamp(response.tx_time, timezone.utc))


#import struct
#import sys
#import time

#def RequestTimefromNtp(addr='0.ie.pool.ntp.org'):
#    REF_TIME_1970 = 2208988800  # Reference time
#    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#    data = b'\x1b' + 47 * b'\0'
#    client.sendto(data, (addr, 123))
#    data, address = client.recvfrom(1024)
#    if data:
#        t = struct.unpack('!12I', data)[10]
#        t -= REF_TIME_1970
#    return time.ctime(t), t

#print(RequestTimefromNtp())
#import pandas as pd

#import pyntcloud
#from pyntcloud import PyntCloud

image_choices = ["jpeg", "distance", "amplitude"]
#if OPEN3D_AVAILABLE:
image_choices += ["xyz"]
#if OPEN3D_AVAILABLE:
#    image_choices += ["xyz"]
#    
buf = buffer_id

#UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
#UDPClientSocket.bind(('127.0.0.1', 59157))  # Инициализирует ip-адрес и порт.
##UDPClientSocket.bind(('127.0.0.1', 59152))  # Инициализирует ip-адрес и порт.

##client_address=('127.0.0.1', 59152)

#print(UDPClientSocket)



###UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
###UDPServerSocket.bind(('192.168.1.10', 59152))  # Инициализирует ip-адрес и порт.

### Receive-UDP-data event-loop begins her
##UDPClientSocket.setblocking(True)
#s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
#s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
##s.bind(('',59155))
#s.setblocking(0)
#print('UDP server is turning on')

def SendData(Transmit,OpenedSocket, target_address):
    sent = OpenedSocket.sendto(Transmit, target_address)


def extract_xml(bytes_parcel, find = 'all'): # 'find' could be also the list of key-words
    res_dict = dict()
    #print(   bytes_parcel)
    try: # checkking if we can parse input data to ElementTree
        tree = xml.parse(io.BytesIO(bytes_parcel)) # xml.etree.ElementTree.ElementTree

    except TypeError:
        do_nothing=0#print('incorrect type of input data')

    else:
        try: # checking 'find'
            find = find.lower()
            

        except AttributeError: #operating with list
            root = tree.getroot()
            
            #print('cant make lowercase')
            for i_tag in find:
                elem = tree.find(i_tag)                
                try: # if incorrect key_word in list
                    if elem.text is not None:
                        res_dict[elem.tag] = elem.text    # if element contains text
                    else:
                        res_dict[elem.tag] = elem.attrib  # if element contains dict
                except AttributeError:
                    print('Data do not contain "{}" key-word'.format(i_tag))


        else: #checking ALL
            if find == 'all':
                root = tree.getroot()
                if root.text is not None:        
                    res_dict[root.tag] = root.text    # if element contains text
                else:
                    res_dict[root.tag] = root.attrib  # if element contains dict

                for elem in root:
                    if elem.text is not None:        
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
    DecodedTree = xml.tostring(root, encoding='utf8', method='xml', short_empty_elements='true')
    
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


#def get_jpeg(buf):
#    return buf.amplitude_image()


#def get_distance(buf):
#    #img = cv2.normalize(buf.distance_image(), None, 0,
#                        #255, cv2.NORM_MINMAX, cv2.CV_8U)
#    #img = cv2.applyColorMap(img, cv2.COLORMAP_JET)
#    return buf.distance_image()


#def get_amplitude(buf):
#    return buf.amplitude_image() #amp = 


#    shade = cv2.medianBlur(amp, 13)
#    #shade = cv2.cvtColor(shade, cv2.COLOR_BGR2GRAY)
#    #shade = cv2.cvtColor(shade, cv2.COLOR_GRAY2BGR)

#    flat = (amp+(1-(shade/255))*~shade*2)/2
#    norm = flat*255/np.max(flat)

#    #cv2.imwrite(pth+'/shade.jpg', shade)
#    amp = np.hstack((amp, flat))
#    out = np.hstack((amp, norm))
#    #cv2.imwrite(pth+'/Flatten.jpg', np.vstack((org, out)))
##    return cv2.cvtColor(cv2.normalize(buf.amplitude_image(), None, 0,
##                        255, cv2.NORM_MINMAX, cv2.CV_8U), cv2.COLOR_GRAY2RGB)#.astype(np.float32)
#    return cv2.cvtColor(out, cv2.COLOR_GRAY2RGB)#.astype(np.float32)
    
def get_xyz(buf):
    return buf.xyz_image()


#def display_2d(fg, buf, getter, title, index):
    
        

 
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcic-port", help="The pcic port from which images should be received", type=int,
                        required=False, default=50010)
    parser.add_argument("--image", help="The image to received (default: distance)", type=str,
                        choices=image_choices, required=False, default="amplitude")
    parser.add_argument("--ip", help="IP address of the sensor (default: 192.168.178.69)",
                        type=str, required=False, default="192.168.178.201")
    parser.add_argument("--xmlrpc-port", help="XMLRPC port of the sensor (default: 50010)",
                        type=int, required=False, default=50010)
    parser.add_argument("--index", help="results file index",
                        type=str, required=False, default=0)  
                  
    args = parser.parse_args()                  
    args = parser.parse_args()
    
    #getter = globals()["get_" + args.image]
    
    index=1
    subindex=0
    
    today = date.today()  
    datestamp=today.strftime("%Y_%m_%d")
    timestamp=today.strftime("%H_%M_%S")
    ampm=today.strftime("%p")
    print(datestamp)
    os.makedirs('./data/'+datestamp+'/'+ampm, exist_ok=True) 
    
    path='./data/'+datestamp+'/'+ampm+'/'
    isExist = os.path.exists(path+str(index))
    while isExist:
        index=index+1
        path='./data/'+datestamp+'/'+ampm+'/'+str(index)
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
    os.makedirs('./data/'+datestamp+'/'+ampm, exist_ok=True)    
    #O3D = O3D(ip=args.ip, pcic_port=args.pcic_port)
    #fg = FrameGrabber(O3D, pcic_port=50012)
    print(dir(buffer_id))
    ###['ALGO_DEBUG', 'AMPLITUDE_IMAGE', 'CARTESIAN_ALL', 'CARTESIAN_X_COMPONENT', 'CARTESIAN_Y_COMPONENT', 'CARTESIAN_Z_COMPONENT', 'CONFIDENCE_IMAGE', 'DIAGNOSTIC', 'EXPOSURE_TIME', 'EXTRINSIC_CALIB', 'GRAYSCALE_IMAGE', 'ILLUMINATION_TEMP', 'INTRINSIC_CALIB', 'INVERSE_INTRINSIC_CALIBRATION', 'JPEG_IMAGE', 'JSON_DIAGNOSTIC', 'JSON_MODEL', 'MONOCHROM_2D', 'MONOCHROM_2D_12BIT', 'NORM_AMPLITUDE_IMAGE', 'O3D_DISTANCE_IMAGE_INFO', 'O3D_ODS_INFO', 'O3D_ODS_OCCUPANCY_GRID', 'O3D_RGB_IMAGE_INFO', 'RADIAL_DISTANCE_IMAGE', 'RADIAL_DISTANCE_NOISE', 'REFLECTIVITY', 'RGB_INFO', 'TOF_INFO', 'UNIT_VECTOR_ALL', 'XYZ', '__class__', '__delattr__', '__dir__', '__doc__', '__entries', '__eq__', '__format__', '__ge__', '__getattribute__', '__getstate__', '__gt__', '__hash__', '__index__', '__init__', '__init_subclass__', '__int__', '__le__', '__lt__', '__members__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__setstate__', '__sizeof__', '__str__', '__subclasshook__', 'name', 'value']
    cam = O3D(ip=args.ip, xmlrpc_port=args.xmlrpc_port)
    fg = FrameGrabber(cam, pcic_port=args.xmlrpc_port)

    fg.start(
        [buffer_id.NORM_AMPLITUDE_IMAGE, buffer_id.RADIAL_DISTANCE_IMAGE, buffer_id.CONFIDENCE_IMAGE, buffer_id.XYZ]
    )
    title = "O3D Port {}".format(str(args.pcic_port))
    
    
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
     
    global hand_hist
    is_hand_hist_created = False
#    capture = cv2.VideoCapture(0)

#    while capture.isOpened():
#        pressed_key = cv2.waitKey(1)
#        _, frame = capture.read()
#        frame = cv2.flip(frame, 1)

    #cv2.startWindowThread()
   # cv2.namedWindow("teat_detector", cv2.WINDOW_NORMAL)
    
#    def rec_UDP():
#        while True:
#            # UDP commands for listening
##            UDP_PORT = 5005
##            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
##            sock.bind(('10.0.0.15', UDP_PORT))
#            
#            
#            data, addr = UDPClientSocket.recvfrom(4096)
#            print ("received message:", data)
#            return data


#    # The thread that ables the listen for UDP packets is loaded
#    listen_UDP = threading.Thread(target=rec_UDP)
#    listen_UDP.start()
    sent_mess_list = list()
    sent_mess_list.append({'Sen' : {'Type' : 'ImNotFree'}})
    sent_mess_list.append({'Screenshot': '0'})#_request
    telegram = create_xml(sent_mess_list)
         
#    UDPClientSocket.sendto(telegram, ('127.0.0.1', 59155))
    cv2.startWindowThread()
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    detector = apriltag.Detector(searchpath=apriltag._get_dll_path())
    i=0
    while True:
        key=None
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
        
        #print (telegram)
#        msg = result[0][0].recv(bufferSize) 
        #msg =UDPClientSocket.recv(4096) 
        msg=[]
        #print("got mesg")
        #print (msg)   
        received_dict = extract_xml(msg, find='all')
        ##cam_dict
        if( 'Screenshot' in received_dict.keys()):
            
            Shoot=extract_xml(msg, find=['Screenshot'])
           # print("Shoot is ", int(shoot))
            shoot=float(Shoot['Screenshot'])
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
        # Start timer
        start_time = time.time()
        [ok, frame]=fg.wait_for_frame().wait_for(5)
        if ok:
            print("OK!!", frame.timestamps())
#        while not  :
#            continue
        if not ok:
            continue
            #raise RuntimeError("Timeout while waiting for a frame.")
        #frame1 = getter(buf)
        #NORM_AMPLITUDE_IMAGE, buffer_id.RADIAL_DISTANCE_IMAGE, buffer_id.XYZ]  )
        a=frame.get_buffer(buffer_id.NORM_AMPLITUDE_IMAGE)
        rows = a.shape[1]
        cols = a.shape[0]
        print(rows)
        print(cols)
        print(a.max()) 
        
        for x in range(0, cols - 1):
            for y in range(0, rows -1):
                #print(a[x,y])
                pixel_value=a[x,y]
                if pixel_value>=3000:
                    a[x,y]=3000
                  #  print(pixel_value)
                if x>=235:
                    a[x,y]=0
        print(a.max())     
        frame1=cv2.normalize(a, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        
        #frame1=np.uint8(frame.get_buffer(buffer_id.NORM_AMPLITUDE_IMAGE))
       # frame1=frame.get_buffer(buffer_id.NORM_AMPLITUDE_IMAGE)
        frame2=np.uint8(frame.get_buffer(buffer_id.RADIAL_DISTANCE_IMAGE))
        frame3=np.uint8(frame.get_buffer(buffer_id.CONFIDENCE_IMAGE))
#        frame1=frame.get_buffer(buffer_id.NORM_AMPLITUDE_IMAGE)
#        frame2=frame.get_buffer(buffer_id.RADIAL_DISTANCE_IMAGE)
#        frame3=frame.get_buffer(buffer_id.CONFIDENCE_IMAGE)
        pt=frame.get_buffer(buffer_id.XYZ)
        
        
        result, overlay = apriltag.detect_tags(frame1,
                                                   detector,
                                                   camera_params=(3156.71852, 3129.52243, 359.097908, 239.736909),
                                                   tag_size=0.2,
                                                   vizualization=3,
                                                   verbose=0,
                                                   annotation=True
                                                  )
        
        print(f'result {result} {type(result)}')
        
            
        
        if len(result) > 0:
        T_mtx = result[1]
        print("T_mtx", T_mtx)
        print(f"T_mtx type : {type(T_mtx)}")
        
        
        sumx=0
        sumy=0 
        sumz=0  
        crnrs=0   
        for corner in result[0].corners:
                    
            x=pt[int(corner[1]), int(corner[0])][0]
            y=pt[int(corner[1]), int(corner[0])][1]
            z=pt[int(corner[1]), int(corner[0])][2]
            #print(pt[int(corner[1]), int(corner[0])]) #, file = sourceFile)
            #print(pt[int(corner[1]), int(corner[0])])
            if (x!=0 or y!=0 or z!=0):
                sumx+=x
                sumy+=y
                sumz+=z
                crnrs=crnrs+1
            else:
                print('issue')
                pass
                            
                            
                        
        if crnrs==4:           
            centroid = (sumx / 4, sumy / 4, sumz/4)
            print("calculated center")
            print(centroid)
            
            T_mtx[:3, -1] = centroid
            print(f"updated T_mtx: {T_mtx}")     
        
        
        
        
        
        

            
        #key = cv2.waitKey(20000)
        cv2.imshow(title, overlay)
        
        #frame=np.expand_dims(frame, axis=0)
        #frame=frame[:, :, 3]#
        #frame = cv2.flip(frame, 0)
        
            
        
       # pressed_key = cv2.waitKey(1)
       
       
#        if pressed_key & 0xFF == ord('z'):
            #print("tracking started")
#        is_hand_hist_created = True
#        hand_hist = hand_histogram(frame1)
        if(int(shoot) is not prev_shoot  ):
            prev_shoot=int(shoot)
            if(int(shoot) is 1 ):
                index=index+1
                subindex=0
                print ("Index is", index)
                today = datetime.now()  
                datestamp=today.strftime("%Y_%m_%d")
                timestamp=today.strftime("%Y_%m_%d_%H_%M_%S")
                #todaystr = today.isoformat()   
                #os.makedirs(
                os.makedirs('./data/'+datestamp+'/'+ampm+'/'+str(index), exist_ok=True)
            
#        if is_hand_hist_created:
#            manage_image_opr(frame1, frame2, pt,  hand_hist)

#        else:
#            #frame = draw_rect(frame1)
#            is_hand_hist_created = True
#            hand_hist = hand_histogram(frame1)
            

#        cv2.imshow("teat_detector", rescale_frame(frame1))
        #print(pt)
        shoot=1
        if(int(shoot)):
        ########################    
#            subindex=subindex+1
#            filestr='/home/niall/data/'+datestamp+'/'+ampm+'/'+str(index)+'/'+timestamp+'_i'+str(index)+'_s'+str(subindex)
#           # os.mkdir(todaystr)
#            im1 = Image.fromarray(frame1)
#            
#            im1.save(filestr+'_amp.png')
#            im2 = Image.fromarray(frame2)
#            im2.save(filestr+'_dist.png')
#            im3 = Image.fromarray(frame3)
#            im3.save(filestr+'_conf.png')
#            ptw = pt.reshape(pt.shape[:-3] + (-1, 3))
#            print(ptw.shape)
#            #pt=buf.xyz_image()
#          #  center=result[0].center
#####            sourceFile = open("data/"+str(index)+str(i), 'w')
#####            print(Receivedict, file = sourceFile)
#####            print(result, file = sourceFile)

#####            print("pcl center", file = sourceFile)   
#####            print("pcl center")
#            #pt=buf.xyz_image()
#           
#            
##            pcd = o3d.geometry.PointCloud()
##            pcd.points = o3d.utility.Vector3dVector(pt)

#            ####data=np.hstack((pt, frame1))
#            #data = pd.DataFrame(pt, columns=["x", "y", "z"])
#            #o3d.io.write_point_cloud("./data.ply", pcd)
#            #o3d.visualization.draw_geometries([pcd])
#           # o3d.io.write_point_cloud('/media/niall/tagmii1/data'+datestamp+'/'+timestamp+'_i'+str(index)+'_s'+str(subindex)+'_pcd.pcd', pcd)
#            
#            
#            
#            
#            #cloud = PyntCloud(data)
#           
#    # same arguments that you are passing to visualize_pcl

#            #cloud.to_file('/media/niall/tagmii1/data'+datestamp+'/'+timestamp+'_i'+str(index)+'_s'+str(subindex)+'_pcd.ply')
#            np.save(filestr+'_pcd2', ptw)
#            #np.savez_compressed('/media/niall/tagmii1/data'+datestamp+'/'+timestamp+'_i'+str(index)+'_s'+str(subindex)+'_pcd3', pt)
#            #cv2.imwrite('/media/niall/tagmii1/data'+datestamp+'/'+timestamp+'_i'+str(index)+'_s'+str(subindex)+'_detections.png', frame1)
#        #####################
#        if pressed_key == 27:
#            break
            key = cv2.waitKey(20)
            if key == ord('q'):
                break
            
            if key ==ord('p'):
              #cv2.waitKey(-1) #wait until any key is pressed
              print(key)
              if (len(result)>0):
                if(result):
                    #print("got result")
                    #key = cv2.waitKey(20000)
                    i=i+1         
                   # pt=buf.xyz_image()
                    center=result[0].center
                    #print(client_address)
        ###            sent_mess_list = list()
        ###            sent_mess_list.append({'cam' : '123'})
        ###            
        ###            telegram = create_xml(sent_mess_list)
        ###            
        ###            UDPClientSocket.sendto(telegram, client_address)
        ###            print(sent_mess_list)
        ###            print(telegram)
        ###            bytesAddressPair = UDPClientSocket.recvfrom(4096)
        ####        
        ###            ReceivedMessage = bytesAddressPair[0]
        ###            address = bytesAddressPair[1]
        ###            clientMsg = "Message from Client:{}".format(ReceivedMessage)
        ###            clientIP = "Client IP Address:{}".format(address)
        ###            #print(clientMsg)
        ###            #print(clientIP)
                    print("getting robot message")
                    #print(ReceivedMessage)
                    #Receivedict = extract_xml(msg, find='all')
                    #sourceFile = open("data/"+str(index)+str(i), 'w')
                    sourceFile = open("data/"+str(index)+str(i), 'a')
                    print(received_dict, file = sourceFile)
                    print(result, file = sourceFile)
                    #sourceFile.close()
                    print("pcl center", file = sourceFile)   
                    print("pcl center")
                   # pt=buf.xyz_image()
                    center=result[0].center
        #                pt=get_pcl_by_intrinsics(buf)
        #                #print(result[0].center, file = sourceFile)
        #                center=result[0].center
        #                #print(pt.shape)
        #                print(pt[int(center[1]), int(center[0])], file = sourceFile)
        #                print(pt[int(center[1]), int(center[0])])
        #                print("center") 
                    
        #                pt=buf.xyz_image()
                    #pt=get_pcl_by_intrinsics(buf)
                    #print(result[0].center, file = sourceFile)
        #                center=result[0].center
                    #print(pt.shape)
                    
                    
                    print(pt[int(center[1]), int(center[0])], file = sourceFile)
                    print(pt[int(center[1]), int(center[0])])
                    print("corners", file = sourceFile)  
                    
        #                cam_dict={'XYZ1': {'X': str(pt[int(center[1]), int(center[0])][0]*1000), 'Y': str(pt[int(center[1]), int(center[0])][1]*1000),'Z': str(pt[int(center[1]), int(center[0])][2]*1000)}}
                    
                    
                    #sent_mess_list.append({'XYZ1': {'X': str(-pt[int(center[1]), int(center[0])][0]), 'Y': str(-pt[int(center[1]), int(center[0])][1]),'Z': str(pt[int(center[1]), int(center[0])][2])}})    
                    sumx=0
                    sumy=0 
                    sumz=0     
                    for corner in result[0].corners:
                            
                                x=pt[int(corner[1]), int(corner[0])][0]
                                y=pt[int(corner[1]), int(corner[0])][1]
                                z=pt[int(corner[1]), int(corner[0])][2]
                                print(pt[int(corner[1]), int(corner[0])], file = sourceFile)
                                print(pt[int(corner[1]), int(corner[0])])
                                sumx+=x
                                sumy+=y
                                sumz+=z
                               
                    centroid = (sumx / 4, sumy / 4, sumz/4)
                    print("calculated center")
                    print(centroid)
                    print(centroid, file = sourceFile)
                cam_dict={'XYZ1': {'X': str(centroid[0]), 'Y': str(centroid[1]),'Z': str(centroid[2])}}        
                sourceFile.close()
                #cam_dict={'XYZ1' : {'X' : '1', 'Y' : '2', 'Z' : '3'}}
                cam_pos = list()
                        
                cam_pos.append(cam_dict)
                telegram = create_xml(cam_pos)
                print("xyz1")
                print(telegram)
                UDPClientSocket.sendto(telegram, ('127.0.0.1', 59152))
                UDPClientSocket.sendto(telegram, ('127.0.0.1', 59155))
        # End timer
        end_time = time.time()

#        # Calculate elapsed time
        elapsed_time = end_time - start_time

       # print("Elapsed time is ", elapsed_time)
    cv2.destroyAllWindows()
    capture.release()
    apriltag.release()



if __name__ == '__main__':
    main()
