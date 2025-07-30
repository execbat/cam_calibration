

from IPython import display
from collections import deque

# import socket
# import threading
from multiprocessing import Manager, Process, Queue

import time
from lib_xml_tree import *
from lib_connection import *
from lib_config_loader import load_config
from lib_manager_buffer import *


def routine_for_camera(server_address, port, mess_input, mess_output):
    conn = create_socket(server_address, port)
    
    # init variables supposed to send to camera
    watchDog_out = '0' 
    frame_assigned = '0' 
    position_reached = '0' 
    need_cam_cal = '0'        
    screenshot = '0' 
    
    
    while True:
        telegram_from_robot = None
        try:
            # get message from robot (dict)
            # if len(mess_input) > 0:
            # if mess_input.has_message():
            ################################################
            #RECEIVING MESSAGES
            ################################################ 
            
            # GET MESSAGE FROM ROBOT SERVER-SUBPROCESS
            telegram_from_robot = mess_input.get()
            if telegram_from_robot is not None:
                # extract data from telegram
                try:
                    watchDog_out = telegram_from_robot["WatchDog_out"]
                    frame_assigned = telegram_from_robot["Frame_assigned"]
                    position_reached = telegram_from_robot["Position_reached"]
                    need_cam_cal = telegram_from_robot["Need_cam_cal"]
                    screenshot = telegram_from_robot["Screenshot"]
                    DEF_RIst = telegram_from_robot['DEF_RIst']
                    ipoc = telegram_from_robot["IPOC"]
                    
                except Exception as ex:
                    print(f" Data can not be extracted from telegram from robot: {ex}")                
            # else:
            #     print('camera has no messages from robot')

            # GET MESSAGE FROM CAMERA CLIENT  
            bytesAddressPair = conn.recvfrom(4096)            
            ReceivedMessage = bytesAddressPair[0]
            received_address = bytesAddressPair[1]
            # # clientMsg = "Message from { } : {}".format(received_address, ReceivedMessage)
            # clientIP = "Client IP Address:{}".format(received_address)
            # # print(clientMsg)
            # # print(clientIP)    
            received_dict = extract_xml(ReceivedMessage)



            ################################################
            #SENDING MESSAGES
            ################################################           

            # SEND MESSAGE TO ROBOT SERVER-SUBPROCESS
            mess_output.put(received_dict)
            
                     
            # SEND MESSAGE TO CAMERA CLIENT
            sent_mess_list = list() # This message to be sent from Server to Camera client
            sent_mess_list.append({"Sen" : {'Type' : 'ServerToCamera'}})    
            sent_mess_list.append({"WatchDog_out" : watchDog_out})  
            sent_mess_list.append({'Frame_assigned': frame_assigned})
            sent_mess_list.append({'Position_reached': position_reached})
            sent_mess_list.append({'Need_cam_cal': need_cam_cal})
            sent_mess_list.append({'Screenshot': screenshot})     
            sent_mess_list.append({'DEF_RIst': DEF_RIst})   
            #creating a new telegram to send
            # telegram = create_xml(sent_mess_list)    
            telegram = create_xml_fast(sent_mess_list)
            # print(telegram)    
            SendData(telegram, conn, received_address)
            

            time.sleep(0.001)

        except Exception as e:
            print(f"[Server] Error in client thread: {e}")
            break



def routine_for_robot(server_address, port, mess_input, mess_output):
    conn = create_socket(server_address, port)
    
    # init variables supposed to send to robot
    watchDog_in = '0'
    frame_assign = '0'
    move_next_pt = '0'
    cam_cal_in_proc = '0'
    see_4_targets = '0'
    xyz1 = {'X': '0.0', 'Y': '0.0', 'Z': '0.0'}
    xyz2 = {'X': '0.0', 'Y': '0.0', 'Z': '0.0'}
    xyz3 = {'X': '0.0', 'Y': '0.0', 'Z': '0.0'}
    xyz4 = {'X': '0.0', 'Y': '0.0', 'Z': '0.0'}
    cam_cal_res = {'X': '0.0', 'Y': '0.0', 'Z': '0.0', 'A': '0.0', 'B': '0.0', 'C': '0.0'}
    
    while True:
        telegram_from_camera = None
        try:
            # get message from camera (dict)
            # if len(mess_input) > 0:
            # if mess_input.has_message(): 
            ################################################
            #RECEIVING MESSAGES
            ################################################           

            # GET MESSAGE FROM CAMERA SERVER-SUBPROCESS
            telegram_from_camera = mess_input.get()
            if telegram_from_camera is not None:
            # extract data from telegram
                try:
                    watchDog_in = telegram_from_camera["WatchDog_in"]
                    frame_assign = telegram_from_camera["Frame_assign"]
                    move_next_pt = telegram_from_camera["Move_next_pt"]
                    cam_cal_in_proc = telegram_from_camera["Cam_cal_in_proc"]
                    see_4_targets = telegram_from_camera["See_4_targets"]
                    xyz1 = telegram_from_camera["XYZ1"]
                    xyz2 = telegram_from_camera["XYZ2"]
                    xyz3 = telegram_from_camera["XYZ3"]
                    xyz4 = telegram_from_camera["XYZ4"]
                    cam_cal_res = telegram_from_camera["CAM_CAL_RES"]
                    
                    
                except Exception as ex:
                    print(f" Data can not be extracted from telegram from camera: {ex}")
            # else:
            #     print('robot has no messages from camera')
                

            # GET MESSAGE FROM ROBOT CLIENT
            bytesAddressPair = conn.recvfrom(4096)            
            ReceivedMessage = bytesAddressPair[0]
            received_address = bytesAddressPair[1]
            # clientMsg = "Message from { } : {}".format(received_address, ReceivedMessage)
            # clientIP = "Client IP Address:{}".format(received_address)
            # print('received from robot', ReceivedMessage)
            # print(clientIP)    
            received_dict = extract_xml(ReceivedMessage)


            

            ################################################
            #SENDING MESSAGES
            ################################################
            
            # SEND MESSAGE TO CAMERA SERVER-SUBPROCESS
            mess_output.put(received_dict)            
            
            # print(clientIP, received_dict)
            
            # SEND MESSAGE TO ROBOT CLIENT
            sent_mess_list = list() # This message to be sent from Server to KRC client
            sent_mess_list.append({"Sen" : {'Type' : 'ImFree'}})   
            sent_mess_list.append({'XYZ1': xyz1})
            sent_mess_list.append({'XYZ2': xyz2})
            sent_mess_list.append({'XYZ3': xyz3})
            sent_mess_list.append({'XYZ4': xyz4})
            sent_mess_list.append({'CAM_CAL_RES': cam_cal_res})
            sent_mess_list.append({"WatchDog_in" : watchDog_in}) 
            sent_mess_list.append({"Frame_assign" : frame_assign}) 
            sent_mess_list.append({"Move_next_pt" : move_next_pt}) 
            sent_mess_list.append({"Cam_cal_in_proc" : cam_cal_in_proc}) 
            sent_mess_list.append({"See_4_targets" : see_4_targets}) 
            # sent_mess_list.append({'RKorr': {'X': '278.60', 'Y': '164.79', 'Z': '615.22', 'A': '0.0', 'B': '0.0', 'C': '0.0'}})  
            sent_mess_list.append({'IPOC': received_dict['IPOC']})
            
            #creating a new telegram to send
            # telegram = create_xml(sent_mess_list)
            telegram = create_xml_fast(sent_mess_list)
            # print('to send to robot', telegram)    
            SendData(telegram, conn, received_address)

            time.sleep(0.001)

        except Exception as e:
            print(f"[Server] Error in client thread: {e}")
            break


if __name__ == '__main__':
    
    # Load IP, PORT configuration from config
    config = load_config()
    server_address = config['server']['address']
    robot_port = config['server']['robot_port']
    camera_port = config['server']['camera_port']

    

    # Bind server's address and port to expect message from client
    # conn_to_client_robot = create_socket(address = server_address, port = robot_port)
    # conn_to_client_camera = create_socket(address = server_address, port = camera_port)

    # Create shared messages from robot to camera and vice versa
    with Manager() as manager:
        mess_rob_to_cam = SingleMessageQueue(manager)
        mess_cam_to_rob = SingleMessageQueue(manager)


        p1 = Process(target=routine_for_robot, args=(server_address, robot_port, mess_cam_to_rob, mess_rob_to_cam))
        p2 = Process(target=routine_for_camera, args=(server_address, camera_port, mess_rob_to_cam, mess_cam_to_rob))

        p1.start()
        p2.start()
    
        print('UDP server is turning on (multiprocessing mode)')
        try:
            while True:
                time.sleep(0.001)
                display.clear_output(wait=True)
        except KeyboardInterrupt:
            print("Shutting down...")
    
        # Остановка процессов
        p1.terminate()
        p2.terminate()
        p1.join()
        p2.join()


                
        