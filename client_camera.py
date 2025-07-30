

from IPython import display

import time
from lib_xml_tree import *
from lib_connection import *
from lib_config_loader import load_config


    
if __name__ == '__main__':

    # Load IP, PORT configuration from config
    config = load_config()
    target_address = config['clients']['camera']['target_address']  
    target_port = config['clients']['camera']['target_port']

    UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    server_address =  (target_address, target_port)

    
    telegram = create_xml_fast([{'IPOC': '0'}])
    print('start_mess = :', telegram)

    # init dependant telegram variables
    watchDog_out = '0'
    
    while True:
        #Send
        TransmitData = telegram
        SendData(TransmitData, UDPClientSocket, server_address)


        #Receive
        bytesAddressPair = UDPClientSocket.recvfrom(4096)

        ReceivedMessage = bytesAddressPair[0]
        address = bytesAddressPair[1]
        # clientMsg = "Message from Client:{}".format(ReceivedMessage)
        # clientIP = "Client IP Address:{}".format(address)
        #print(clientMsg)
        #print(clientIP)

        received_dict = extract_xml(ReceivedMessage)
        print('camera_received_telegram', ReceivedMessage)

                # use received data from cam
        try:
            watchDog_out = received_dict['WatchDog_out']
        except Exception as ex:
            print(f" Data can not be extracted from received to camera telegram: {ex}")   


        sent_mess_list = list() # List of dictionaries. every dictionary will be the element of ElementTree
        # sent_mess_list.append({"Sen" : {'Type' : 'ImFree'}})  
        # try:
        sent_mess_list.append({"Sen" : {'Type' : 'Camera'}})   
        sent_mess_list.append({'XYZ1': {'X': '0.0', 'Y': '0.0', 'Z': '0.0'}})
        sent_mess_list.append({'XYZ2': {'X': '0.0', 'Y': '0.0', 'Z': '0.0'}})
        sent_mess_list.append({'XYZ3': {'X': '0.0', 'Y': '0.0', 'Z': '0.0'}})
        sent_mess_list.append({'XYZ4': {'X': '0.0', 'Y': '0.0', 'Z': '0.0'}})
        sent_mess_list.append({'CAM_CAL_RES': {'X': '0.0', 'Y': '0.0', 'Z': '0.0', 'A': '0.0', 'B': '0.0', 'C': '0.0'}})
        sent_mess_list.append({"WatchDog_in" : watchDog_out}) 
        sent_mess_list.append({"Frame_assign" : '0'}) 
        sent_mess_list.append({"Move_next_pt" : '0'}) 
        sent_mess_list.append({"Cam_cal_in_proc" : '0'}) 
        sent_mess_list.append({"See_4_targets" : '0'}) 

        #cresting a new telegram to send
        telegram = create_xml_fast(sent_mess_list)
        print('camera_sending_telegram', telegram)
        
                
        display.clear_output(wait=True)


        


        
        
