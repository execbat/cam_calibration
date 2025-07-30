

from IPython import display

import time
from lib_xml_tree import *
from lib_connection import *
from lib_config_loader import load_config


    
if __name__ == '__main__':
    
    # Load IP, PORT configuration from config
    config = load_config()
    target_address = config['clients']['robot']['target_address']  
    target_port = config['clients']['robot']['target_port']

    
  
    UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    server_address =  (target_address, target_port) 

    telegram = create_xml_fast([{'IPOC': '0'}])
    print('start_mess = :', telegram)


    # init dependant telegram variables
    watchDog_in = '0'
    
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
        print('robot_received_telegram',ReceivedMessage)

        # use received data from cam
        try:
            watchDog_in = received_dict['WatchDog_in']
        except Exception as ex:
            print(f" Data can not be extracted from received to robot telegram: {ex}")        




        sent_mess_list = list() # List of dictionaries. every dictionary will be the element of ElementTree
        sent_mess_list.append({"Sen" : {'Type' : 'KRC'}})  
        sent_mess_list.append({'WatchDog_out': str(1 - int(watchDog_in))})                        
        sent_mess_list.append({'Frame_assigned': '0'})
        sent_mess_list.append({'Position_reached': '0'})
        sent_mess_list.append({'Need_cam_cal': '0'})        
        sent_mess_list.append({'Screenshot': '0'}) 
        sent_mess_list.append({'IPOC': str(int(received_dict['IPOC']) + 1)})

        #cresting a new telegram to send
        # telegram = create_xml(sent_mess_list)
        telegram = create_xml_fast(sent_mess_list)
        print('robot_sending_telegram',  telegram)
        
                
        display.clear_output(wait=True)

        


        
        
