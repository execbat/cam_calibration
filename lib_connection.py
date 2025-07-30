import socket

def SendData(Transmit,OpenedSocket, target_address):
    sent = OpenedSocket.sendto(Transmit, target_address)

def create_socket(address = '127.0.0.1', port = 59152):
    UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    UDPServerSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # allows re-bind
    UDPServerSocket.bind((address, port))
    return UDPServerSocket

