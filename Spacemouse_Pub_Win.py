#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun  2 02:38:10 2024
@author: kathanpatel Pub.py
"""
import pywinusb.hid as hid
import logging
import signal
import threading
import socket
import time

# Configure logging
logging.basicConfig(level=logging.DEBUG)

class dev_3d(object):
    def __init__(self, device):
        self.device = device
        self.data = [0, 0, 0, 0, 0, 0, 0, 0]
        self.id = -1

def dealdata(data_new, data_rec):
    if data_new:
        logging.debug(f"Received data: {data_new}")

    if data_new[0] == 1:
        # translation packet
        data_rec[0] = data_new[1] + (data_new[2]*256)
        data_rec[1] = data_new[3] + (data_new[4]*256)
        data_rec[2] = data_new[5] + (data_new[6]*256)

        if data_new[2] > 127:
            data_rec[0] -= 65536
        if data_new[4] > 127:
            data_rec[1] -= 65536
        if data_new[6] > 127:
            data_rec[2] -= 65536
        
    if data_new[0] == 2:
        # rotation packet
        data_rec[3] = data_new[1] + (data_new[2]*256)
        data_rec[4] = data_new[3] + (data_new[4]*256)
        data_rec[5] = data_new[5] + (data_new[6]*256)

        if data_new[2] > 127:
            data_rec[3] -= 65536
        if data_new[4] > 127:
            data_rec[4] -= 65536
        if data_new[6] > 127:
            data_rec[5] -= 65536
        
    if data_new[0] == 3:
        data_rec[6] = data_new[1] & 0x01
        data_rec[7] = (data_new[1] & 0x02) >> 1



    kkp = [data_rec[0], data_rec[1], data_rec[2], data_rec[3], data_rec[4], data_rec[5], data_rec[6], data_rec[7]]
    # Return the processed data
    return kkp

def sigint_handler(signal, frame):
    global run
    run = False
    logging.info('Key interrupt')

def read_task(dev, client_socket):
    global run
    event = threading.Event()

    def data_handler(data):
        try:
            dev.data = dealdata(data, dev.data)
            # Send the processed data to the client
            print("this is data which is send = ",dev.data)
            client_socket.sendall(str(dev.data).encode('utf-8'))
        except Exception as e:
            logging.error(f"Data processing failed: {e}")
        event.set()

    dev.device.set_raw_data_handler(data_handler)

    while run:
        event.wait()
        event.clear()

    dev.device.close()

if __name__ == '__main__':
    global run
    global lx
    global ly
    global lz
    global rx
    global ry
    global rz

    run = True

    signal.signal(signal.SIGINT, sigint_handler)

    # Look for SpaceNavigator
    all_hids = hid.HidDeviceFilter(vendor_id=0x256f, product_id=0xc635).get_devices()

    if not all_hids:
        logging.error('SpaceNavigator not found')
        exit(1)
    else:
        logging.info('SpaceNavigator found')

    dev_list = []
    threads = []

    # Create a socket object
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.settimeout(0.5)

    # Bind the socket to a public host and a well-known port
    host = '127.0.0.1'
    port = 12345
    server_socket.bind((host, port))

    # Listen for incoming connections (queue up to 5 requests)
    server_socket.listen(5)
    print(f"Server listening on {host}:{port}")

    client_socket = None
    while run and client_socket is None:
        try:
            client_socket, addr = server_socket.accept()
            client_socket.setblocking(False)
            print(f"Got a connection from {addr}")
        except socket.timeout:
            continue
        except Exception as e:
            logging.error(f"Error accepting connection: {e}")

    if client_socket is None:
        logging.error('No client connection established. Exiting.')
        exit(1)

    for device in all_hids:
        device.open()
        dev = dev_3d(device)
        dev.id = len(dev_list)
        dev_list.append(dev)

        t = threading.Thread(target=read_task, args=(dev, client_socket))
        threads.append(t)

    logging.info(f"Important! Exit by pressing Ctrl-C, total {len(dev_list)} device(s)")

    for t in threads:
        t.setDaemon(True)
        t.start()

    try:
        while run:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    for t in threads:
        t.join()

    # Close the client socket
    if client_socket:
        client_socket.close()
        print("Client socket closed.")

    # Close the server socket
    server_socket.close()
    print("Server socket closed.")
