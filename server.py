#!/usr/bin/env python3

import _thread
import socket
import time

HOST = ''  # Standard loopback interface address (localhost)
PORT = 27015        # Port to listen on (non-privileged ports are > 1023)


def sender(conn):
    i = 0
    while True:
        time.sleep(1)
        i+=1
        conn.sendall(b"halo\n")

def default(conn):
    while True:
        data = conn.recv(1024)
        print(data)
        if not data:
            break
        conn.sendall(data)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()

    with conn:
        print('Connected by', addr)
        _thread.start_new_thread( default, (conn, ) )
        
