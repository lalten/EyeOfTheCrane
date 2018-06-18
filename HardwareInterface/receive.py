#!/usr/bin/env python

import socket

from serial import Serial

TCP_IP = '192.168.43.3'
TCP_PORT = 5005
BUFFER_SIZE = 20  # Normally 1024, but we want fast response

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)

ser = Serial('/dev/serial/by-id/usb-SparkFun_SparkFun_Pro_Micro-if00', 115200, timeout=10)

conn, addr = s.accept()
print 'Connection address:', addr
while 1:
    data = conn.recv(BUFFER_SIZE)
    if not data: break
    print data
    ser.write(data)
    conn.send(data)  # echo
conn.close()
