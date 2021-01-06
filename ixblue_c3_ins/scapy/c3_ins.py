#!/usr/bin/python

# Send 'fake" C3 INS "NAVIGATION LONG" packets to the test board
# nav long messages are 90 bytes payload and must start with 0x24 0xAA

# To install scapy:
# sudo apt-get install scapy

# To run:
# sudo python ./c3_ins.py


from scapy.all import *

src_ip =  "192.168.214.2"
sport = 54001
dst_ip = "192.168.214.200"
dport = 8888

ip=IP()
ip.src = src_ip
ip.dst = dst_ip

udp = UDP()
udp.sport = sport
udp.dport = dport
payload = bytearray( [0x24, 0xAA,             # header
                      0x00, 0x00, 0x00, 0x00, # user status
                      0x00, 0x00, 0x00, 0x00, # algo status 1
                      0x00, 0x00, 0x00, 0x00, # algo status 2
                      0x40, 0x48, 0xF5, 0xC3, # heading 3.1400
                      0x00, 0x00, 0x00, 0x00, # roll
                      0x00, 0x00, 0x00, 0x00, # pitch
                      0x40, 0x02, 0x00, 0x00, # north speed 2.5
                      0x00, 0x00, 0x00, 0x00, # east speed
                      0x00, 0x00, 0x00, 0x00, # vertical
                      0x32, 0xA9, 0x6F, 0xBC, # latitude 42.36 N
                      0x33, 0x0E, 0x19, 0x58, # longitude 71.05 W
                      0xC2, 0xC8, 0x00, 0x00, # altitude -100
                      0x00, 0x00, 0x00, 0x00, # time
                      0x00, 0x00, 0x00, 0x00, # heading error
                      0x00, 0x00, 0x00, 0x00, # roll err
                      0x00, 0x00, 0x00, 0x00, # pitch err
                      0x00, 0x00, 0x00, 0x00, # north speed err
                      0x00, 0x00, 0x00, 0x00, # east speed err
                      0x00, 0x00, 0x00, 0x00, # vertical speed err
                      0x00, 0x00, 0x00, 0x00, # latitude err
                      0x00, 0x00, 0x00, 0x00, # longitude err
                      0x00, 0x00, 0x00, 0x00,] ) # altitude err

while (1) :
    print("sending UDP packet")
    send(ip/udp/Raw(load=payload)) 
    time.sleep(15)

