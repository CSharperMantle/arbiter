import socket
from ipaddress import IPv4Address
from math import floor
from random import random

from WirelessClientMessage_pb2 import WirelessClientMessage
from WirelessHostMessage_pb2 import WirelessHostMessage

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("0.0.0.0", 10010))

print("Starting handshake responder...")
while True:
    data, addr = s.recvfrom(1024)
    packet = WirelessClientMessage.FromString(data)
    print("I: ADDR: ", addr)
    print("I: RAW: ", data)
    print("I: FORMATTED:")
    print(packet, end="")
    if packet.type == WirelessClientMessage.HANDSHAKE_REQUEST:
        response = WirelessHostMessage()
        response.type = WirelessHostMessage.HANDSHAKE_ASSIGNMENT
        response.assignment.id = floor(1023 * random() + 1)
        response.assignment.address_ipv4 = int(IPv4Address(addr[0]))
        response.assignment.latency = 0
        bin_str = response.SerializeToString()
        s.sendto(bin_str, (addr[0], 10012))
        print("I: RESPONSE: ", bin_str)
        print("I: FORMATTED:")
        print(response)
    print("----------")
