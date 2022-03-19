from math import floor
from random import random
import socket
from WirelessClientMessage_pb2 import WirelessClientMessage
from WirelessHostMessage_pb2 import WirelessHostMessage


s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("0.0.0.0", 10010))


while True:
    data, addr = s.recvfrom(1024)
    packet = WirelessClientMessage.FromString(data)
    print("I: ", addr, " ", data)
    print(packet, end="")
    print("I: RESPONDING WITH")
    response = WirelessHostMessage()
    response.type = WirelessHostMessage.HANDSHAKE_ASSIGNMENT
    response.assignment.id = floor(1023 * random() + 1)
    response.assignment.address_ipv4 = 0xFFFFFFFF
    response.assignment.latency = 0
    bin_str = response.SerializeToString()
    # bin_str = b"\x12\x34\x56\x78\x90\xAB\xCD\xEF"
    s.sendto(bin_str, (addr[0], 10012))
    print(bin_str)
    print("----------")
