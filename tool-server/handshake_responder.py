from ipaddress import IPv4Address
from socketserver import UDPServer, BaseRequestHandler
from math import floor
from random import random

from WirelessClientMessage_pb2 import WirelessClientMessage
from WirelessHostMessage_pb2 import WirelessHostMessage


clients = []

class UDPHandler(BaseRequestHandler):
    def handle(self):
        print("I: Got UDP packet")
        in_data = self.request[0].strip()
        out_socket = self.request[1]
        in_packet = WirelessClientMessage.FromString(in_data)
        if in_packet.type == WirelessClientMessage.HANDSHAKE_REQUEST:
            print("I: HANDSHAKE {}".format(self.client_address))
            out_packet = WirelessHostMessage()
            out_packet.type = WirelessHostMessage.HANDSHAKE_ASSIGNMENT
            out_packet.assignment.id = floor(1023 * random() + 1)
            out_packet.assignment.address_ipv4 = int(IPv4Address(self.client_address[0]))
            out_packet.assignment.latency = 0
            out_socket.sendto(out_packet.SerializeToString(), self.client_address)
        elif in_packet.type == WirelessClientMessage.HANDSHAKE_ASSIGNMENT_CONFIRMATION:
            print("I: HANDSHAKE_CONFIRMATION {}".format(self.client_address))
            clients.append((in_packet.assignment.id, IPv4Address(self.client_address[0])))
            print("I: CLIENTS {}".format(clients))
        print("----------")

print("Starting handshake responder...")
while True:
    with UDPServer(("0.0.0.0", 10010), UDPHandler) as server:
        server.serve_forever()
