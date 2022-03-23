from ipaddress import IPv4Address
from socketserver import UDPServer, BaseRequestHandler, StreamRequestHandler, ThreadingMixIn, TCPServer
from math import floor
from random import random
from threading import Thread
from time import sleep, time

from WirelessClientMessage_pb2 import WirelessClientMessage
from WirelessHostMessage_pb2 import WirelessHostMessage


clients = []


class ThreadedUDPServer(ThreadingMixIn, UDPServer):
    pass


class ThreadedTCPServer(ThreadingMixIn, TCPServer):
    pass


class UDPHandler(BaseRequestHandler):
    def handle(self):
        in_data = self.request[0].strip()
        in_packet = WirelessClientMessage.FromString(in_data)
        print("U: Got UDP packet from {}, type {}".format(
            self.client_address,
            in_packet.type
        ))
        out_socket = self.request[1]
        if in_packet.type == WirelessClientMessage.HANDSHAKE_REQUEST:
            out_packet = WirelessHostMessage()
            out_packet.type = WirelessHostMessage.HANDSHAKE_ASSIGNMENT
            out_packet.assignment.id = floor(1023 * random() + 1)
            out_packet.assignment.address_ipv4 = int(
                IPv4Address(self.client_address[0]))
            out_packet.assignment.latency = 0
            out_socket.sendto(
                out_packet.SerializeToString(),
                self.client_address
            )
        elif in_packet.type == WirelessClientMessage.HANDSHAKE_ASSIGNMENT_CONFIRMATION:
            clients.append(
                (in_packet.assignment.id, IPv4Address(self.client_address[0]))
            )


class TCPHandler(BaseRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._last_ping = -1

    def handle(self):
        print("T: TCP connection established with {}".format(self.client_address))
        while True:
            ping_packet = WirelessHostMessage()
            ping_packet.type = WirelessHostMessage.PING
            out_data = ping_packet.SerializeToString()
            sleep(1)
            print("T: Pinging {} with {}".format(
                self.client_address, out_data
            ))
            self.request.sendall(out_data)
            self._last_ping = time()
            print("T: Waiting for Pong from {}".format(self.client_address))
            in_data = self.request.recv(1024).strip()
            in_packet = WirelessClientMessage.FromString(in_data)
            print("T: Got TCP packet from {}, type {}".format(
                self.client_address,
                in_packet.type
            ))
            if in_packet.type == WirelessClientMessage.PONG:
                print("T: Ping latency: {}".format(time() - self._last_ping))
                self._last_ping = -1
            sleep(5)


if __name__ == "__main__":
    print("Starting handshake responder...")
    udp_server = ThreadedUDPServer(("0.0.0.0", 10010), UDPHandler)
    tcp_server = ThreadedTCPServer(("0.0.0.0", 10011), TCPHandler)
    udp_thread = Thread(target=udp_server.serve_forever, daemon=True)
    tcp_thread = Thread(target=tcp_server.serve_forever, daemon=True)
    udp_thread.start()
    tcp_thread.start()
    print("I: UDP server running on thread {}.".format(udp_thread.name))
    print("I: TCP server running on thread {}.".format(tcp_thread.name))

    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("I: Stopping UDP server...")
        udp_server.shutdown()
        udp_thread.join()
        print("I: Stopping TCP server...")
        tcp_server.shutdown()
        tcp_thread.join()
        print("I: Stopped.")
