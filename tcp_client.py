"""Example TCP socket client."""
import socket
import threading
import json
import queue

class ExploreDrone:
    "Construct an instance of an explorer Drone"

    def __init__(self, host, port, coords):
        """Construct a Manager instance and start listening for messages."""

        self.host = host
        self.port = port
        self.coords = coords
        self.startup = True

        self.run_drone()


    def tcp_server(self):

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:

            # Bind the socket to the server
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self.host, self.port))
            sock.listen()

            if self.startup:
                self.send_coords()
                self.startup = False

            # Socket accept() will block for a maximum of 1 second.  If you
            # omit this, it blocks indefinitely, waiting for a connection.
            sock.settimeout(1)

            while True:
                # Wait for a connection for 1s.  The socket library avoids consuming
                # CPU while waiting for a connection.
                try:
                    clientsocket, address = sock.accept()
                except socket.timeout:
                    continue
                print("Connection from", address[0])

                # Socket recv() will block for a maximum of 1 second.  If you omit
                # this, it blocks indefinitely, waiting for packets.
                clientsocket.settimeout(1)

                # Receive data, one chunk at a time.  If recv() times out before we
                # can read a chunk, then go back to the top of the loop and try
                # again.  When the client closes the connection, recv() returns
                # empty data, which breaks out of the loop.  We make a simplifying
                # assumption that the client will always cleanly close the
                # connection.
                with clientsocket:
                    message_chunks = []
                    while True:
                        try:
                            data = clientsocket.recv(4096)
                        except socket.timeout:
                            continue
                        if not data:
                            break
                        message_chunks.append(data)

                # Decode list-of-byte-strings to UTF8 and parse JSON data
                message_bytes = b''.join(message_chunks)
                message_str = message_bytes.decode("utf-8")
                print(message_str)

                try:
                    message_dict = json.loads(message_str)
                except json.JSONDecodeError:
                    continue
                print(message_dict)
                self.handle_message(message_dict)
    
    def handle_message(self, message_dict):
        if message_dict["message_type"] == "coords_ack":
            self.handle_coords_ack(message_dict)
        else:
            print("Message Unknown")
        
    # adds one pair of coords
    def handle_coords_ack(self, message_dict):
        self.send_coords()

    def send_coords(self):
        if self.coords.empty():
            return
        
        coord = self.coords.get()
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect(("rpi1", 8000))
            message = json.dumps({
                "message_type": "coordinates",
                "coords": coord
            })
            sock.sendall(message.encode('utf-8'))
        
    def run_drone(self):
        tcp_thread = threading.Thread(target=self.tcp_server)
        tcp_thread.start()

        tcp_thread.join()

def main():

    # coords_list = []
    coords_list = queue.Queue()
    coords_list.put((1, 2))
    coords_list.put((0, 0))
    coords_list.put((6, 7))
    ExploreDrone("rpi2", 8000, coords_list)

    # """Test TCP Socket Client."""
    # # create an INET, STREAMing socket, this is TCP

    # with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    #     # connect to the server
    #     sock.connect(("rpi1", 8000))

    #     for coord in coords_list:
            
    #         print("HELLO")
    #         # send a message
    #         message = json.dumps({"message_type": "coordinates",
    #                             "coords": coord,
    #                             }) + "\n"
                                
    #         sock.sendall(message.encode('utf-8'))


if __name__ == "__main__":
    main()

