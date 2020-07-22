from Utils.utils import *
import threading
import socket



"""This class handles network communications."""

class Message_Handler:
    """Message_Handler handles processing of message from RPi
       It also acts as message sender
       i.e. it is self._sender in controller.py
    """
    def __init__(self, android_receive_handler):
        """Initialize the sender."""
        # Socket is used to listen for messages from RPi
        # address family: AF_INET
        self._rpi_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._rpi_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # connect to remote rpi socket over wifi; WIFI_HOST & RPI_PORT makes up the address of type tuple
        self._rpi_sock.connect((WIFI_HOST, RPI_PORT))


        self._android_recv_queue = []
        self._arduino_recv_queue = []
        self._rpi_recv_queue = []
        self._android_receive_handler = android_receive_handler
#        to invoke _receiver_rpi func, pass in _rpi_sock as parameter
        rpi_recv_thread = threading.Thread(target=self._receiver_rpi, args=(self._rpi_sock,), daemon=True)
#        rpi_recv_thread.daemon = True
        rpi_recv_thread.start()

    def _receiver_rpi(self, sock):
        """
        Listen for messages from the RPi and save the messages into the corresponding queue based on the source of the message.
        
        :Take in _rpi_sock
        :Return null
        """
        while True:
            data = sock.recv(1024)
            data = data.decode().strip()
            print('RECEIVED PRi: {}'.format(data))
            if not data:
                enable_print()
                print('RPi data not received')
                disable_print()
                break
                
#                data processing: need to check with RPI the format
            if data.startswith('RP'):
#             arrow
                data = data.split('RP')
#               data = ['', 'the rest']
                data[:] = [x for x in data if x != '']
#               data = ['the rest']
                print('Data from RPi: {}'.format(data))
#                _rpi_recv_queue is a list initialised to []; extend more suitable than extend in terms of formatting
                self._rpi_recv_queue.extend(data)
                print('rpi_recv_queue: {}'.format(self._rpi_recv_queue))

            elif data.startswith('AN'):
#            sensor reading
                data = data.split('AN')
                data[:] = [x for x in data if x != '']
                print('Data from Android: {}'.format(data))
                self._android_recv_queue.extend(data)
                print('android_recv_queue: {}'.format(self._android_recv_queue))

            elif data.startswith('AR'):
                data = data.split('AR')
                # if data[:8]=
                data[:] = [x for x in data if x != '']
                print('Data from Arduino: {}'.format(data))
                self._arduino_recv_queue.extend(data)
                print('arduino_recv_queue: {}'.format(self._arduino_recv_queue))

            else:
                print('Unable to identify Data source: {}'.format(data))

            if self._android_recv_queue:
                next_command = self._android_recv_queue[0]
                if next_command == ANDROID_EXPLORE:
                    print("received start exploration command")
                    continue
                next_command = self._android_recv_queue.pop(0)
                print('Pop Andoird Command: {}'.format(next_command))
                print('arduino_recv_queue: {}'.format(self._arduino_recv_queue))
#              _android_receive_handler is defined in controller.py as _receive_handler
#                if
                self._android_receive_handler(next_command)
#   used in controller.py
    def send_android(self, msg):
        """Send a message to the Android."""
        to_send = 'AN%s\n' % msg
        _send(self._rpi_sock, to_send)

    def send_arduino(self, msg):
        """Send a message to the Arduino."""
        to_send = 'AR%s\n' % msg
        enable_print()
        print('sending to arduino')
        print(to_send)
        _send(self._rpi_sock, to_send)

    def send_rpi(self, msg):
        """Send a message to the RPi."""
        to_send = 'RP%s\n' % msg
        _send(self._rpi_sock, to_send)
        
    def wait_android(self):
        while True:
            if self._android_recv_queue:
                next_command = self._android_recv_queue.pop(0)
                if next_command == ANDROID_EXPLORE:
                    return next_command
                else:
                    break

    def wait_arduino(self, msg_or_pattern, is_regex=False):
#    is_regex is for sensor reading
        """
        Wait for a message from the Arduino.

        :param msg_or_pattern: message to wait for, or pattern for message to match.
        :param is_regex: true if waiting for pattern, false if waiting for message.
        :return: returns matched string if waiting for pattern, nothing otherwise.
        """
        print("WAITING {} from Arduino".format(msg_or_pattern))
        while True:
            if self._arduino_recv_queue:
                next_command = self._arduino_recv_queue.pop(0)
#               if it is not sensor reading:
                if not is_regex:
#                 ARM
                    if next_command == 'M':
                        print("RECEIVED ARDUINO", next_command)
                        return 'M'
                    else:
                        print("here we are trying to remove DDDD")
                        match = msg_or_pattern.fullmatch(next_command)
                        if match:
                            print("RECEIVED ARDUINO", next_command)
                            return next_command
                        
                else:
                    match = msg_or_pattern.fullmatch(next_command)
                    if match:
                        print("RECEIVED ARDUINO", next_command)
                        return next_command

    def wait_rpi(self):
        """
        Wait for a message from the RPi.

        :param msg: The message to wait for.
        :return: N/A
        """
        # print("WAITING {} from RPi".format(msg_or_pattern))
        while True:
            if self._rpi_recv_queue:
                next_command = self._rpi_recv_queue.pop(0)
                
#                if next_command == '0':
#                    print("No image recognised")
#                    return False
#                else:
#                   nex_command is in the format of 0,1; where 0 is cell index, 1 is the image ID
                return next_command
                

#                if not is_regex:
#                    if next_command == msg:
#                        print("WAITED RPI", next_command)
#                        break
#                else:
#                    match = msg_or_pattern.fullmatch(next_command)
#                    if match:
#                        print("RECEIVED RPI", next_command)
#                        return next_command

def _send(sock, msg):
    "Send a message on a socket."""
    print("SENDING", msg)
    enable_print()
    print('sending xxxxx')
    sock.sendall(msg.encode())

#data = str.encode('k', "UTF-8")
#print(data)
#msg = 'AN'
#print(msg.encode())
