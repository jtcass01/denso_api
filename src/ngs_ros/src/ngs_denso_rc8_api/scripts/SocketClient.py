import socket

class SocketClient(object):
    '''
    '   Class Description: TCP Socket client
    '   Author: Jacob Taylor Cassady
    '''
    def __init__(self, sock=None, buffer_size=1024):
        '''
        '   Constructor
        '   Author: Jacob Taylor Cassady
        '''
        if sock is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock
        self.buffer_size = buffer_size
        self.prompt_for_connection()

    def __del__(self):
        '''
        '   Destructor
        '   Author: Jacob Taylor Cassady
        '''
        del self.buffer_size
        del self.sock
        del self

    def connect(self, ip_address, port):
        '''
        '   Method Description: Connects to a given ip_address and port
        '   Author: Jacob Taylor Cassady
        '''
        port_description = (ip_address, port)

        print("Attempting to bind to port with description:", port_description)

        # Bind to the port description
        self.sock.connect(port_description)

        print("Successfully created connection with server @ address {} through port {}".format(ip_address, port))


    def loop_listen(self, expected_message_count = 10):
        '''
        '   Method Description: Constantly checks the socket for a message and prints it
        '   Author: Jacob Taylor Cassady
        '''
        while(1):
            response = self.sock.recv(self.buffer_size)
            print(response)

    def send_message(self, message):
        '''
        '   Method Description: Encodes a message and sends it across the socket
        '   Author: Jacob Taylor Cassady
        '''
        self.sock.send(message.encode())

    def prompt_for_connection(self):
        '''
        '   Method Description: Prompts for a connection from the user and attempts to create a socket matching the given definition.
        '   Author: Jacob Taylor Cassady
        '''
        ip_address = raw_input("What is the ipaddress of the robot [default = 192.168.1.100] : ")
        if ip_address == "":
            ip_address = "192.168.1.100"

        port = raw_input("What port would you like to connect to [default = 49152]: ")
        if port == "":
            port = 49152
        else:
            port = int(port)

        self.connect(ip_address=ip_address, port=port)

    def prompt_for_message(self):
        '''
        '   Method Description: Requests a message form the user and sends it
        '   Author: Jacob Taylor Cassady
        '''
        message = input('What would you like to send to the server: ')
        print("Sending message: " + message + '\r')
        self.send_message(message = message + '\r')

    def prompt_for_loop_listen(self):
        '''
        '   Method Description: Asks the user how many messages they are expecting and prints out that many.
        '   Author: Jacob Taylor Cassady
        '''
        expected_message_count = input('How many messages are you expecting: ')
        self.loop_listen(expected_message_count = int(expected_message_count))

    def prompt_for_binary(self):
        '''
        '   Method Description: Prompts for a message and encodes it in binary.
        '   Author: Jacob Taylor Cassady
        '''
        binary_string = input('Enter the binary representation you\'d like to send: ')
        self.sock.send(bytes(int(binary_string, base=2)))

    def end_connection(self):
        '''
        '   Method Description: Closes the socket.
        '   Author: Jacob Taylor Cassady
        '''
        print("Ending the TCP Connection.")
        self.sock.close()

    def menu(self):
        '''
        '   Method Description: Prints a menu, requests a response, and calls the matching function.
        '   Author: Jacob Taylor Cassady
        '''
        print("==== MENU ====")
        print("1) send a message")
        print("2) loop listen")
        print("3) communication loop [In progress...]")
        print("4) Send binary")
        print("5) Reconnect")
        print("0) end connection")

        response = int(input("What would you like to do: "))

        if(response == 1):
            self.prompt_for_message()
        elif(response == 2):
            self.prompt_for_loop_listen()
        elif(response == 3):
            self.communication_loop()
        elif(response == 4):
            self.prompt_for_binary()
        elif(response == 5):
            self.end_connection()
            self.prompt_for_connection()
        elif(response == 0):
            self.end_connection()
        else:
            print("Invalid menu selection.  Please try again...")

        return response


if __name__ == "__main__":
    '''
    ' Test Script for the SocketClient.  Will run when you call 'python SocketClient.py' from the command line.
    ' Author: Jacob Taylor Cassady
    '''
    test_client = SocketClient()

    response = 1

    while(response != 0):
        response = test_client.menu()
