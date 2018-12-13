import socket

class SocketClient(object):
    """TCP Socket client for communication"""
    def __init__(self, sock=None, buffer_size=1024):
        if sock is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock
        self.buffer_size = buffer_size

    def connect(self, ip_address, port):
        port_description = (ip_address, port)

        print("Attempting to bind to port with description:", port_description)

        # Bind to the port description
        self.sock.connect(port_description)

        print("Successfully created connection with server @ address {} through port {}".format(ip_address, port))


    def loop_listen(self, expected_message_count = 10):
        while(1):
            self.sock.recv(self.buffer_size)
            print()

    def send_message(self, message):
        self.sock.send(message.encode())

    def communication_loop(self):
        message = ""

        while(message != "QUIT"):
            message = input('What would you like to send to the server: ')
            self.send_message(message = message)

            for response in range(3):
                print(self.sock.recv(self.buffer_size))

    def prompt_for_connection(self):
        ip_address = "192.168.1.100"
        port = int(input("What port would you like to connect to: "))

        self.connect(ip_address=ip_address, port=port)

    def prompt_for_message(self):
        message = input('What would you like to send to the server: ')
        print("Sending message: " + message + '\r')
        self.send_message(message = message + '\r')

    def prompt_for_loop_listen(self):
        expected_message_count = input('How many messages are you expecting: ')
        self.loop_listen(expected_message_count = int(expected_message_count))

    def prompt_for_binary(self):
        binary_string = input('Enter the binary representation you\'d like to send: ')
        self.sock.send(bytes(int(binary_string, base=2)))

    def end_connection(self):
        print("Ending the TCP Connection.")
        self.sock.close()

    def menu(self):
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
    test_client = SocketClient()
    test_client.prompt_for_connection()

    response = 1

    while(response != 0):
        response = test_client.menu()
