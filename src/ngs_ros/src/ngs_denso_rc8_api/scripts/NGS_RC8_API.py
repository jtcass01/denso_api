import enum

from SocketClient import SocketClient

class NGS_RC8_API():
    '''
    Class Description:
    Author: Jacob Taylor Cassady - Undergraduate Research Assistant
        University of Louisville J.B. Speed School of Engineering
        Next Generation Systems Robotics Lab
    '''
    def __init__(self):
        '''
        Constructor
        '''
#        self.socket_client = SocketClient()
        self.socket_client = None

    def __del__(self):
        '''
        Destructor
        '''
        pass

    def reconnect(self):
        self.socket_client.end_connection()
        self.socket_client.prompt_for_connection()

    def call_function(self, input_string):
        print("\nFunction input string: ", input_string)

        encoded_input = self.encode_input(input_string)

        print("Encoded function input: ", encoded_input)

#        self.socket_client.send_message(encoded_input + '\r')


    def encode_input(self, input_string):
        input_string = input_string.rstrip().lstrip()
        func = input_string.split(' ')[0].upper()

        if func == self.RC8Functions.DRAW:
            return NGS_RC8_API.format_for_draw(input_string)
        elif func == self.RC8Functions.DRIVE:
            return NGS_RC8_API.format_for_drive(input_string)
        elif func == self.RC8Functions.DRIVEA:
            return NGS_RC8_API.format_for_driveA(input_string)
        else:
            print("function: ", func, " is not currently supported.")


    class RC8Functions(enum.EnumMeta):
        DRAW = "DRAW"
        DRIVE = "DRIVE"
        DRIVEA = "DRIVEA"


    @staticmethod
    def process_phrase(phrase):
        phrase = phrase.rstrip().lstrip()

        if(phrase[0].lower() == 's'):
            phrase = "S" + str(int("".join(list(filter(str.isdigit, phrase)))))
        else:
            phrase = "".join(phrase.split(' '))

        return phrase

    @staticmethod
    def retrieve_arguments(argument_string):
        phrase = ""
        message_string = ""
        open_bracket = False

        for character in argument_string:
            if character == '(':
                open_bracket = True
                phrase += character
            elif character == ')':
                open_bracket = False
                phrase += character
            elif character == ',' and not open_bracket:
                message_string += NGS_RC8_API.process_phrase(phrase) + ";"
                phrase = ""
            else:
                phrase += character

        message_string += NGS_RC8_API.process_phrase(phrase)

        return message_string

    @staticmethod
    def format_for_draw(input_string):
        # Cut string to get rid of function
        input_string = input_string[5:]

        #retrieve segments of draw call
        return "1;" + NGS_RC8_API.retrieve_arguments(input_string)

    @staticmethod
    def format_for_drive(input_string):
        # Cut string to get rid of function
        input_string = input_string[6:]

        #retrieve segments of draw call
        return "2;" + NGS_RC8_API.retrieve_arguments(input_string)

    @staticmethod
    def format_for_driveA(input_string):
        # Cut string to get rid of function
        input_string = input_string[7:]

        #retrieve segments of draw call
        return "3;" + NGS_RC8_API.retrieve_arguments(input_string)

if __name__ == "__main__":
    test_api = NGS_RC8_API()

    test_api.call_function("Draw L, V(50, 10, 50 ), Speed = 180")
    test_api.call_function("drive (1, J(10, 20, 30, 40, 50, 60, 70, 80))")
    test_api.call_function(" DRIVEA (1, J(10, 20, 30, 40, 50, 60, 70, 80))")
