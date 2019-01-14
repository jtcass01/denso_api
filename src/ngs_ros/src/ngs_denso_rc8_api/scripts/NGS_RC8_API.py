import enum

from SocketClient import SocketClient

class NGS_RC8_API():
    '''
    ' Class Description: API for communication with Denso RC8
    ' Author: Jacob Taylor Cassady - Undergraduate Research Assistant
    '    University of Louisville J.B. Speed School of Engineering
    '    Next Generation Systems Robotics Lab
    '''
    def __init__(self, socket_connection = False):
        '''
        Constructor
        '''
        self.socket_connection = socket_connection

        self.implemented_functions = {
            "MOVE" : self.RC8Functions.MOVE,
            "DRAW" : self.RC8Functions.DRAW
            #            "DRIVE" : self.RC8Functions.DRIVE,
            #            "DRIVEA" : self.RC8Functions.DRIVEA
        }

        # Ensure there are no descrepancies between the implemented functions and the functions held within the enum
        assert len(self.implemented_functions) == len(NGS_RC8_API.RC8Functions), "There is a descrepancy in the number of implemented RC8Functions and the dictionary used to map these functions.  Dictionary's current state:\n\t" + str(self.implemented_functions)

        # Create a SocketClient connection if the API requests it.  If not, enters echo mode.
        if socket_connection == True:
            self.socket_client = SocketClient()

    def __del__(self):
        '''
        ' Destructor
        '''
        if self.socket_connection == True:
            del self.socket_client

        del self.socket_connection
        del self
        pass

    def reconnect(self):
        '''
        ' Class Description: Ends connection with socket and prompts for a new one.
        ' Author: Jacob Taylor Cassady
        '''
        if self.socket_connection == True:
            self.socket_client.end_connection()
            self.socket_client.prompt_for_connection()
        else:
            print("Reconnect called but NGS_RC8_API was initialized without a socket connection.  Reinitialize the object with argument socket_connect = True")

    def call_function(self, input_string):
        '''
        ' Class Description: 'Calls' a function by sending a message through the socket client to the RC8 PacScript Server.
        ' Author: Jacob Taylor Cassady
        '''
        print("Function input string: " + input_string)

        encoded_input = NGS_RC8_API.encode_input(input_string)

        print("Encoded function input: " + encoded_input + "\n")

        if self.socket_connection == True:
            self.socket_client.send_message(encoded_input + '\r')


    class RC8Functions(enum.Enum):
        '''
        ' Class Description: Enum class for keeping a list of implemented functions on RC8 PacScript Server.
        ' Author: Jacob Taylor Cassady
        '''
        MOVE = 1
        DRAW = 2
#        DRIVE = 3
#        DRIVEA = 4

    class InterpolationMethods(enum.Enum):
        '''
        ' Class Description: Enum class for keeping a list of valid interpolation methods used by move implemented_functions
        '       on the RC8 PacScript Server.
        ' Author: Jacob Taylor Cassady
        '''
        PTP = 1     # Point to Point
        P = PTP     # Point to Point
        '''
        ' PTP interpolation motion. This motion moves the fastest. TCP point track is not considered.
        '''
        L = 2       # Linear
        '''
        ' CP motion. TCP point moves from the current position to the target position linearly at constant speed
        ' in other than acceleration/deceleration areas.
        '''
        C = 3       # Circular
        '''
        ' Circular interpolation motion. The robot moves along the circular arc consisting of the current position,
        ' target position and relay position in order of the current position -> relay position -> target position.
        ' The robot moves at constant speed in other than acceleration/deceleration areas.
        ' Refer to https://densorobotics.com/content/user_manuals/19/000441.html
        '''
        S = 4       # Free Curve Motion
        '''
        ' Free curve motion. The robot moves at constant speed along a smooth curve crossing the registered point.
        ' Refer to https://densorobotics.com/content/user_manuals/19/000442.html
        '''

    class MotionOptions(enum.Enum):
        '''
        ' Class Description: Enum class for keeping a list of valid motion options used when moving the robotic arm.
        ' Refer to https://densorobotics.com/content/user_manuals/19/000521.html for more information.
        ' Author: Jacob Taylor Cassady
        '''
        S = 1       # Internal speed specification
        SPEED = S
        ACCEL = 2   # Internal acceleration specification
        DECEL = 3   # Internal deceleration specification
        TIME = 4    # Travel time specification


    @staticmethod
    def encode_input(input_string):
        '''
        ' Class Description: Formats an input input string into a message to be sent to the RC8 PacScript Server.
        ' Author: Jacob Taylor Cassady
        '''
        input_string = input_string.rstrip().lstrip()
        func = input_string.split(' ')[0].upper()

        if NGS_RC8_API.RC8Functions[func] == NGS_RC8_API.RC8Functions.MOVE:
            return NGS_RC8_API.format_for_move(input_string)
        elif NGS_RC8_API.RC8Functions[func] == NGS_RC8_API.RC8Functions.DRAW:
            return NGS_RC8_API.format_for_draw(input_string)
        elif NGS_RC8_API.RC8Functions[func] == NGS_RC8_API.RC8Functions.DRIVE:
            return NGS_RC8_API.format_for_drive(input_string)
        elif NGS_RC8_API.RC8Functions[func] == NGS_RC8_API.RC8Functions.DRIVEA:
            return NGS_RC8_API.format_for_driveA(input_string)
        else:
            print("function: ", func, " is not currently supported.")


    @staticmethod
    def process_argument(phrase):
        '''
        ' Method Description: Formats an argument of the message to be sent to the RC8 PacScript Server
        '   by striping the phrase of all spaces and reformatting unique phrases such as 'S for Speed'.
        ' Author: Jacob Taylor Cassady
        '''
        phrase = phrase.rstrip().lstrip()

        if(phrase[0].upper() == 'S'):
            phrase = "S" + str(int("".join(list(filter(str.isdigit, phrase)))))
        else:
            phrase = "".join(phrase.split(' '))

        return phrase

    @staticmethod
    def retrieve_arguments(argument_string):
        '''
        ' Method Description: Iterates over each argument and starts to build a message to be sent to the
        '   RC8 PacScript Server.
        ' Author: Jacob Taylor Cassady
        '''
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
                message_string += NGS_RC8_API.process_argument(phrase) + "|"
                phrase = ""
            else:
                phrase += character

        message_string += NGS_RC8_API.process_argument(phrase)

        return message_string

    @staticmethod
    def verify_interpolation_method(interpolation_key):
        denso_interpolation_methods = {
            "P" : NGS_RC8_API.InterpolationMethods.PTP,
            "PTP" : NGS_RC8_API.InterpolationMethods.PTP,
            "L" : NGS_RC8_API.InterpolationMethods.L,
            "C" : NGS_RC8_API.InterpolationMethods.C,
            "S" : NGS_RC8_API.InterpolationMethods.S,
        }

        assert len(denso_interpolation_methods) - 1 == len(NGS_RC8_API.InterpolationMethods), "There is a descrepancy in the number of interpolation methods and the dictionary used to map these methods.  Dictionary's current state:\n\t" + str(denso_interpolation_methods)

        result = denso_interpolation_methods.get(interpolation_key, "Not found")

        assert result != "Not found", "Motion interpolation key: " + interpolation_key + " is not included in list of denso interpolation methods:\n\t" + str(denso_interpolation_methods)

        return result.name

    @staticmethod
    def verify_motion_option(motion_option_key):
        motion_options = {
            "S" : NGS_RC8_API.MotionOptions.S,
            "SPEED" : NGS_RC8_API.MotionOptions.S,
            "ACCEL" : NGS_RC8_API.MotionOptions.ACCEL,
            "DECEL" : NGS_RC8_API.MotionOptions.DECEL,
            "TIME" : NGS_RC8_API.MotionOptions.TIME,
        }

        assert len(motion_options) - 1 == len(NGS_RC8_API.MotionOptions), "There is a descrepancy in the number of motion options and the dictionary used to map these options.  Dictionary's current state:\n\t" + str(motion_options)

        result = motion_options.get(motion_option_key, "Not found")

        assert result != "Not found", "The motion_option_key given: " + motion_option_key + " does not match any of the motion options supported by denso.  Please see the below dictionary for a list of the motion options and refer to https://densorobotics.com/content/user_manuals/19/000521.html for more information:\n\t" + str(motion_options)

        return result.name

    @staticmethod
    def list_implemented_pacscript_functions():
        print("\n- List of implemented pacscript functions and their matching enum -")
        index = 1
        for implemented_function, enum_obj in NGS_RC8_API.RC8Functions.__members__.items():
            print("Function enum: " + str(index) + ", Function Name: " + implemented_function.capitalize())
            index += 1
        print("")

    @staticmethod
    def format_for_move(input_string):
        '''
        ' Method Description: Formats an input string matching the move function.
        ' Author: Jacob Taylor Cassady
        '''
        # Cut string to get rid of function
        input_string = input_string[5:].lstrip().rstrip()

        # Determine interpolation
        interpolation = input_string[0]
        interpolation = NGS_RC8_API.verify_interpolation_method(interpolation)

        # Determine target positions
        target_positions = input_string[2:]
        target_positions = NGS_RC8_API.retrieve_arguments(target_positions)

        #retrieve segments of draw call
        return str(NGS_RC8_API.RC8Functions.MOVE.value) + ";" + str(interpolation)  + ";" +  str(target_positions)

    @staticmethod
    def format_for_draw(input_string):
        '''
        ' Method Description: Formats an input string matching the draw function.
        ' Author: Jacob Taylor Cassady
        '''
        # Cut string to get rid of function
        input_string = input_string[5:].lstrip().rstrip()

        # Determine interpolation
        interpolation = input_string[0]
        interpolation = NGS_RC8_API.verify_interpolation_method(interpolation)

        # Determine target positions
        target_positions = input_string[2:]
        target_positions = NGS_RC8_API.retrieve_arguments(target_positions)

        #retrieve segments of draw call
        return str(NGS_RC8_API.RC8Functions.DRAW.value) + ";" + str(interpolation)  + ";" +  str(target_positions)

    @staticmethod
    def format_for_drive(input_string):
        '''
        ' Method Description: Formats an input string matching the drive function.
        ' Author: Jacob Taylor Cassady
        '''
        # Cut string to get rid of function
        input_string = input_string[6:]

        #retrieve segments of draw call
        return str(NGS_RC8_API.RC8Functions.DRIVE.value) + ";" + NGS_RC8_API.retrieve_arguments(input_string)

    @staticmethod
    def format_for_driveA(input_string):
        '''
        ' Method Description: Formats an input string matching the driveA (Absolute) function.
        ' Author: Jacob Taylor Cassady
        '''
        # Cut string to get rid of function
        input_string = input_string[7:]

        #retrieve segments of draw call
        return str(NGS_RC8_API.RC8Functions.DRIVEA.value) + ";" + NGS_RC8_API.retrieve_arguments(input_string)

if __name__ == "__main__":
    '''
    ' Test Script for api.  Will run when you call 'python NGS_RC8_API.py' from the command line.
    ' Author: Jacob Taylor Cassady
    '''
    test_api = NGS_RC8_API()

    test_api.call_function("Move P, P( 740, 0, 480, 180, 0, 180, -1 )")
#    test_api.call_function("drive (1, J(10, 20, 30, 40, 50, 60, 70, 80))")
#    test_api.call_function(" DRIVEA (1, J(10, 20, 30, 40, 50, 60, 70, 80))")
