import os
from NGS_RC8_API import Command

class FileManager():
    @staticmethod
    def load_command_set(command_set_alias):
        '''
        ' Method Description: Retrieves a set of commands from a command_set_alias stored as a csv file in the config folder.
        ' Author: Jacob Taylor Cassady
        '''
        command_set = list([])

        with open("config" + os.path.sep + command_set_alias + ".csv") as command_set_file:
            # read header
            command_line = command_set_file.readline()
            # read first line
            command_line = command_set_file.readline()

            while command_line:
                command_data = command_line.split(",")
                position_data = [command_data[3], command_data[4], command_data[5], command_data[6], command_data[7], command_data[8], command_data[9]]
                command_set.append(Command(command = command_data[1], motion_interpolation = command_data[2], position_type = "P", position_data = position_data))
                command_line = command_set_file.readline()

        return command_set

if __name__ == "__main__":
    FileManager.load_command_set("small_electric_gripper")
