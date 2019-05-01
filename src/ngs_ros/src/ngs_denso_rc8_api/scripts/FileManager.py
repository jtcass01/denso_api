import os
from NGS_RC8_API import Command

class FileManager():
    @staticmethod
    def load_tool_paths(tool_directory = os.getcwd() + os.path.sep + "config"+ os.path.sep + "tool_paths"+ os.path.sep):
        tools = list([])

        for root, dirs, files in os.walk(tool_directory):
            for file in files:
                if file.endswith(".csv"):
                    tools.append(file[:-4])

        return tools

    @staticmethod
    def list_tool_paths(tool_directory = os.getcwd() + os.path.sep + "config"+ os.path.sep + "tool_paths"+ os.path.sep):
        print("Available tool paths: ")

        for root, dirs, files in os.walk(tool_directory):
            for file in files:
                if file.endswith(".csv"):
                    print("\t" + file[:-4])


    @staticmethod
    def is_valid_tool_path(tool_name, tool_directory = os.getcwd() + os.path.sep + "config"+ os.path.sep + "tool_paths"+ os.path.sep):
        for root, dirs, files in os.walk(tool_directory):
            for file in files:
                tool_in_file = file[:-4]
                if tool_name == tool_in_file:
                    return True
        return False

    @staticmethod
    def load_path(tool):
        '''
        ' Method Description: Retrieves a set of commands from a command_set_alias stored as a csv file in the config folder.
        ' Author: Jacob Taylor Cassady
        '''
        path = list([])

        with open("config" + os.path.sep + "tool_paths" + os.path.sep + tool + ".csv") as tool_path:
            # read header
            command_line = tool_path.readline()
            # read first line
            command_line = tool_path.readline()

            while command_line:
                command_data = command_line.split(",")
                position_data = [command_data[3], command_data[4], command_data[5], command_data[6], command_data[7], command_data[8], command_data[9]]
                path.append(Command(command = command_data[1], motion_interpolation = command_data[2], position_type = "P", position_data = position_data))
                command_line = tool_path.readline()

        return path

if __name__ == "__main__":
    print("Testing directory" + os.getcwd() + "/../../../config/tool_paths/")
    print(FileManager.load_tool_paths())
