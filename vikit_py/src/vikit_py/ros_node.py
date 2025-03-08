#!/usr/bin/python3

import os

class RosNode:
    def __init__(self, package, executable):
        self._package = package
        self._executable = executable
        self._param_string = ''
    
    def add_parameters(self, namespace, parameter_dictionary):
        for key in parameter_dictionary.keys():
            if type(parameter_dictionary[key]) is dict:
                self.add_parameters(namespace+key+'/', parameter_dictionary[key])
            else:
                # ROS 2 uses double dash for parameters
                self._param_string += ' --' + namespace + key + ' ' + str(parameter_dictionary[key])
        
    def run(self, parameter_dictionary, namespace=''):
        self.add_parameters(namespace, parameter_dictionary)
        print('Starting ROS 2 node with parameters: ' + self._param_string)
        
        # Use ros2 run instead of rosrun
        os.system('ros2 run ' + self._package + ' ' + self._executable + ' ' + self._param_string)
        print('ROS 2 node finished processing.')