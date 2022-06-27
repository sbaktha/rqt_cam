#!/usr/bin/env python
"""
Launching nodes in ROS2 style using Python API
Author: Bakthakolahalan Shyamsundar
"""

import sys
import time
import subprocess

import rospy
import roslaunch

state = False
pkg_name = 'ecam_v4l2'
exec_name = 'img_pub_node'

class Node:
    """
    Class for defining a node and its operations
    """
    def __init__(self, package, executable, name, instance):
        self._package = package
        self._executable = executable
        self._name = name
        self._instance = instance
        self._launch = roslaunch.scriptapi.ROSLaunch()
        self._process = None
        self._node = None

    def define_node(self):
        """
        Define a node
        """
        self._node = roslaunch.core.Node(
                        package=self._package, node_type=self._executable, name=self._name,
                        namespace='/', machine_name=None,
                        args=str(self._instance),
                        respawn=False, respawn_delay=0.0,
                        remap_args=None,
                        env_args=None,
                        output='screen',
                        cwd=None,
                        launch_prefix=None,
                        required=False,
                        filename='<unknown>'
                        )

    def launch(self):
        """
        Function to launch a ROS node
        """
        self._launch.start()
        self._process = self._launch.launch(self._node)

    def kill(self):
        """
        Function to kill a node
        """
        self._process.stop()

    def status(self):
        """
        Function to check status of the node
        """
        if self._process.is_alive():
            return "Active"
        else:
            return "Inactive"

def launch_roscore():
    """
    Function to launch ROS core
    """
    subprocess.Popen('roscore')
    time.sleep(3)

def launch_node():
    """
    Function to launch any ROS node
    """
    package = pkg_name
    executable = exec_name
    node = roslaunch.core.Node(package, executable)

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    launch.launch(node)

def main(no_of_nodes):
    """
    Function to launch multiple instances of a node

    Args:
        no_of_nodes (Integer): No of instances needed
    """
    node_collections = []

    try:
        launch_roscore()
        global state

        package = pkg_name
        executable = exec_name

        # Creating multiple instances of a node
        for i in range(1, no_of_nodes+1):
            name = 'Nilecam_' + str(i)
            # Node() takes the arguments package, executable, node names and instance number
            node_collections.append(Node(package, executable, name, i))

            # Define and launch the nodes one by one
            node_collections[-1].define_node()
            node_collections[-1].launch()

        rospy.set_param('/activity_status', 1)

        # Check if the activity status was set
        while True:
            status = rospy.get_param('/activity_status')

            if status != 1:
                break
            else:
                pass

    except KeyboardInterrupt:
        state = True
        print("KeyboardInterrupt detected", state)

    except Exception as e:
        print(e)

    finally:
        print("Exiting the process ...")
        exit()
        # Nodes will be killed via an interrupt rospy.is_shutdown


if __name__ == '__main__':
    main(int(sys.argv[1]))