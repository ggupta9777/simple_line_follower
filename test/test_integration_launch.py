#! /usr/bin/env python3

import time
import unittest
import os
import pytest

import launch
import launch_ros as lr
import launch.actions
import launch_testing.actions
import launch_testing.markers
from launch.actions import ExecuteProcess
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():

    os.environ['DISPLAY'] = ':0'

    # GTest script containing tests for checking activity of topics, nodes and topic rates
    testExecutable = os.path.join(get_workspace_path(), "build", "simple_line_follower", "test", "test_integration")

    # Executable script containing tests
    test_script = ExecuteProcess(
        cmd=[testExecutable, '--node-name', launch.substitutions.LaunchConfiguration('node-name')],
        name='test_integration',
        output='screen')

    # Launch argument to take the name of a node as input
    node_name_arg = launch.actions.DeclareLaunchArgument(
        'node-name',
        default_value=['/line_follower'],
        description='Name of the node to be tested',
    )
    
    # Line follower node
    line_follower_node = lr.actions.Node(
        package="simple_line_follower",
        executable="line_follower_node",
        output="screen"
    )
    
    # Naive line follower node
    naive_line_follower_node = lr.actions.Node(
        package="simple_line_follower",
        executable="naive_line_follower_node",
        output="screen"
    )

    return launch.LaunchDescription([
        node_name_arg, 
        line_follower_node,
        naive_line_follower_node,
        test_script,
        
        # Start the tests
        launch_testing.actions.ReadyToTest()
    ])

class DelayShutdown(unittest.TestCase):
    # Wait for 10 seconds for the GTest script to complete
    def test_delay(self):
        time.sleep(10)

class TestFixture(unittest.TestCase):
    # Check for presence of a ROS 2 Node
    def test_node_presence(self, proc_output):
        rclpy.init()
        node = Node('test_node')

        # Assert that the node exists
        assert wait_for_node(node, 'naive_line_follower', 8.0), 'Node not found !'
        
        rclpy.shutdown()

# These tests will run after the launched processes have been shut down
@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    # Check that all processes exited with code 0
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)


# Helper function to obtain the current workspace directory
def get_workspace_path():
    # Get package share directory
    package_name = 'simple_line_follower'
    package_path = os.path.dirname(get_package_share_directory(package_name))

    # Get the path to the workspace's directory
    workspace_path = os.path.join(package_path, "..", "..", "..")
    
    return workspace_path

# Helper function to assert the presence of a ROS 2 Node
def wait_for_node(dummy_node, node_name, timeout=8.0):
    start = time.time()
    flag = False
    print('Waiting for node...')
    while time.time() - start < timeout and not flag:
        flag = node_name in dummy_node.get_node_names()
        time.sleep(0.1)

    return flag
