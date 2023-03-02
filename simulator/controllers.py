from collections import OrderedDict
import numpy as np
import copy
from util import geom


class WheeledArmController(object):

    def __init__(self) -> None:

        self._robot_target = {'joint_pos': OrderedDict(),
                              'joint_vel': OrderedDict(),
                              'joint_trq': OrderedDict(),
                              'body_pos': OrderedDict(),
                              'body_vel': OrderedDict()}
        self._left_gripper_target = {'joint_pos': OrderedDict(),
                                     'joint_vel': OrderedDict(),
                                     'joint_trq': OrderedDict()}
        self._right_gripper_target = {'joint_pos': OrderedDict(),
                                      'joint_vel': OrderedDict(),
                                      'joint_trq': OrderedDict()}                           
        self._hand_target = {'left': OrderedDict(),
                             'right': OrderedDict()}
        self._sensor_data = OrderedDict()
        self._init_sate = OrderedDict()
        # self._init_sate.update(config['Simulation']['Initial State'])
        self._init_sate = {
            'Joint Pos': {'base_wheel_{}'.format(key): 0.0 for key in ['fl', 'fr', 'bl', 'br']},
            'Body Pos': [0.0, 0.0, 0.0],
            'Body Quat': [0.0, 0.0, 0.0, 1.0]
        }
        self._init_sate['Joint Pos'].update({'left_joint_{}'.format(idx): value 
                    for idx, value in enumerate([-0.5*np.pi, -0.5*np.pi, 0.5*np.pi, 0.5*np.pi, 0.0, 1.0*np.pi, 0.0])})
        self._init_sate['Joint Pos'].update({'right_joint_{}'.format(idx): value 
                    for idx, value in enumerate([0.5*np.pi, -0.5*np.pi, -0.5*np.pi, 0.5*np.pi, 0.0, 1.0*np.pi, 0.0])})
        self.reset()


    def reset(self):

        self._robot_target['joint_pos'].update(self._init_sate['Joint Pos'])
        self._robot_target['joint_vel'].update({key: 0.0 for key in self._robot_target['joint_pos'].keys()})
        self._robot_target['joint_trq'].update({key: 0.0 for key in self._robot_target['joint_pos'].keys()})
        self._robot_target['body_pos'].update({'pos': np.array(self._init_sate['Body Pos']), 
                                               'quat': np.array(self._init_sate['Body Quat'])})
        self._robot_target['body_vel'].update({'pos': np.zeros(3), 'rpy': np.zeros(3)})

        self._right_gripper_target['joint_pos'].update({'gripper': 0.0})
        self._right_gripper_target['joint_vel'].update({key: 0.0 for key in self._right_gripper_target['joint_pos'].keys()})
        self._right_gripper_target['joint_trq'].update({key: 0.0 for key in self._right_gripper_target['joint_pos'].keys()})

        self._left_gripper_target['joint_pos'].update({'gripper': 0.0})
        self._left_gripper_target['joint_vel'].update({key: 0.0 for key in self._left_gripper_target['joint_pos'].keys()})
        self._left_gripper_target['joint_trq'].update({key: 0.0 for key in self._left_gripper_target['joint_pos'].keys()})

        self._hand_target['right'].update({'pos': np.array([0.3, -0.3, 0.3]), 
                                           'quat': np.array([0.0, 0.70710678, 0.0, -0.70710678])})
        self._hand_target['left'].update({'pos': np.array([0.3, 0.3, 0.3]), 
                                          'quat': np.array([0.0, 0.70710678, 0.0, -0.70710678])})


    def update_sensor_data(self, sensor_data):
        pass

    def get_control(self):
        pass

    def standby(self):
        pass

