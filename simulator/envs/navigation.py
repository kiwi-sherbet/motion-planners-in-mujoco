import os
import sys
import numpy as np
import mujoco

cwd = os.getcwd()
sys.path.append(cwd)

from simulator.arenas import NavigationArena
from simulator.robots import WheeledBase, WheeledArm
from simulator.grippers import Robotiq2F85
from simulator.controllers import WheeledArmController
import simulator.sim_util as sim_util

from robosuite.models.robots.manipulators import Panda
from robosuite.models.base import MujocoXML
from robosuite.utils.binding_utils import MjSim

from robosuite.models.objects import BoxObject, CylinderObject

PATH_TO_WORLD_XML = os.path.expanduser(cwd+'/models/base.xml')
SIM_TIME = 0.002
TELEOP_TIME = 0.05
RENDER_TIME = 0.03
INIT_TIME = 10 * SIM_TIME


PANDA_MAP = {
    'joint': {
        'joint_0':  'joint1',
        'joint_1':  'joint2',
        'joint_2':  'joint3',
        'joint_3':  'joint4',
        'joint_4':  'joint5',
        'joint_5':  'joint6',
        'joint_6':  'joint7',
    },

    'actuator': {
        'joint_0':  'torq_j1',
        'joint_1':  'torq_j2',
        'joint_2':  'torq_j3',
        'joint_3':  'torq_j4',
        'joint_4':  'torq_j5',
        'joint_5':  'torq_j6',
        'joint_6':  'torq_j7',
    }    
}

def set_arm_key_map(arm, key_map):

    arm.key_map = {'joint': {}, 'actuator': {}}

    for key in key_map['joint'].keys():
        arm.key_map['joint'].update({key: arm.naming_prefix+key_map['joint'][key]})
        arm.key_map['actuator'].update({key: arm.naming_prefix+key_map['actuator'][key]})


class NavigationEnv():

    def __init__(self, obj_info={'obstacle':{}, 'goal':{}, 'start':{}}) -> None:

        self.obj_info = obj_info
        self.placement_initializer = None
        self._load_model()
        model = self.world.get_model(mode="mujoco")
        self.sim = MjSim(model)
        self.renderer = None
        self.recorder = None
        self.controller = WheeledArmController()

        # self.geom_ground_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, self.robot.naming_prefix+'root')
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, self.robot.naming_prefix+'root')
        joint_qposadr = self.sim.model.jnt_qposadr[joint_id]

        self.conf_ids = [joint_qposadr+idx for idx in range(2)]
        self.conf_region = np.array(((0, 0), (10, 10)))

        # self.mask_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, self.robot.naming_prefix+'root')]

        self.geom_info = {'obstacle':{}, 'robot':{}, 'ground':{}}

        for i in range(self.sim.model.ngeom):
            key = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)

            filter = False
            for word in ('start', 'goal', '_viz', '_vis', '_visual'):
                if word in key:
                    filter = True
            if filter:
                continue
            if key.startswith('robot'):
                self.geom_info['robot'][key] = i
            elif key.startswith('gripper'):
                self.geom_info['robot'][key] = i
            elif key in ('ground', 'floor'):
                self.geom_info['ground'][key] = i
            else:
                self.geom_info['obstacle'][key] = i

        print(self.geom_info['obstacle'])
        print(self.geom_info['robot'])
        print(self.geom_info['ground'])


    def reset(self):

        sim_util.set_body_pos_vel(self.sim, self.robot, self.controller._robot_target)
        sim_util.set_motor_pos_vel(self.sim, self.robot, self.controller._robot_target)
        # sim_util.set_motor_trq(self.sim, self.robot, self.controller._robot_target)
        # sim_util.set_motor_impedance(self.sim, self.grippers['left'], self.controller._left_gripper_target, 10.0e-5, 2)
        # sim_util.set_motor_impedance(self.sim, self.grippers['right'], self.controller._right_gripper_target, 10.0e-5, 2)

        self._reset_objects()

        self.sim.forward()

        self._cur_sim_time = 0.0
        self._cur_render_time = 0.0
        self._cur_teleop_time = 0.0

        while self._cur_sim_time < INIT_TIME:
            self.sim.forward()
            self._cur_sim_time += SIM_TIME


    def step(self, action):

        position = np.array([0, 0, 0.15])
        position[0:2] = action

        while self._cur_sim_time - self._cur_teleop_time < TELEOP_TIME:

            self.controller._robot_target['body_pos'].update({'pos': position})
            sim_util.set_body_pos_vel(self.sim, self.robot, self.controller._robot_target)
            self.sim.forward()
            if self._cur_sim_time - self._cur_render_time >= RENDER_TIME:
                self._render()
                self._cur_render_time += RENDER_TIME

            self._cur_sim_time += SIM_TIME

        self._cur_teleop_time += TELEOP_TIME


    def _load_model(self):

        self.world = MujocoXML(PATH_TO_WORLD_XML)
        self.arena = NavigationArena()
        self.world.merge(self.arena)

        self.objects = {'goal': {}, 'start': {}, 'obstacle': {}}
        for obj_type in self.obj_info.keys():
            for obj_key, info in self.obj_info[obj_type].items():
                info_args = {'size':info['size'], 'rgba':info['rgba'], 'name':obj_key}
                if info['shape'] == 'box':
                    obj_obstacle = BoxObject(**info_args)
                elif info['shape'] == 'cylinder':
                    obj_obstacle = CylinderObject(**info_args)
                self.world.merge_assets(obj_obstacle)
                self.world.worldbody.append(obj_obstacle.get_obj())
                self.objects[obj_type][obj_key] = obj_obstacle

        self.base = WheeledBase()
        self.right_arm = Panda(idn=1)
        self.left_arm = Panda(idn=2)
        
        set_arm_key_map(self.left_arm, PANDA_MAP)
        set_arm_key_map(self.right_arm, PANDA_MAP)

        self.robot = WheeledArm(self.base, {'right': self.right_arm, 'left': self.left_arm})
        self.grippers={}
        for key in ['left', 'right']:
            self.grippers[key] = Robotiq2F85(idn=key)
            self.robot.add_gripper(self.grippers[key], arm_name=self.robot._eef_name[key])

        self.world.merge(self.robot)


    def _reset_objects(self):

        for obj_type in self.objects.keys():
            for obj_key, object in self.objects[obj_type].items():
                joint_id = self.sim.model.joint_name2id(object.naming_prefix+'joint0')
                joint_qposadr = self.sim.model.jnt_qposadr[joint_id]
                self.sim.data.qpos[joint_qposadr:joint_qposadr+7] = np.copy(self.obj_info[obj_type][obj_key]['pose'])

    @property
    def cur_time(self):
        return self._cur_sim_time


    @property
    def done(self):
        return False


    @property
    def subtask(self):
        return self._subtask


    def render(self):
        return self._render()


    def set_renderer(self, renderer):
        self.renderer = renderer


    def set_recorder(self, recorder):
        self.recorder = recorder


    def _render(self):
        if self.renderer == None:
            return
        else:
            return self.renderer.render()


    def _record(self, **kwargs):
        if self.recorder == None:
            return
        else:
            return self.recorder.record(**kwargs)

    def _reset_recorder(self, **kwargs):
        if self.recorder == None:
            return
        else:
            return self.recorder.reset(**kwargs)

    def _reset_initial_qpos(self, initial_qpos):
        self.sim.data.qpos[:] = np.copy(initial_qpos)