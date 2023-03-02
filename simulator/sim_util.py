from collections import OrderedDict

import numpy as np

from util import geom
from util import liegroup
import mujoco


def get_mujoco_objects(sim):
    return sim.model._model, sim.data._data


def get_link_iso(sim, robot, link_name):
    model, data = get_mujoco_objects(sim)
    link_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_XBODY, link_name)
    pos = np.array(data.xpos[link_id])
    quat = np.array(data.xquat[link_id])[[1,2,3,0]]
    rot = geom.quat_to_rot(quat)
    return liegroup.RpToTrans(rot, pos)


def get_link_vel(sim, robot, link_name):
    model, data = get_mujoco_objects(sim)
    link_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_XBODY, link_name)
    velp = np.copy(data.cvel(link_id))[:3]
    velr = np.copy(data.cvel(link_id))[3:]
    vel = np.concatenate((velr, velp))
    return vel


def set_motor_impedance(sim, robot, command, kp, kd):
    model, data = get_mujoco_objects(sim)
    key_map = robot.key_map
    trq_applied = OrderedDict()
    for (pnc_key, pos_des), (_, vel_des), (_, trq_des) in zip(
            command['joint_pos'].items(), command['joint_vel'].items(),
            command['joint_trq'].items()):
        mujoco_joint_key = key_map['joint'][pnc_key]
        mujoco_actuator_key = key_map['actuator'][pnc_key]
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, mujoco_joint_key)
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, mujoco_actuator_key)
        joint_qposadr = model.jnt_qposadr[joint_id]
        joint_qveladr = model.jnt_dofadr[joint_id]
        joint_pos = data.qpos[joint_qposadr]
        joint_vel = data.qvel[joint_qveladr]
        if type(kp) == dict: kp_val = kp[pnc_key]
        else: kp_val = kp
        if type(kd) == dict: kd_val = kd[pnc_key]
        else: kd_val = kd
        trq_applied[actuator_id] = trq_des \
                + kp_val * (pos_des - joint_pos)\
                + kd_val * (vel_des - joint_vel)
    data.ctrl[list(trq_applied.keys())] = list(trq_applied.values())


def set_motor_trq(sim, robot, command):
    model, data = get_mujoco_objects(sim)
    key_map = robot.key_map
    trq_applied = OrderedDict()
    for pnc_key, trq_des in command['joint_trq'].items():
        mujoco_actuator_key = key_map['actuator'][pnc_key]
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, mujoco_actuator_key)
        # print(actuator_id)
        trq_applied[actuator_id] = trq_des
    data.ctrl[list(trq_applied.keys())] = list(trq_applied.values())


def set_motor_pos_vel(sim, robot, state):
    model, data = get_mujoco_objects(sim)
    key_map = robot.key_map
    pos_applied = OrderedDict()
    vel_applied = OrderedDict()
    for (pnc_key, pos_des), (_, vel_des) in zip(state['joint_pos'].items(), state['joint_vel'].items()):
        mujoco_joint_key = key_map['joint'][pnc_key]
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, mujoco_joint_key)
        joint_qposadr = model.jnt_qposadr[joint_id]
        joint_qveladr = model.jnt_dofadr[joint_id]
        pos_applied[joint_qposadr] = pos_des
        vel_applied[joint_qveladr] = vel_des
    data.qpos[list(pos_applied.keys())] = list(pos_applied.values())
    data.qvel[list(vel_applied.keys())] = list(vel_applied.values())


def set_body_pos_vel(sim, robot, state):
    model, data = get_mujoco_objects(sim)
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, robot.naming_prefix+'root')
    joint_qposadr = model.jnt_qposadr[joint_id]
    joint_qveladr = model.jnt_dofadr[joint_id]
    data.qpos[joint_qposadr:joint_qposadr+7] = np.concatenate((state['body_pos']['pos'], state['body_pos']['quat'][[3, 0, 1, 2]]))
    data.qvel[joint_qveladr:joint_qveladr+6] = np.concatenate((state['body_vel']['pos'], state['body_vel']['rpy']))


def skew_symmetric(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])




def get_joint_state(sim, robot):
    
    model, data = get_mujoco_objects(sim)
    key_map = robot.key_map
    joint_data = OrderedDict()

    joint_data['joint_pos'] = OrderedDict()
    joint_data['joint_vel'] = OrderedDict()
    for pnc_key in key_map['joint'].keys():
        mujoco_key = key_map['joint'][pnc_key]
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, mujoco_key)
        joint_qposadr = model.jnt_qposadr[joint_id]
        joint_qveladr = model.jnt_dofadr[joint_id]
        joint_data['joint_pos'][pnc_key] = data.qpos[joint_qposadr]
        joint_data['joint_vel'][pnc_key] = data.qvel[joint_qveladr]

    return joint_data
