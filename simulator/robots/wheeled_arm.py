import os
from .mobile import MobileArmModel, MobileBaseModel

cwd = os.getcwd()

PATH_TO_ROBOT_MODEL = os.path.expanduser(cwd+'/models/robots/wheeled_base')
PATH_TO_ROBOT_XML = os.path.join(PATH_TO_ROBOT_MODEL, 'robot.xml')


WHEELED_BASE_MAP = {
    'joint': {
        'wheel_fr': 'joint_fr',
        'wheel_fl': 'joint_fl',
        'wheel_br': 'joint_br',
        'wheel_bl': 'joint_bl',
    },
    'actuator': {
        'wheel_fr': 'actuator_fr',
        'wheel_fl': 'actuator_fl',
        'wheel_br': 'actuator_br',
        'wheel_bl': 'actuator_bl',
    },
}

class WheeledBase(MobileBaseModel):
    def __init__(self, idn=0):
        super().__init__(fname=PATH_TO_ROBOT_XML, idn=idn)

    def _set_key_map(self):
        """
        Sets the key map for this gripper
        """

        self._key_map = {'joint': {}, 'actuator': {}}

        for key in WHEELED_BASE_MAP['joint'].keys():
            self._key_map['joint'].update({key: self.naming_prefix+WHEELED_BASE_MAP['joint'][key]})
            self._key_map['actuator'].update({key: self.naming_prefix+WHEELED_BASE_MAP['actuator'][key]})


class WheeledArm(MobileArmModel):
    def __init__(self, base, arms):
        super().__init__(base=base, arms=arms)

    @property
    def _base_name(self):
        return {'right': 'right_mount', 'left': 'left_mount'}

    @property
    def arm_type(self):
        return 'bimanual'
