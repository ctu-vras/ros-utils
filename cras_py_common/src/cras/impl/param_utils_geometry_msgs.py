# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""geometry_msgs parsers for ROS parameters."""

from functools import partial

import rospkg

from cras.string_utils import to_str
import cras.param_utils as param_utils


def __construct_type_from_list(target_type, expected_length, value):
    if not isinstance(value, list):
        raise ValueError("Cannot construct %s from value of type %s" % (target_type, to_str(type(value))))
    if len(value) != expected_length:
        raise ValueError("Parsing parameter of type %s requires %i items, but %i were given." % (
            to_str(target_type), expected_length, len(value)))
    return target_type(*value)


quat_keys = {"x", "y", "z", "w", "r", "p", "roll", "pitch", "yaw"}
try:
    from tf.transformations import quaternion_from_euler, quaternion_from_matrix
    from cras.geometry_utils import quat_tuple_from_rpy

    def __construct_quat_from_list(target_type, value):
        if not isinstance(value, list):
            raise ValueError("Cannot construct a quaternion from value of type %s" % (to_str(type(value)),))
        if len(value) not in (3, 4, 9):
            raise ValueError("Parsing parameter of type quaternion requires 3, 4 or 9 items, but %i were given." % (
                len(value),))
        if len(value) == 3:
            return target_type(*quat_tuple_from_rpy(value))
        elif len(value) == 4:
            return target_type(*value)
        else:
            matrix = [value[0:3] + [0], value[3:6] + [0], value[6:9] + [0], [0, 0, 0, 1]]
            return target_type(*quaternion_from_matrix(matrix))

    def __construct_quat_from_dict(target_type, value):
        if not isinstance(value, dict):
            raise ValueError("Cannot construct a quaternion from value of type %s" % (to_str(type(value)),))
        if len(value) not in (1, 2, 3, 4):
            raise ValueError("Parsing parameter of type quaternion requires a dictionary with 1-4 items, "
                             "but %i were given." % (len(value),))
        if len(value) == 1 and "y" in value:
            raise ValueError("Cannot initialize quaternion from dict with only 'y' element - it is ambiguous.")
        for k in value:
            if k not in quat_keys:
                raise ValueError("Unknown quaternion key %s." % (k,))
        if "x" in value or "z" in value or "w" in value:
            return target_type(*(value.get("x", 0), value.get("y", 0), value.get("z", 0), value.get("w", 0)))
        elif "r" in value or "p" in value:
            return target_type(*quat_tuple_from_rpy(value.get("r", 0), value.get("p", 0), value.get("y", 0)))
        elif "roll" in value or "pitch" in value or "yaw" in value:
            return target_type(*quat_tuple_from_rpy(value.get("roll", 0), value.get("pitch", 0), value.get("yaw", 0)))
        else:
            raise ValueError("Parsing parameter of type quaternion requires a dictionary with items x/y/z/w, r/p/y "
                             "or roll/pitch/yaw, but %s was given." % ("/".join(value.keys()),))

except (ImportError, rospkg.ResourceNotFound):
    def __construct_quat_from_list(target_type, value):
        if not isinstance(value, list):
            raise ValueError("Cannot construct a quaternion from value of type %s" % (to_str(type(value)),))
        if len(value) != 4:
            raise ValueError("Parsing parameter of type quaternion requires 4 items, but %i were given. Construction "
                             "from 3 or 9 elements is supported only with 'tf' package installed." % (len(value),))
        return target_type(*value)

    def __construct_quat_from_dict(target_type, value):
        if not isinstance(value, dict):
            raise ValueError("Cannot construct a quaternion from value of type %s" % (to_str(type(value)),))
        if len(value) not in (1, 2, 3, 4):
            raise ValueError("Parsing parameter of type quaternion requires a dictionary with 1-4 items, "
                             "but %i were given." % (len(value),))
        if len(value) == 1 and "y" in value:
            raise ValueError("Cannot initialize quaternion from dict with only 'y' element - it is ambiguous.")
        for k in value:
            if k not in quat_keys:
                raise ValueError("Unknown quaternion key %s." % (k,))
        if "x" in value or "z" in value or "w" in value:
            return target_type(*(value.get("x", 0), value.get("y", 0), value.get("z", 0), value.get("w", 0)))
        else:
            raise ValueError("Parsing parameter of type quaternion requires a dictionary with items x/y/z/w, "
                             "but %s was given. Construction from r/p/y or roll/pitch/yaw is supported only with 'tf' "
                             "package installed." % ("/".join(value.keys()),))


def __construct_so3_from_list(tf_type, pose_type, quat_type, value):
    if not isinstance(value, list):
        raise ValueError("Cannot construct %s from value of type %s" % (to_str(tf_type), to_str(type(value))))
    if len(value) not in (6, 7, 16):
        raise ValueError("Parsing parameter of type %s requires 6, 7 or 16 items, but %i were given." % (
            to_str(tf_type), len(value)))
    if len(value) == 16:
        if value[12:] != [0, 0, 0, 1]:
            raise ValueError("Transformation encoded as homogeneous matrix has to have last row 0,0,0,1.")
        pose = __construct_type_from_list(pose_type, 3, [value[i] for i in (3, 7, 11)])
        quat = __construct_quat_from_list(quat_type, [value[i] for i in (0, 1, 2, 4, 5, 6, 8, 9, 10)])
    else:
        pose = __construct_type_from_list(pose_type, 3, value[0:3])
        quat = __construct_quat_from_list(quat_type, value[3:])
    return tf_type(pose, quat)


def __parse_vector(target_type, value):
    if isinstance(value, list):
        return __construct_type_from_list(target_type, 3, value)
    else:
        return target_type(**value)


def __parse_quaternion(target_type, value):
    if isinstance(value, list):
        return __construct_quat_from_list(target_type, value)
    else:
        return __construct_quat_from_dict(target_type, value)


try:
    import geometry_msgs.msg as m

    param_utils.register_param_conversion(m.Vector3, list, partial(__construct_type_from_list, m.Vector3, 3))
    param_utils.register_param_conversion(m.Vector3, dict, lambda d: m.Vector3(**d))
    param_utils.register_param_conversion(m.Point, list, partial(__construct_type_from_list, m.Point, 3))
    param_utils.register_param_conversion(m.Point, dict, lambda d: m.Point(**d))
    param_utils.register_param_conversion(m.Point32, list, partial(__construct_type_from_list, m.Point32, 3))
    param_utils.register_param_conversion(m.Point32, dict, lambda d: m.Point32(**d))
    param_utils.register_param_conversion(m.Pose2D, list, partial(__construct_type_from_list, m.Pose2D, 3))
    param_utils.register_param_conversion(m.Pose2D, dict, lambda d: m.Pose2D(**d))
    param_utils.register_param_conversion(m.Quaternion, list, partial(__construct_quat_from_list, m.Quaternion))
    param_utils.register_param_conversion(m.Quaternion, dict, partial(__construct_quat_from_dict, m.Quaternion))
    param_utils.register_param_conversion(m.Pose, list,
                                          partial(__construct_so3_from_list, m.Pose, m.Point, m.Quaternion))
    param_utils.register_param_conversion(m.Pose, dict, lambda d: m.Pose(
        __parse_vector(m.Point, d["position"]), __parse_quaternion(m.Quaternion, d["orientation"])))
    param_utils.register_param_conversion(m.Transform, list,
                                          partial(__construct_so3_from_list, m.Transform, m.Vector3, m.Quaternion))
    param_utils.register_param_conversion(m.Transform, dict, lambda d: m.Transform(
        __parse_vector(m.Vector3, d["translation"]), __parse_quaternion(m.Quaternion, d["rotation"])))
except ImportError:
    pass
