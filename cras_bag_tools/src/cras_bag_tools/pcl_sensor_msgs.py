# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

""""Interop between sensor_msgs/PointCloud2 and PCD library types."""

from __future__ import print_function

from collections import namedtuple

from sensor_msgs.msg import PointCloud2, PointField


PclPointFieldType = namedtuple('PclPointFieldType', ['type_name', 'size'])


POINT_FIELD_DATATYPE_TO_PCL = {
    PointField.INT8: PclPointFieldType('I', 1),
    PointField.INT16: PclPointFieldType('I', 2),
    PointField.INT32: PclPointFieldType('I', 4),
    PointField.UINT8: PclPointFieldType('U', 1),
    PointField.UINT16: PclPointFieldType('U', 2),
    PointField.UINT32: PclPointFieldType('U', 4),
    PointField.FLOAT32: PclPointFieldType('F', 4),
    PointField.FLOAT64: PclPointFieldType('F', 8),
}


def get_pcd_header(msg):
    # type: (PointCloud2) -> str
    """Get the textual header of a PCD file corresponding to the given message.
    :param msg: The point cloud message.
    :return: The textual header (including the trailing newline preceding data).
    """
    fields_str = "FIELDS"
    size_str = "SIZE"
    type_str = "TYPE"
    count_str = "COUNT"

    offset = 0
    num_paddings = 0
    for f in msg.fields:
        pcl_type = POINT_FIELD_DATATYPE_TO_PCL[f.datatype]
        if f.offset != offset:
            fields_str += " padding" + str(num_paddings)
            size_str += " 1"
            type_str += " I"
            count_str += " " + str(f.offset - offset)
            num_paddings += 1
        fields_str += " " + f.name
        size_str += " " + str(pcl_type.size)
        type_str += " " + pcl_type.type_name
        count_str += " " + str(f.count)
        offset = f.offset + pcl_type.size

    header = """VERSION .7
%s
%s
%s
%s
WIDTH %i
HEIGHT %i
VIEWPOINT 0 0 0 1 0 0 0
POINTS %i
DATA binary
""" % (fields_str, size_str, type_str, count_str, msg.width, msg.height, msg.width * msg.height)
    return header


def convert_PointCloud2_to_pcd(msg):
    # type: (PointCloud2) -> bytes
    """Convert the given PointCloud2 message to PCD format.
    :param msg: The message to convert.
    :return: The PCD file data.
    """
    header = get_pcd_header(msg)
    return header.encode('utf-8') + msg.data
