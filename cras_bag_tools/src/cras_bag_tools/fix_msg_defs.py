from __future__ import print_function

import sys

import roslib.message
from rosbag.bag import Bag


def fix_msg_defs(bag, topics=None):
    """In some cases, wrong message definitions are stored to bag files and they do not match the
    MD5 sums. This function goes through an open bag file and adds the missing definitions
    (according to the definitions found in the currently sourced workspace).

    :param rosbag.Bag bag: The bag file.
    :param List[str] topics: The topics to fix. Leave out to fix all topics.
    """
    assert isinstance(bag, Bag)
    # fix message definitions (they're not stored recursively sometimes)
    connections = bag._get_connections(topics=topics if len(topics) > 0 else None)
    msg_def_maps = dict()
    msg_md5_maps = dict()
    for connection in connections:
        msg_type = connection.datatype
        if msg_type not in msg_def_maps:
            sys_class = roslib.message.get_message_class(msg_type)
            if sys_class is None:
                sys_class = roslib.message.get_service_class(msg_type)
                if sys_class is None:
                    print("Message class '" + msg_type + "' not found.", file=sys.stderr)
                    continue
            msg_def_maps[msg_type] = sys_class._full_text
            msg_md5_maps[msg_type] = sys_class._md5sum

        # here, we either already had a mapping or one was just created
        full_msg_text = msg_def_maps[msg_type]
        msg_md5 = msg_md5_maps[msg_type]

        if connection.md5sum != msg_md5:
            print("Message class '%s' has stored MD5 %s but local MD5 is %s."
                  "Run rosbag fix first or update your message definitions!" %
                  (msg_type, connection.md5sum, msg_md5))
            continue

        # don't touch anything if not needed (final newlines may differ between gencpp and genpy)
        if connection.msg_def.rstrip("\n") == full_msg_text.rstrip("\n"):
            continue

        print("<<<")
        print("Replacing definition of %s (%i chars, MD5 %s)" %
              (msg_type, len(connection.msg_def), connection.md5sum))
        print(connection.msg_def.replace("\n", "\\n"))
        print("with following definition (%i chars, matching MD5)" % (len(full_msg_text,)))
        print(full_msg_text.replace("\n", "\\n"))
        print(">>>")

        # here we really should replace the msg def, so do it
        connection.header['message_definition'] = full_msg_text
        connection.msg_def = full_msg_text
