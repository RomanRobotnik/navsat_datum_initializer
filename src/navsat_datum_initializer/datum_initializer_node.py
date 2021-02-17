#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from datum_initializer import NavSatDatumInitializer


def main():

    rospy.init_node("navsat_datum_initializer_node")

    rc_node = NavSatDatumInitializer()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()
