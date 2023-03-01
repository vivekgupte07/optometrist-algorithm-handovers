#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy

from dynamixel_sdk import * # Uses Dynamixel SDK library


def gripper(grasp, duration=3.0):
    #********* DYNAMIXEL Model definition *********
    #***** (Use only one definition at a time) *****
    #MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
    # MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
    # MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
    MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
    # MY_DXL = 'P_SERIES'     # PH54, PH42, PM54
    # MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V


    # Control table address
    if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
        ADDR_TORQUE_ENABLE          = 64
        ADDR_GOAL_POSITION          = 116
        ADDR_PRESENT_POSITION       = 132
        DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
        DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
        BAUDRATE                    = 57600
    elif MY_DXL == 'PRO_SERIES':
        ADDR_TORQUE_ENABLE          = 512       # Control table address is different in DYNAMIXEL model
        ADDR_GOAL_POSITION          = 564
        ADDR_PRESENT_POSITION       = 580
        DXL_MINIMUM_POSITION_VALUE  = 0   # Refer to the Minimum Position Limit of product eManual
        DXL_MAXIMUM_POSITION_VALUE  = 1150    # Refer to the Maximum Position Limit of product eManual
        BAUDRATE                    = 2000000
    elif MY_DXL == 'P_SERIES' or MY_DXL == 'PRO_A_SERIES':
        ADDR_TORQUE_ENABLE          = 512   # Control table address is different in DYNAMIXEL model
        ADDR_GOAL_POSITION          = 564
        ADDR_PRESENT_POSITION       = 580
        DXL_MINIMUM_POSITION_VALUE  = 0     # Refer to the Minimum Position Limit of product eManual
        DXL_MAXIMUM_POSITION_VALUE  = 501  # Refer to the Maximum Position Limit of product eManual
        BAUDRATE                    = 2000000
    elif MY_DXL == 'XL320':
        ADDR_TORQUE_ENABLE          = 24
        ADDR_GOAL_POSITION          = 30
        ADDR_PRESENT_POSITION       = 37
        DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the CW Angle Limit of product eManual
        DXL_MAXIMUM_POSITION_VALUE  = 1023      # Refer to the CCW Angle Limit of product eManual
        BAUDRATE                    = 2000000   # Default Baudrate of XL-320 is 1Mbps

    # DYNAMIXEL Protocol Version (1.0 / 2.0)
    # https://emanual.robotis.com/docs/en/dxl/protocol2/
    PROTOCOL_VERSION            = 2.0

    # Factory default ID of all DYNAMIXEL is 1
    DXL_ID                      = 1

    # Use the actual port assigned to the U2D2.
    # ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
    DEVICENAME                  = '/dev/ttyUSB0'

    TORQUE_ENABLE               = 1     # Value for enabling the torque
    TORQUE_DISABLE              = 0     # Value for disabling the torque
    DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

    if grasp == 'close':
        index = 1
        rospy.loginfo('Gripper Closing')
    elif grasp == 'open':
        index = 0
        rospy.loginfo('Gripper Opening')
    else:
        index = 0

    dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE] # Goal position


    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Open port
    if portHandler.openPort():
        #rospy.loginfo("Succeeded to open the port")
        pass
    else:
        rospy.logerr("Failed to open the port")
        quit()


    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        pass
        #rospy.loginfo("Succeeded to change the baudrate")
    else:
        rospy.logerr("Failed to change the baudrate")
        quit()

    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    

    start_time = rospy.get_rostime().secs
    while True:
        # Write goal position
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position[index])
        
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        while rospy.get_rostime().secs - start_time < duration:
            # Read present position
            dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
            
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))

            if not abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
                break
        break

    if grasp=='open':
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Close port
    portHandler.closePort()
