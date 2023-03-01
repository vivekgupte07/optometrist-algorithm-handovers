# sawyer-handover-controller

## Handover Controlller for the Sawyer Robot Arm

This repository consists of a handover controller for the Sawyer robotic arm.

The robot picks up an object of fixed shape and hands it over to the collaborating human user.
The handover uses Moveit for trajectories and force sensing to detect the transfer, hence the user has to tug on the object before the gripper opens. The values of force thresholds are fixed for this particular object, and will have to be changed when using a different object.

In ``moveitcontroller`` you will find the code for just performing a single handover. Dynamic force thresholds are used, which are not 100% reliable. It is made more reliable in future versions that you will find on other branches. I hope to update the fixes here soon.

In ``personalization-stack`` and ``personalization-stack-experiment`` controller personalization for the experiment is added.


### Gripper

Gripper used: ROBOTIS RH-P12-RN(A) - [Documentation](https://emanual.robotis.com/docs/en/platform/rh_p12_rna)

[DNYAMIXEL_SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK)

```
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK
$ cd DynamixelSDK/c++/build/linux64
$ sudo make install
```
#### Packet Loss Issue with this Gripper: 
I faced an issue where the gripper would lose packets after a certain amount of use (it is probably just a local problem and might have a hardware fix)
I temporarily fixed it with reading the data again, in case an incomplete packet is read. Since this was a HRI task, the delay if any did not matter at all.


In ``DynamixelSDK/ros/dynamixel_sdk/src/dynamixel_sdk/protocol2_packet_handler.py`` make the following changes:
```python
def read4ByteTxRx(self, port, dxl_id, address):
        data, result, error = self.readTxRx(port, dxl_id, address, 4)
        while len(data)<4:
            data, result, error = self.readTxRx(port, dxl_id, address, 4)
        data_read = DXL_MAKEDWORD(DXL_MAKEWORD(data[0], data[1]),
                                  DXL_MAKEWORD(data[2], data[3])) if (result == COMM_SUCCESS) else 0
        return data_read, result, error
```

>Download the ROS GUI for this gripper - [ROS GUI Example from the ROBOTIS Manual](https://emanual.robotis.com/docs/en/platform/rh_p12_rna/examples/#ros-gui-example)
>ROS GUI will be useful to check the gripper motor parameters and to test it.
