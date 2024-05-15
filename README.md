# BehaviorTree.ROS

[BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) is a __middleware independent__ framework 
to develop Behavior Trees in C++.

The library is not particularly opinionated about the way Actions and Conditions should be created; this gives
more freedom to the developer, but can also be confusing for those people which are getting started with it.

Consequently, many people in the [ROS](http://www.ros.org) community asked for examples and guidelines;
this repository try to provide some basic examples.

Currently, two wrappers are provided:

- [RosServiceNode](include/behaviortree_ros/bt_service_node.h), which can be used to call
  [ROS Services](http://wiki.ros.org/Services)

- [RosActionNode](include/behaviortree_ros/bt_action_node.h) that, similarly, is a wrapper around
  [actionlib::SimpleActionClient](http://wiki.ros.org/actionlib).
# How to run an action and a service with BehaviorTree.ROS and monitor it on Groot2
The action that is ran is the default test_bt that is given by the behaviorTree.ROS library. It runs the AddTwoInts service and the Fibonacci sequence action. The intern modified the test_bt.cpp file in "/test" to make it more visible on Groot2 by slowing down the the tasks. He also included the groot2 publisher and the xml parsing files from the behaviorTree.CPP library. Theses helped him to publish the code on the channel that Groot2 monitors. 

The intern proved that you can start an xml file from groot2 and make it easily compatible with the test_bt.cpp example. 

# Steps accomplished by the developper to use Groot2 as an interface to start biological expirements. 

The developper makes the cpp file and creates leaf nodes on Groot2 that are compatible with the cpp file. The nodes can also be imported from an existing xml file with the script language. 
With the user interface : 

![01](https://github.com/berg0401/stage_t5/assets/72279192/5c51b833-daf5-4569-9dd1-8dcea6799b80)

Or with the imported xml_files :

![04_installer_à_partir_xml_des_nodes](https://github.com/berg0401/stage_t5/assets/72279192/75478060-d6c2-4254-a4ff-3397c853ceba)


He adds the input and ouptut ports corresponding to the nodes. 
![02](https://github.com/berg0401/stage_t5/assets/72279192/f17b912e-485a-44f4-a772-0b80f2523443)

The developper must put a default value to the input or output ports like the server name because the biologists that will use groot2 to create their experiences have no clue of the meaning of this input. 

When the node are added, they are now available to drag and drop in the groot behavior Tree

![03](https://github.com/berg0401/stage_t5/assets/72279192/b2da66df-8a3e-45ea-ae95-559545a9d0ab)






