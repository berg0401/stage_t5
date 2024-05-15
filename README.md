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
The action that is ran is the default test_bt that is given by the behaviorTree.ROS library. It runs the AddTwoInts service and the Fibonacci sequence action. The intern modified the test_bt.cpp file in "/test" to make it more visible on Groot2 by slowing down the the tasks. He also included the groot2 publisher and the xml parsing files from the behaviorTree.CPP library. Theses helped him to publish the code on the channel that Groot2 monitors. He modified also the test_server.launch to launc the server and the client with one command.

The intern proved that you can start an xml file from groot2 and make it easily compatible with the test_bt.cpp example. 

## Steps accomplished by the developper to use Groot2 as an interface to start biological expirements 

The developper makes the cpp file and creates leaf nodes on Groot2 that are compatible with the cpp file. The nodes can also be imported from an existing xml file with the script language. 
With the user interface : 

![01](https://github.com/berg0401/stage_t5/assets/72279192/5c51b833-daf5-4569-9dd1-8dcea6799b80)

Or with the imported xml_files :

![04_installer_à_partir_xml_des_nodes](https://github.com/berg0401/stage_t5/assets/72279192/75478060-d6c2-4254-a4ff-3397c853ceba)


He adds the input and ouptut ports corresponding to the nodes. 
![02](https://github.com/berg0401/stage_t5/assets/72279192/f17b912e-485a-44f4-a772-0b80f2523443)

The developper must put a default value to the input or output ports like the server name because the biologists that will use groot2 to create their experiences have no clue of the meaning of this input. 

When the nodes are added, they are now available to drag and drop in the groot behavior Tree

![03](https://github.com/berg0401/stage_t5/assets/72279192/b2da66df-8a3e-45ea-ae95-559545a9d0ab)

## Steps required to create a biological expirement from Groot2 by a non-developper
To demonstrate the power or behaviorTree quickly, the service and the action acccomplished by the test_bt.cpp file has nothing to do with biology. It will add two inputs as a service and return the fibonacci sequence as an action. If a biologist wanted to run this expirement, he would have to follow these steps.

He can drag and drop the leaf nodes that he needs :

![01](https://github.com/berg0401/stage_t5/assets/72279192/6728f772-d5e0-4655-a843-1672dfb60461)
![02](https://github.com/berg0401/stage_t5/assets/72279192/a13363f5-61e5-4ffd-94e7-5949cf7307c0)

He can create his sequence using the Groot2 tools like sequence or timeout and linked them following the behaviorTree standards : 

![03](https://github.com/berg0401/stage_t5/assets/72279192/69395fff-4b56-4f44-bb38-8fa2f64d5bd6)

He can then add the inputs to the actions depending on the results that he wants. If it was a biological exprirement, theses inputs could be "solvent_quantity = 10". 

![04](https://github.com/berg0401/stage_t5/assets/72279192/64c196cf-4ea7-4d22-8583-ada08daa2274)

He then saves the project in the appropiate path : 

![05](https://github.com/berg0401/stage_t5/assets/72279192/d3cd6509-40e1-491e-99cc-5056435669f9)

This step generates a xml file : 

![06_exemple_de_fichier](https://github.com/berg0401/stage_t5/assets/72279192/62edb994-6f3e-4b95-b326-f120fa322b6c)

The appropriate path is the same than the one in the cpp file that executes the experiment : 

![07_comment_cpp_va_le_chercher](https://github.com/berg0401/stage_t5/assets/72279192/3eccc827-5570-4244-8407-61906b954db1)

To launch the expirement and the robot, the biologist must open a first terminal, wirte roscore. Then, he must open a new terminal, and write the following command : 

![08_partir_l'executable](https://github.com/berg0401/stage_t5/assets/72279192/60e65965-58df-4970-a301-a91940f2acf1)

To monitor the expirment in real-time, he must go on monitor mode in Groot2 : 

![09](https://github.com/berg0401/stage_t5/assets/72279192/ffd861e6-8561-4f61-a9a9-09106981dde8)

He then presses the disconnected button to connect to the default port 1667. 

![10](https://github.com/berg0401/stage_t5/assets/72279192/3649e225-87ec-4fa2-9af9-014c643533c4)

He can then see the flow of the experiment threw a behavior three : 

![11](https://github.com/berg0401/stage_t5/assets/72279192/eff500cf-0fa9-48c5-aa9d-be1423523ddd)


















