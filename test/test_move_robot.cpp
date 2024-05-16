#include <behaviortree_ros/bt_action_node.h>
#include <ros/ros.h>
#include <behaviortree_ros/move_robotAction.h>
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/xml_parsing.h"
#include <chrono>

using namespace BT;
namespace chr = std::chrono;

struct PoseTCP

{
    double x, y, z, Roll, Pitch, Yaw;
};



namespace BT
{
    template <> inline PoseTCP convertFromString(StringView str)
    {
        auto parts = splitString(str,';');
        if (parts.size() != 6)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            PoseTCP output;
            output.x = convertFromString<double>(parts[0]);
            output.y = convertFromString<double>(parts[1]);
            output.z = convertFromString<double>(parts[2]);
            output.Roll = convertFromString<double>(parts[3]);
            output.Pitch = convertFromString<double>(parts[4]);
            output.Yaw = convertFromString<double>(parts[5]);
            return output;
        }
    }
};


class GrabPiece : public BT::StatefulActionNode
{
  public:
    // Any TreeNode with ports must have a constructor with this signature
    GrabPiece(const std::string& name, const BT::NodeConfig& config)
      : StatefulActionNode(name, config)
    {}

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<int>("object_measure") };
    }

    // this function is invoked once at the beginning.
    BT::NodeStatus onStart() override;

    // If onStart() returned RUNNING, we will keep calling
    // this method until it return something different from RUNNING
    BT::NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

  private:
    int object_measure;
    chr::system_clock::time_point _completion_time;
};

//-------------------------

BT::NodeStatus GrabPiece::onStart()
{
  if ( !getInput<int>("object_measure", object_measure))
  {
    throw BT::RuntimeError("missing required input [object_measure]");
  }
  printf("[ Grab a piece: SEND REQUEST ]. size: %d\n",
         object_measure);

  // We use this counter to simulate an action that takes a certain
  // amount of time to be completed (200 ms)
  _completion_time = chr::system_clock::now() + chr::milliseconds(3000);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GrabPiece::onRunning()
{
  // Pretend that we are checking if the reply has been received
  // you don't want to block inside this function too much time.
  std::this_thread::sleep_for(chr::milliseconds(10));

  // Pretend that, after a certain amount of time,
  // we have completed the operation
  if(chr::system_clock::now() >= _completion_time)
  {
    std::cout << "[ Grabbed piece]" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void GrabPiece::onHalted()
{
  printf("[ GrabPiece: ABORTED ]");
}




// class GrabPiece : public BT::SyncActionNode
// //This action should at least be async... for the demonstration, we used a sync one for speed
// {
// public:
//   GrabPiece(const std::string& name, const BT::NodeConfiguration& config)
//   : BT::SyncActionNode(name, config) {}

//   BT::NodeStatus tick() override {
//     int value = 0;
//     if( getInput("object_measure", value ) ){
//       ros::Duration take_break(2.0);
//       take_break.sleep();  
//       std::cout << "Grabbed the " << value << "cm wide object." <<std::endl;
//       return NodeStatus::SUCCESS;
//     }
//     else{
//       std::cout << "PrintValue FAILED "<< std::endl;
//       return NodeStatus::FAILURE;
//     }
//   }

//   static BT::PortsList providedPorts() {
//     return{ BT::InputPort<int>("object_measure") };
//   }
// };

class Move_Robot_Server: public RosActionNode<behaviortree_ros::move_robotAction>
{
private :
PoseTCP position;
public:
  Move_Robot_Server( ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
RosActionNode<behaviortree_ros::move_robotAction>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
      InputPort<PoseTCP>("TCP_Position")
  };
  }
  bool sendGoal(GoalType& goal) override
  {
    getInput<PoseTCP>("TCP_Position", position);
    goal.x = position.x;
    goal.y = position.y;
    goal.z = position.z;
    goal.Roll = position.Roll;
    goal.Pitch = position.Pitch;
    goal.Yaw = position.Yaw;
    ROS_INFO("move_robot sending request");
    return true;
  }

  NodeStatus onResult( const ResultType& res) override
  {     
      ROS_INFO("Actual state of the robot : x : %f , y : %f, z: %f, Roll : %f, Pitch : %f, Yaw: %f", res.x,res.y,res.z,res.Roll,res.Pitch,res.Yaw);
      return NodeStatus::SUCCESS;
   
  }

  virtual NodeStatus onFailedRequest(FailureCause failure) override
  {
    ROS_ERROR("Move_Robot request failed %d", static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  void halt() override
  {
    if( status() == NodeStatus::RUNNING )
    {
      ROS_WARN("Move_robot halted");
      BaseClass::halt();
    }
  }
};


int main(int argc, char **argv)
{
  
  //system("rosrun behaviortree_ros test_server");
  
  ros::init(argc, argv, "test_behavior_tree");
  ros::NodeHandle nh;

  BehaviorTreeFactory factory;

  factory.registerNodeType<GrabPiece>("GrabPiece");
  RegisterRosAction<Move_Robot_Server>(factory, "move_robot", nh);
  
  std::string xml_models = BT::writeTreeNodesModelXML(factory);
  
  factory.registerBehaviorTreeFromFile("/home/berg0401/catkin_ws/src/BehaviorTree.ROS-master/move_robot.xml");
  
  auto tree = factory.createTree("move_pick");
  
  BT::Groot2Publisher publisher(tree);
while(true){
  NodeStatus status = NodeStatus::IDLE;

  ros::Duration sleep_time1(2);
  sleep_time1.sleep();

  while( ros::ok() && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING))
  {
    ros::spinOnce();
    status = tree.tickExactlyOnce();
    std::cout << status << std::endl;
    ros::Duration sleep_time(0.01);
    sleep_time.sleep();
  }
  
}
  return 0;
}


