#include <ros/ros.h>
#include <behaviortree_ros/move_robotAction.h>
#include <actionlib/server/simple_action_server.h>

class Move_Robot_Server
{
protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  actionlib::SimpleActionServer<behaviortree_ros::move_robotAction> server_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  behaviortree_ros::move_robotFeedback feedback_;
  behaviortree_ros::move_robotResult result_;

  int call_number_;
  



public:

  float x_;
  float y_;
  float z_;
  float Roll_;
  float Pitch_;
  float Yaw_;
  int range_of_motion_;
  Move_Robot_Server(std::string name) :
  server_(nh_, name, boost::bind(&Move_Robot_Server::executeCB, this, _1), false),
  action_name_(name)
  {
    server_.start();
    call_number_ = 0;
    x_=0;
    y_=0;
    z_=0;
    Roll_=0;
    Pitch_=0;
    Yaw_=0;
    range_of_motion_ = 1;
  }

  ~Move_Robot_Server(void)
  {
  }

  void reset_task(){
    x_=0;
    y_=0;
    z_=0;
    Roll_=0;
    Pitch_=0;
    Yaw_=0;
    range_of_motion_ = 1;
  }
  void executeCB(const behaviortree_ros::move_robotGoalConstPtr &goal)
  {
    bool preempted = false;
    

    while(x_!= goal->x || y_!= goal->y || z_!= goal->z || Roll_!=goal->Roll || Pitch_!= goal->Pitch || Yaw_!=goal->Yaw)
    {
        if (server_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            server_.setPreempted();
            preempted = true;
            break;
        }
        
        if(range_of_motion_==1)
        {
            x_ = goal->x;
        }
        if(range_of_motion_==2)
        {
            y_ = goal->y;
        }
        if(range_of_motion_==3)
        {
            z_ = goal->z;
        }
        if(range_of_motion_==4)
        {
            Roll_ = goal->Roll;
        }
        if(range_of_motion_==5)
        {
            Pitch_ = goal->Pitch;
        }
        if(range_of_motion_==6)
        {
            Yaw_ = goal->Yaw;
        }
        feedback_.percentage = range_of_motion_*16;
        ros::Duration take_break(0.50);
        take_break.sleep();
        server_.publishFeedback(feedback_);
        range_of_motion_+=1;
    }

    if(!preempted)
    {
      result_.x = x_;
      result_.y = y_;
      result_.z = z_;
      result_.Roll = Roll_;
      result_.Pitch = Pitch_;
      result_.Yaw = Yaw_;
      reset_task(); //beacause it's always the same goal that is sent for this example.
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      server_.setSucceeded(result_);
    }
    else{
      ROS_WARN("%s: Preempted", action_name_.c_str());
    }
  }
};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_robot_server");
  ros::NodeHandle n;
  Move_Robot_Server move_robot("move_robot");
  ros::spin();
  return 0;
}



