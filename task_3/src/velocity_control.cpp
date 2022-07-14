#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <keyboard/Key.h>
#include <math.h>
#include <angles/angles.h>


ros::Publisher vel_sp_pub;


geometry_msgs::PoseStamped currentPos,nextPos,initCirclePos;
geometry_msgs::TwistStamped vs;
int angle,angleStep;
bool isFlyCircle,hasInitFlyCircle;
float radius;
float speed;

void flyCircleWithRadius(double r,double speed)
{
  if(!hasInitFlyCircle){
    initCirclePos = currentPos;
    nextPos = initCirclePos;
    angle = angle + angleStep;
    nextPos.pose.position.x = r * cos(angles::from_degrees(angle)) - r + 
                              initCirclePos.pose.position.x;
    nextPos.pose.position.y = r * sin(angles::from_degrees(angle)) + initCirclePos.pose.position.y;
    
    geometry_msgs::Point velocityVector;
    velocityVector.x = nextPos.pose.position.x - currentPos.pose.position.x;
    velocityVector.y = nextPos.pose.position.y - currentPos.pose.position.y;

    float length = sqrt(velocityVector.x*velocityVector.x + velocityVector.y*velocityVector.y);
    velocityVector.x = velocityVector.x/length * speed;
    velocityVector.y = velocityVector.y/length * speed;

    vs.twist.linear.x = velocityVector.x;
    vs.twist.linear.y = velocityVector.y;
    vs.header.stamp = ros::Time::now();
    vel_sp_pub.publish(vs);
    ROS_INFO_STREAM("next angle:" << angle);
    hasInitFlyCircle = true;
    return;
  } 
  
  // judge whether is reached
  bool isReached = false;
  double distance = sqrt((currentPos.pose.position.x - nextPos.pose.position.x)*(currentPos.pose.position.x - nextPos.pose.position.x)  + 
                       (currentPos.pose.position.y - nextPos.pose.position.y)*(currentPos.pose.position.y - nextPos.pose.position.y));
  double threshold = 0.2;
  if (distance < threshold)
  {
    isReached = true;
  }

  if (isReached)
  {
    // send next pos
    angle = angle + angleStep;
    if(angle > 360) angle = angle - 360;
    nextPos = initCirclePos;
    nextPos.pose.position.x = r * cos(angles::from_degrees(angle)) - r + 
                              initCirclePos.pose.position.x;
    nextPos.pose.position.y = r * sin(angles::from_degrees(angle)) + initCirclePos.pose.position.y;
    ROS_INFO_STREAM("next angle:" << angle);

  }

    geometry_msgs::Point velocityVector;
    velocityVector.x = nextPos.pose.position.x - currentPos.pose.position.x;
    velocityVector.y = nextPos.pose.position.y - currentPos.pose.position.y;

    float length = sqrt(velocityVector.x*velocityVector.x + velocityVector.y*velocityVector.y);
    velocityVector.x = velocityVector.x/length * speed;
    velocityVector.y = velocityVector.y/length * speed;

    vs.twist.linear.x = velocityVector.x;
    vs.twist.linear.y = velocityVector.y;
    vs.header.stamp = ros::Time::now();
    vel_sp_pub.publish(vs);
 

}

void sendCommand(const keyboard::Key &key)
{
   switch (key.code)
   {
   		case 'w':
   		{
   			ROS_INFO_STREAM("up");
   			vs.twist.linear.z += 0.1;
   			break;
   		}
      case 's':
      {
        ROS_INFO_STREAM("down");
        vs.twist.linear.z -= 0.1;
        break;
      }
   		case 'j':
   		{
   			ROS_INFO_STREAM("left");
   			vs.twist.linear.y += 0.1;
   			break;
   		}
   		case 'l':
   		{
   			ROS_INFO_STREAM("right");
   			vs.twist.linear.y -= 0.1;
   			break;
   		}
      case 'i':
      {
        ROS_INFO_STREAM("forward");
        vs.twist.linear.x += 0.1;
        break;
      }
      case 'k':
      {
        ROS_INFO_STREAM("backward");
        vs.twist.linear.x -= 0.1;
        break;
      }
      case 'u':
      {
        ROS_INFO_STREAM("rotate left");
        vs.twist.angular.z += 0.1;
        break;
      }
      case 'o':
      {
        ROS_INFO_STREAM("rotate right");
        vs.twist.angular.z -= 0.1;
        break;
      }
      case 'y':
      {
        // fly circle
        isFlyCircle = true;
        ROS_INFO_STREAM("Fly Circle Mode");
        break;
      }
      case 'h':
      {
        // turn to manual mode
        isFlyCircle = false;
        hasInitFlyCircle = false;
        vs.twist.linear.x = 0;
        vs.twist.linear.y = 0;
        vs.twist.linear.z = 0;
        vs.twist.angular.z = 0;
        ROS_INFO_STREAM("Manual Mode");
        break;
      }
      case 't':
      {
        // increase radius
        radius += 0.1;
        ROS_INFO_STREAM("increase radius" << radius);
        break;
      }
      case 'g':
      {
        // increase radius
        radius -= 0.1;
        if (radius < 1)
        {
          radius = 1;
        }
        ROS_INFO_STREAM("decrease radius" << radius);
        break;
      }
      case 'b':
      {
        angleStep++;
        ROS_INFO_STREAM("angle step:" << angleStep);
        break;
      }
      case 'n':
      {
        angleStep--;
        if (angleStep < 1)
        {
          angleStep = 1;
        }
        ROS_INFO_STREAM("angle step:" << angleStep);
        break;
      }
      case 'a':
      {
        speed += 0.1;
        ROS_INFO_STREAM("speed up" << speed);
        break;
      }
      case 'd':
      {
        speed -= 0.1;
        ROS_INFO_STREAM("speed down" << speed);
        break;
      }

   		
   		default:
   		{
   			
   		}

   }
}

void localPositionReceived(const geometry_msgs::PoseStampedConstPtr& msg){
    currentPos = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "px4_offboard_velocity_control_node");
  ros::NodeHandle nodeHandle;
  vel_sp_pub = nodeHandle.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
  ros::Subscriber commandSubscriber = nodeHandle.subscribe("/keyboard/keydown",1,sendCommand);
  ros::Subscriber localPositionSubsciber = nodeHandle.subscribe("/mavros/local_position/local", 10, localPositionReceived);
  
  // fly circle parameters
  isFlyCircle = false;
  hasInitFlyCircle = false;
  angle = 0;
  radius = 1;
  angleStep = 5;
  speed = 0.2;

  ros::Rate loop_rate(10.0);

  while(ros::ok())
  {

      vs.header.seq++;

      if(!isFlyCircle){
        vs.header.stamp = ros::Time::now();
        //ROS_INFO_STREAM("send ps" << ps);
        vel_sp_pub.publish(vs);
      } else {
        flyCircleWithRadius(radius,speed);
        //flyHeartWithRadius(radius);
        //flyPeachHeartWithRadius(radius);
      }
    

    ros::spinOnce();

    loop_rate.sleep();
  }


}

