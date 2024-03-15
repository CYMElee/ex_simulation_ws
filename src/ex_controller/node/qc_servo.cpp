#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "Eigen/Dense"
#include "cmath"
#include "string"

using namespace Eigen;
using namespace std;

std_msgs::Float32MultiArray fd; // desire forc for each UAV

class MAV{
    private:
        std_msgs::Float32MultiArray T; //T[0] is net thrust T[1] is apha T[2] is beta
        Vector3f fd_e;
        ros::Publisher MAV_cmd ;

    public:
        MAV(ros::NodeHandle nh, string Topic);
        void Thrust(std_msgs::Float32MultiArray \
            fd,int i);

};

MAV::MAV(ros::NodeHandle nh, string Topic)
{
    MAV_cmd = nh.advertise<std_msgs::Float32MultiArray>(Topic,10);

}

void MAV::Thrust(std_msgs::Float32MultiArray fd,int i)
{

    fd_e(0,1) = fd.data[i];
    fd_e(1,1) = fd.data[i+1];
    fd_e(2,1) = fd.data[i+2];
    
    T.data[0] = fd_e.norm(); // net thrust(PWM 0~1) you should imply thrust curve here
    T.data[1] = atan2(-fd_e(1,1),fd_e(2,1)); //alpha
    T.data[2] = asin(fd_e(0,1)/T.data[0]);  //beta
    MAV_cmd.publish(T);
}
void thrust_cb(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    fd = *msg;
}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"qc_servo");
    ros::NodeHandle nh;

    ros::Subscriber thrust = nh.subscribe<std_msgs::Float32MultiArray>
        ("/gripper/desire_thrust_each",10,thrust_cb);

    MAV mav[4] = {MAV(nh, "/MAV1/cmd"),
                  MAV(nh, "/MAV2/cmd"),
                  MAV(nh, "/MAV3/cmd"),
                  MAV(nh, "/MAV4/cmd")};
    ros::Rate rate(100);

    while(ros::ok())
    {
        mav[0].Thrust(fd,0);
        mav[1].Thrust(fd,3);
        mav[2].Thrust(fd,6);
        mav[3].Thrust(fd,9);
        
        ros::spinOnce();
        rate.sleep();
    }




    return 0;
}