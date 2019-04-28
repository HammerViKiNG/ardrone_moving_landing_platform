#ifndef ARDRONE_POSE_HANDLER_H
#define ARDRONE_POSE_HANDLER_H


#include "ros/ros.h"
#include "pose_rpy/pose_rpy.h"
#include "ardrone_autonomy/Navdata.h"


class ArdronePoseHandler
{
    public:
        ArdronePoseHandler(std::string navdata_topic);
        PoseRPY get_pose_rpy(void);
        int8_t get_state(void);

        PoseRPY local_to_global(const PoseRPY& pose);
        PoseRPY global_to_local(const PoseRPY& pose);

	static PoseRPY local_to_global_shifted(PoseRPY pose, const double& rot_z);
	static PoseRPY global_to_local_shifted(PoseRPY pose, const double& rot_z);

        static std::pair<double, double> global_to_local(double x, double y, double rot_z);
    
    private:
        void navdata_callback(const ardrone_autonomy::Navdata& msg);
        void odometry(double vx, double vy, double dt);

        ros::NodeHandle nh;
        ros::Subscriber sub_navdata;

        double last_time;
        PoseRPY pose_rpy;
        int8_t state;
};


#endif
