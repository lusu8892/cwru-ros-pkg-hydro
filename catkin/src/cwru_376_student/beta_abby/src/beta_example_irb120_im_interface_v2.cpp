// simple_marker_listener.cpp
// Wyatt Newman
// node that listens on topic "marker_listener" and prints pose received

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <interactive_markers/interactive_marker_server.h>
#include <irb120_kinematics.h>
#include <cwru_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>

//callback to subscribe to marker state
Eigen::Vector3d g_p;
Vectorq6x1 g_q_state;
double g_x,g_y,g_z;
//geometry_msgs::Quaternion g_quat; // global var for quaternion
Eigen::Quaterniond g_quat;
Eigen::Matrix3d g_R;
Eigen::Affine3d g_A_flange_desired;
bool g_trigger=false;

 
tf::TransformListener* g_tfListener;
tf::StampedTransform g_armlink1_wrt_baseLink;
geometry_msgs::PoseStamped g_marker_pose_in;
geometry_msgs::PoseStamped g_marker_pose_wrt_arm_base;

using namespace std;

void markerListenerCB(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM(feedback->marker_name << " is now at "
            << feedback->pose.position.x << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z);
    ROS_INFO_STREAM("marker frame_id is "<<feedback->header.frame_id);
    g_marker_pose_in.header = feedback->header;
    g_marker_pose_in.pose=feedback->pose;
    g_tfListener->transformPose("link1", g_marker_pose_in, g_marker_pose_wrt_arm_base);
     
    //copy to global vars:
     /*
    g_p[0] = feedback->pose.position.x;
    g_p[1] = feedback->pose.position.y;
    g_p[2] = feedback->pose.position.z;
    g_quat.x() = feedback->pose.orientation.x;
    g_quat.y() = feedback->pose.orientation.y;
    g_quat.z() = feedback->pose.orientation.z;
    g_quat.w() = feedback->pose.orientation.w;   
    g_R = g_quat.matrix(); */
     
    g_p[0] = g_marker_pose_wrt_arm_base.pose.position.x;
    g_p[1] = g_marker_pose_wrt_arm_base.pose.position.y;
    g_p[2] = g_marker_pose_wrt_arm_base.pose.position.z;
    g_quat.x() = g_marker_pose_wrt_arm_base.pose.orientation.x;
    g_quat.y() = g_marker_pose_wrt_arm_base.pose.orientation.y;
    g_quat.z() = g_marker_pose_wrt_arm_base.pose.orientation.z;
    g_quat.w() = g_marker_pose_wrt_arm_base.pose.orientation.w;   
    g_R = g_quat.matrix();      


}

void jointStateCB(
const sensor_msgs::JointStatePtr &js_msg) {
    
    for (int i=0;i<6;i++) {
        g_q_state[i] = js_msg->position[i];
    }
    //cout<<"g_q_state: "<<g_q_state.transpose()<<endl;
    
}

bool triggerService(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response)
{
    ROS_INFO("service callback activated");
    response.resp = true; // boring, but valid response info
    // grab the most recent IM data and repackage it as an Affine3 matrix to set a target hand pose;
    g_A_flange_desired.translation() = g_p;
    g_A_flange_desired.linear() = g_R;
    cout<<"g_p: "<<g_p.transpose()<<endl;
    cout<<"R: "<<endl;
    cout<<g_R<<endl;
    g_trigger=true; //inform "main" that we have a new goal!
    return true;
}

//command robot to move to "qvec" using a trajectory message, sent via ROS-I
void stuff_trajectory( Vectorq6x1 qvec, trajectory_msgs::JointTrajectory &new_trajectory) {
    
    std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points1(25);
    std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points2(25);

    new_trajectory.points.clear();  
    int jointsSum = new_trajectory.joint_names.size();
    auto trajPointsSum1 = trajectory_points1.size(); // This trajPointsSum is the sum value of the trajectory points ABBY go thru
    auto trajPointsSum2 = trajectory_points2.size();
    ROS_INFO("the number of joints: %d",jointsSum);

    new_trajectory.header.stamp = ros::Time::now();  
    
    for (auto i = 0; i < trajPointsSum1; ++i){
         trajectory_points1[i].positions.clear();
    }

    for (auto i = 0; i < trajPointsSum2; ++i){
         trajectory_points2[i].positions.clear();
    }

    Vectorq6x1 ptAboveTb; // the joints angles which make ABBY from current position to a point above from the table
    ptAboveTb << 0.0, -2*3.14/6, 2*3.14/10, 0.0, 2*3.14/10, 0.0;
    Vectorq6x1 interJointAngle1; // default-initializing a variable interJointAngle of type Vectorq6x1
    for (size_t i = 0; i < min( ptAboveTb.size(), g_q_state.size() ); ++i){
        interJointAngle1[i] = (ptAboveTb[i] - g_q_state[i]) / (24);
    }

    
    std::vector<Vectorq6x1> interJointAngleVec1(24); // declaring a vector which has trajectory_points.size() - 1 number of elements
    auto interJointAngleVecSum1 = interJointAngleVec1.size();
    interJointAngleVec1[0] = g_q_state + interJointAngle1; // defining the first element of the vector
    // defining the rest of the element of the vector
    for (auto i = 1; i < interJointAngleVecSum1; i++){
        interJointAngleVec1[i] = interJointAngleVec1[i-1] + interJointAngle1;
    }
    

    for (int ijnt = 0; ijnt < jointsSum; ijnt++){
        trajectory_points1[0].positions.push_back(g_q_state[ijnt]); // stuff in position commands for 6 joints
    }
    trajectory_points1[0].time_from_start = ros::Duration(0);
    for (auto i = 1; i < 25; i++){
        //time_from_start is relative to trajectory.header.stamp 
        //each trajectory point's time_from_start must be greater than the last
        for (int ijnt = 0; ijnt < jointsSum; ijnt++){
            trajectory_points1[i].positions.push_back(interJointAngleVec1[i-1](ijnt)); // stuff in position commands for 6 joints
        }
        //trajectory_points[i].positions.push_back(interJointAngleVec[i-1].segment(0,5));
        trajectory_points1[i].time_from_start = ros::Duration(0.05*i); 
    }
    
    //new_trajectory.points.clear();
    Vectorq6x1 interJointAngle2; // default-initializing a variable interJointAngle of type Vectorq6x1
    for (size_t i = 0; i < min( qvec.size(), ptAboveTb.size() ); ++i){
        interJointAngle2[i] = (qvec[i] - ptAboveTb[i]) / (24);
        
    }
    //ROS_INFO("itnerJointAngle2: %f",interJointAngle2);
    //auto intjsize = interJointAngle2.size();
    //ROS_INFO("interJointAngle2size: %d",intjsize);
    std::vector<Vectorq6x1> interJointAngleVec2(24); // declaring a vector which has trajectory_points.size() - 1 number of elements
    auto interJointAngleVecSum2 = interJointAngleVec2.size();
    interJointAngleVec2[0] = ptAboveTb + interJointAngle2; // defining the first element of the vector
    // defining the rest of the element of the vector
    for (auto i = 1; i < interJointAngleVecSum2; i++){
        interJointAngleVec2[i] = interJointAngleVec2[i-1] + interJointAngle2;
    }
    

    for (int ijnt = 0; ijnt < jointsSum; ijnt++){
        trajectory_points2[0].positions.push_back(ptAboveTb[ijnt]); // stuff in position commands for 6 joints
    }
    trajectory_points2[0].time_from_start = ros::Duration(1.4);
    for (auto i = 1; i < 25; i++){
        //time_from_start is relative to trajectory.header.stamp 
        //each trajectory point's time_from_start must be greater than the last
        for (int ijnt = 0; ijnt < jointsSum; ijnt++){
            trajectory_points2[i].positions.push_back(interJointAngleVec2[i-1](ijnt)); // stuff in position commands for 6 joints
        }
        //trajectory_points[i].positions.push_back(interJointAngleVec[i-1].segment(0,5));
        trajectory_points2[i].time_from_start = ros::Duration(1.4+0.05*i); 
    }
    
    // starting a point above table to the final point
    for (int i = 0; i < 25; i++){
        new_trajectory.points.push_back(trajectory_points1[i]);
    }

    for (int i = 0; i < 25; i++){
        new_trajectory.points.push_back(trajectory_points2[i]);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_marker_listener"); // this will be the node name;
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);  
    ROS_INFO("setting up subscribers ");
    ros::Subscriber sub_js = nh.subscribe("/joint_states",1,jointStateCB);
    ros::Subscriber sub_im = nh.subscribe("example_marker/feedback", 1, markerListenerCB);
    ros::ServiceServer service = nh.advertiseService("move_trigger", triggerService);   
    
    Eigen::Vector3d p;
    Eigen::Vector3d n_des,t_des,b_des;
    std::vector<Vectorq6x1> q6dof_solns;
    Vectorq6x1 qvec;
    ros::Rate sleep_timer(10.0); //10Hz update rate    
    Irb120_fwd_solver irb120_fwd_solver; //instantiate forward and IK solvers
    Irb120_IK_solver ik_solver;
    Eigen::Vector3d n_urdf_wrt_DH,t_urdf_wrt_DH,b_urdf_wrt_DH;
    // in home pose, R_urdf = I
    //DH-defined tool-flange axes point as:
    // z = 1,0,0
    // x = 0,0,-1
    // y = 0,1,0
    // but URDF frame is R = I
    // so, x_urdf_wrt_DH = z_DH = [0;0;1]
    // y_urdf_wrt_DH = y_DH = [0;1;0]
    // z_urdf_wrt_DH = -x_DH = [-1; 0; 0]
    // so, express R_urdf_wrt_DH as:
    n_urdf_wrt_DH <<0,0,1;
    t_urdf_wrt_DH <<0,1,0;
    b_urdf_wrt_DH <<-1,0,0;
    Eigen::Matrix3d R_urdf_wrt_DH;
    R_urdf_wrt_DH.col(0) = n_urdf_wrt_DH;
    R_urdf_wrt_DH.col(1) = t_urdf_wrt_DH;
    R_urdf_wrt_DH.col(2) = b_urdf_wrt_DH;    

    trajectory_msgs::JointTrajectory new_trajectory; // an empty trajectory
    new_trajectory.joint_names.push_back("joint_1");
    new_trajectory.joint_names.push_back("joint_2");
    new_trajectory.joint_names.push_back("joint_3");
    new_trajectory.joint_names.push_back("joint_4");
    new_trajectory.joint_names.push_back("joint_5");
    new_trajectory.joint_names.push_back("joint_6");
    
    //qvec<<0,0,0,0,0,0;
    Eigen::Affine3d A_flange_des_DH;
    
    //   A_fwd_DH = irb120_fwd_solver.fwd_kin_solve(qvec); //fwd_kin_solve

    //std::cout << "A rot: " << std::endl;
    //std::cout << A_fwd_DH.linear() << std::endl;
    //std::cout << "A origin: " << A_fwd_DH.translation().transpose() << std::endl;   
    
    
    g_tfListener = new tf::TransformListener;  //create a transform listener
    
    // wait to start receiving valid tf transforms between map and odom:
    bool tferr=true;
    ROS_INFO("waiting for tf between base_link and link1 of arm...");
    while (tferr) {
        tferr=false;
        try {
                //try to lookup transform from target frame "odom" to source frame "map"
            //The direction of the transform returned will be from the target_frame to the source_frame. 
             //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
                g_tfListener->lookupTransform("base_link", "link1", ros::Time(0), g_armlink1_wrt_baseLink);
            } catch(tf::TransformException &exception) {
                ROS_ERROR("%s", exception.what());
                tferr=true;
                ros::Duration(0.5).sleep(); // sleep for half a second
                ros::spinOnce();                
            }   
    }
    ROS_INFO("tf is good");
    // from now on, tfListener will keep track of transforms        
  
    
    int nsolns;
    
    while(ros::ok()) {
            ros::spinOnce();
            if (g_trigger) {
                // ooh!  excitement time!  got a new tool pose goal!
                g_trigger=false; // reset the trigger
                //is this point reachable?
                A_flange_des_DH = g_A_flange_desired;
                A_flange_des_DH.linear() = g_A_flange_desired.linear()*R_urdf_wrt_DH.transpose();
                cout<<"R des DH: "<<endl;
                cout<<A_flange_des_DH.linear()<<endl;
                nsolns = ik_solver.ik_solve(A_flange_des_DH);
                ROS_INFO("there are %d solutions",nsolns);

                if (nsolns>0) {      
                ik_solver.get_solns(q6dof_solns);  
                //qvec = q6dof_solns[0];
                std::vector<int> weight{1,2,3,3,2,1}; //defining a weight vector
                double sum;
                double minimum = 1e6;
                int ikSoluNo = 0;
                Vectorq6x1 oneIkSolu;
                for (int i = 0; i < q6dof_solns.size(); ++i){
                    oneIkSolu = q6dof_solns[i];
                    sum = 0;
                    for (int ijnt = 0; ijnt < 6; ++ijnt){
                        sum = sum + oneIkSolu[ijnt] * weight[ijnt];
                    }
                    if (sum < minimum){
                        minimum = sum;
                        ikSoluNo = i; // remember the IK solution which has the minimum last joint angle solution
                    }
                }
                qvec = q6dof_solns[ikSoluNo];

                stuff_trajectory(qvec,new_trajectory);

                pub.publish(new_trajectory);
                }
            }
            sleep_timer.sleep();    
            
    }
    
    return 0;
}


