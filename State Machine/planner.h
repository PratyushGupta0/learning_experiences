#include <ros/ros.h>
#include <future>
#include <chrono>
#include <string>
#include <thread>
#include <math.h>
#include <iostream>

//for TakeofF(Hovering example)
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/String.h>

//For wavepoint publishing(position_publisher.cpp)
#include<simulator/center_depth.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include<std_msgs/Header.h>
#include<geometry_msgs/Pose.h>

//For defining state machine
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/back/mpl_graph_fsm_check.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/euml/common.hpp>
#include <boost/msm/front/euml/operator.hpp>

#define echo(X) std::cout<<X<<std::endl;

//Gotop1(position_publisher.cpp)
const float img_center_x=400;
const float img_center_y=400;
float x_rel,y_rel,z_rel;
float x_lab,y_lab,z_lab;
float x_drone,y_drone,z_drone;
float x_lab_initial,y_lab_initial,z_lab_initial;
float x_frame=8,y_frame=-2;
std_msgs::Header header;
void DronePositionCallback(geometry_msgs::PointStamped msg){
            header=msg.header;
            x_drone=msg.point.x;
            y_drone=msg.point.y;
            z_drone=msg.point.z;
}
void center_depthCallback(simulator::center_depth msg){
            float center_x=msg.x;
            float center_y=msg.y;
            float depth=msg.depth;

            float x_angle=((center_x-img_center_x)/(2*img_center_x))*1.39;
            float y_angle=((center_y-img_center_y)/(2*img_center_y))*1.39;
            float Z=depth/(sqrt(1+tan(x_angle)*tan(x_angle)+tan(y_angle)*tan(y_angle)));
            x_rel=Z*tan(x_angle);
            y_rel=Z*tan(y_angle);
            z_rel=Z;

            x_lab=x_frame;
            y_lab=y_frame;
            z_lab=z_drone-y_rel;
}


namespace state_machine{

    namespace msm=boost::msm;
    namespace mpl=boost::mpl;

    //event declaration
    struct CmdTakeoff {CmdTakeoff(){}};
    struct CmdGotoP1 {CmdGotoP1(){}};
    struct CmdGotoP2 {CmdGotoP2(){}};
    // struct CmdDetection {CmdDetection(){}};

    //defining state_machine

    struct fsm:public msm::front::state_machine_def<fsm>{

        typedef msm::active_state_switch_before_transition active_state_switch_policy;
        template<class Event,class FSM> void on_entry(Event const&,FSM &) {echo("Entered state_machine");}
        template<class Event,class FSM> void on_exit(Event const&,FSM &) {echo("Exited state_machine");}

        struct Rest:public msm::front::state<>{
            template<class Event,class FSM> void on_entry(Event const&,FSM &) {echo("Entered Rest state");}
            template<class Event,class FSM> void on_exit(Event const&,FSM &) {echo("Exited Rest state");}
        };

        struct Hover:public msm::front::state<>{
            template<class Event,class FSM> void on_entry(Event const&,FSM &) {echo("Entered Hover state");}
            template<class Event,class FSM> void on_exit(Event const&,FSM &) {echo("Exited Hover state");}
        };

        struct ReachP1:public msm::front::state<>{
            template<class Event,class FSM> void on_entry(Event const&,FSM &) {echo("Entered ReachP1 state");}
            template<class Event,class FSM> void on_exit(Event const&,FSM &) {echo("Exited ReachP1 state");}
        };

        struct ReachP2:public msm::front::state<>{
            template<class Event,class FSM> void on_entry(Event const&,FSM &) {echo("Entered ReachP2 state");}
            template<class Event,class FSM> void on_exit(Event const&,FSM &) {echo("Exited ReachP2 state");}
        };

        // struct CenterDetect:public msm::front::state<>{
        //     template<class Event,class FSM> void on_entry(Event const&,FSM &) {echo("Entered CenterDetect state");}
        //     template<class Event,class FSM> void on_exit(Event const&,FSM &) {echo("Exited CenterDetect state");}
        // };

        typedef Rest initial_state;

        //Takeoff Function(Mav hovering)
        void Takeoff(CmdTakeoff const &cmd){
            ros::NodeHandle nh;
            // Create a private node handle for accessing node parameters.
            ros::NodeHandle nh_private("~");
            ros::Publisher trajectory_pub =
                nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
                    mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
            ROS_INFO("Started hovering example.");

            std_srvs::Empty srv;
            bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
            unsigned int i = 0;

            // Trying to unpause Gazebo for 10 seconds.
            while (i <= 10 && !unpaused) {
                ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
                std::this_thread::sleep_for(std::chrono::seconds(1));
                unpaused = ros::service::call("/gazebo/unpause_physics", srv);
                ++i;
            }

            if (!unpaused) {
                ROS_FATAL("Could not wake up Gazebo.");
                return ;
            } else {
                ROS_INFO("Unpaused the Gazebo simulation.");
            }

            // Wait for 5 seconds to let the Gazebo GUI show up.
            ros::Duration(5.0).sleep();

            trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
            trajectory_msg.header.stamp = ros::Time::now();

            // Default desired position and yaw.
            Eigen::Vector3d desired_position(0.0, 0.0, 1.5);
            double desired_yaw = 0.0;

            // Overwrite defaults if set as node parameters.
            nh_private.param("x", desired_position.x(), desired_position.x());
            nh_private.param("y", desired_position.y(), desired_position.y());
            nh_private.param("z", desired_position.z(), desired_position.z());
            nh_private.param("yaw", desired_yaw, desired_yaw);

            mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
                desired_position, desired_yaw, &trajectory_msg);

            ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
                    nh.getNamespace().c_str(), desired_position.x(),
                    desired_position.y(), desired_position.z());
            trajectory_pub.publish(trajectory_msg);

            return;
        }

        //Go to P1(position_publisher.cpp)
        void GotoP1(CmdGotoP1 const &cmd){

                ros::NodeHandle nh;
                ros::Rate loopRate(30);

                ros::Subscriber center_depth_sub=nh.subscribe<simulator::center_depth>("/center_depth",10,center_depthCallback);
                ros::Subscriber drone_pos_sub=nh.subscribe<geometry_msgs::PointStamped>("/iris/ground_truth/position",10,DronePositionCallback);

                ros::Publisher wavepoint_pub=nh.advertise<geometry_msgs::PoseStamped>("/iris/command/pose",10);
                geometry_msgs::PoseStamped coordinates;

                int i=0;
                float z_avg_lab=0,x_avg_lab=0,y_avg_lab=0;
                while(ros::ok()&&i<30){
                    ros::spinOnce();
                    x_avg_lab+=x_lab;
                    y_avg_lab+=y_lab;
                    z_avg_lab+=z_lab;
                    i++;
                    loopRate.sleep();
                }
                int frame_rate=30;
                x_avg_lab=x_avg_lab/frame_rate;
                y_avg_lab=y_avg_lab/frame_rate;
                z_avg_lab=z_avg_lab/frame_rate;

                x_lab_initial=x_avg_lab;
                y_lab_initial=y_avg_lab;
                z_lab_initial=z_avg_lab+0.5;
                std::cout<<"x_lab_initial:"<<x_lab_initial<<std::endl; 
                std::cout<<"y_lab_initial:"<<y_lab_initial<<std::endl;
                std::cout<<"z_lab_initial:"<<z_lab_initial<<std::endl; 
                

                i=0;

                while(ros::ok()&&i<210){
                    if(i==0)   ROS_INFO_STREAM("Entered for vertical movement");
                    i++;
                
                    ros::spinOnce();
                    coordinates.header=header;
                    coordinates.pose.position.x=x_drone;
                    coordinates.pose.position.y=y_drone;
                    coordinates.pose.position.z=z_lab_initial;
                    coordinates.pose.orientation.x=0;
                    coordinates.pose.orientation.y=0;
                    coordinates.pose.orientation.z=0;
                    coordinates.pose.orientation.w=1;
                    wavepoint_pub.publish(coordinates);
                    std::cout<<coordinates.pose.position<<std::endl;
                    loopRate.sleep();
                }

                ROS_INFO_STREAM("Exited the vertical motion");

                float x_increment,y_increment;
                if(x_lab_initial>x_drone) x_increment=0.2;
                else x_increment=-0.2;

                if(y_lab_initial>y_drone) y_increment=0.2;
                else y_increment=-0.2; 
                
                i=0;
                int flag_x=1,flag_y=1;
                //float dis=sqrt((x_drone-x_lab_initial)*(x_drone-x_lab_initial)+(y_drone-y_lab_initial)*(y_drone-y_lab_initial)+(z_drone-z_lab_initial)*(z_drone-z_lab_initial));
                while(ros::ok() && (flag_x||flag_y)){
                    ros::spinOnce();
                    if(i==0) {
                        ROS_INFO_STREAM("Started custom wavepoint publisher");
                        i++;
                        }
                    std::cout<<"x_rel:"<<abs(x_drone-x_lab_initial)<<std::endl;
                    std::cout<<"y_rel:"<<abs(y_drone-y_lab_initial)<<std::endl;
                    std::cout<<"flag_x:"<<flag_x<<std::endl;
                    std::cout<<"flag_y:"<<flag_y<<std::endl;
                    int result=(ros::ok() && (flag_x||flag_y));
                    std::cout<<"ExitState:"<<result<<std::endl;


                    if(flag_x||flag_y) {
                        if(abs(x_drone-x_lab_initial)>1){
                            coordinates.pose.position.x=x_drone+x_increment;
                        }
                        else{
                            coordinates.pose.position.x=x_drone;
                            flag_x=0;
                        }

                        if(abs(y_drone-y_lab_initial)>1){
                            coordinates.pose.position.y=y_drone+y_increment;
                        }
                        else{
                            coordinates.pose.position.y=y_drone;
                            flag_y=0;
                        }
                    }
                    coordinates.header=header;
                    coordinates.pose.position.z=z_lab_initial;
                    // coordinates.pose.orientation.x=0;
                    // coordinates.pose.orientation.y=0;
                    // coordinates.pose.orientation.z=0;
                    // coordinates.pose.orientation.w=1;

                    std::cout<<coordinates.pose.position<<std::endl;
                    
                    wavepoint_pub.publish(coordinates);
                
                    loopRate.sleep();
                }
                return;
        }

        void GotoP2(CmdGotoP2 const &cmd){
                ros::NodeHandle nh;
                ros::Rate loopRate(30);

                ros::Subscriber center_depth_sub=nh.subscribe<simulator::center_depth>("/center_depth",10,center_depthCallback);
                ros::Subscriber drone_pos_sub=nh.subscribe<geometry_msgs::PointStamped>("/iris/ground_truth/position",10,DronePositionCallback);
                ros::Publisher wavepoint_pub=nh.advertise<geometry_msgs::PoseStamped>("/iris/command/pose",10);
                geometry_msgs::PoseStamped coordinates;

                float tan=abs(y_lab_initial-y_drone)/abs(x_lab_initial-x_drone);
                
                float x_increment=0.2;
                float y_increment=tan*x_increment;

                float x_drone_initial=x_drone;
                float y_drone_initial=y_drone;
                int flag_x=1,flag_y=1;

                while(ros::ok()&&(flag_x||flag_y)){

                    ros::spinOnce();

                    if(y_drone_initial>y_lab_initial) {
                        coordinates.pose.position.y=y_drone-y_increment;
                        if((y_lab_initial-y_drone)>1) flag_y=0;
                    }
                    else {
                        coordinates.pose.position.y=y_drone+y_increment;
                        if((y_drone-y_lab_initial)>1) flag_y=0;
                    }

                    if(x_drone_initial>x_lab_initial) {
                        coordinates.pose.position.x=x_drone-x_increment;
                        if((x_lab_initial-x_drone)>1) flag_x=0;
                    }
                    else {
                        coordinates.pose.position.x=x_drone+x_increment;
                        if((x_drone-x_lab_initial)) flag_x=0;
                    }

                    coordinates.pose.position.z=z_drone;
                    coordinates.header=header;

                     wavepoint_pub.publish(coordinates);

                    loopRate.sleep();

                }
                return;

        }



    struct transition_table : mpl::vector<
     //      Type        Start            Event            Next              Action				    Guard
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                a_row<    Rest          ,  CmdTakeoff   ,  Hover         ,  &fsm::Takeoff                               >,
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                a_row<    Hover         ,  CmdGotoP1    ,  ReachP1       ,  &fsm::GotoP1                                 >,
                          
        // // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                a_row<    ReachP1       ,  CmdGotoP2     ,  ReachP2      ,  &fsm::GotoP2                                 >
        // // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
        //         a_row<    ReachP2  ,       CmdDetection  ,  CenterDetect,  &fsm::Detection    ,                        >,
        // // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
        >{};

    };

    typedef msm::back::state_machine<fsm> fsm_;

    static char const *const state_names[]={"Rest",
                                            "Hover",
                                            "ReachP1",
                                            "ReachP2"
                                            // "CenterDetect"
    };

    void echo_state(fsm_ const& msg){echo("Current state -- " << state_names[msg.current_state()[0]]);}

    void statePublish(ros::NodeHandle nh, fsm_ *fsm)
    {
        ros::Publisher statePub = nh.advertise<std_msgs::String>("curr_state", 10);
        ros::Rate loopRate(10);

        std_msgs::String msg;
        while(ros::ok()){
            msg.data = state_names[fsm->current_state()[0]];
            statePub.publish(msg);
            loopRate.sleep();
        }
    }
}


