#ifndef tracker_node_H
#define tracker_node_H

//std C++
#include <iostream>

//Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


//ROS headers for image I/O
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>

class TrackerNode
{
    protected:
        //ros node handle
        ros::NodeHandle nh_;
        //initializing subscribers
        ros::Subscriber state_space_topic_;
        //Parameters needed to calculate the Kalmann Filter
        //INPUTS:
        //Eigen::Matrix<float,4,1> x_0_; //x_0
        Eigen::Matrix<float,4,1> x_0_;
        Eigen::Matrix<float,4,4> C_0_x_; //x_0
        Eigen::Matrix<float,4,4> F_t_; //x_0
        Eigen::Matrix<float,4,4> G_t_;
        Eigen::Matrix<float,2,4> H_t_;

        //OUTPUTS

        Eigen::Matrix <float,4,4> C_x_t_;

        //OTHER DEFINITIONS
        Eigen::Matrix<float,4,1> u_t_;
        Eigen::Matrix <float,4,4> C_n_x_t_;
        Eigen::Matrix <float,2,2> C_n_z_t_;

        float At_;


        //PREDICTION //OUTPUT:

        //Eigen::Matrix<float,4,1> g_t_;



        //CORRECTION

        Eigen::Matrix<float,2,1> z_t_;
        Eigen::Matrix<float,4,2> K_t_;
        Eigen::Matrix<float,4,1> x_t;
        Eigen::Matrix<float,2,1> z_t;
        Eigen::Matrix <float,4,4> C_x_t;


        //Eigen::Vector2d meas_space_; //measurement_space
        //Eigen::Matrix<int,5,5> F_;
        //Eigen::Matrix<int,3,4> H_;
        //Eigen::Matrix<int,1,1> G_;
        //Eigen::Matrix<int,2,2> Cn_z_;
        //Eigen::Matrix<int,5,5> Cn_x_;




      public:
        //wished process rate, [hz]
        double rate_;
        //Constructor
        TrackerNode();

        //Desctructor
        ~TrackerNode();

        // Returns rate_
        double getRate() const;

        void prediction();
        void correction();
        Eigen::Matrix<float,4,1> x_t_;
        float radius;
        ros::Publisher Kalmann_Filter_;

   protected:
        // callbacks
        void StateSpaceCallback(const geometry_msgs::Vector3& _msg);


};
#endif
