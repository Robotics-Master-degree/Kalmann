#include "tracker_node.h"


TrackerNode::TrackerNode():
  nh_(ros::this_node::getName())
  //img_tp_(nh_)
{
  //loop rate [hz], Could be set from a yaml file
  new_detection = false;
  //last_prediction_ts
  rate_=10;
  At_ =0.1;
  x_0_ << 0.1,0.1,0.1,0.1;
  C_0_x_ << 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 100, 0,
           0, 0, 0, 100;
  F_t_ << 1,0,0.1,0,
          0,1,0,0.1,
          0,0,1,0,
          0,0,0,1;
  u_t_ << 1,1,1,1;
  G_t_ << 0,0,0,0,
          0,0,0,0,
          0,0,0,0,
          0,0,0,0;
  C_n_x_t_ << 100,0,0,0,
            0,100,0,0,
            0,0,400,0,
            0,0,0,400;
  H_t_<< 1.0,0,0,0,
        0,1.0,0,0;
  C_n_z_t_ << 1,0,
              0,1;
  x_t_=x_0_;
  //z_t=;
  C_x_t_ = C_0_x_;
  new_detection = true;

  //std::cout << "new_detection" << new_detection;



  //std::cout << x_0_ << '\n';
  /*x_0_.x=0;
  x_0_.y=0;
  x_0_.z=0;
  x_0_.w=0;*/
  raw_data_= nh_.subscribe("/ros_img_processor/raw_data", 1, &TrackerNode::StateSpaceCallback, this);
  Kalmann_Filter_ =nh_.advertise<geometry_msgs::Vector3>("Kalmann_Filter", 1);
  // Kalmann_Filter_ = nh_.subscribe("/tracker_node/Kalmann_Filter", 100, &RosImgProcessorNode::KalmanFilterCallback, this);

}

TrackerNode::~TrackerNode()
{
    //
}


/*void TrackerNode::StateSpaceCallback(const geometry_msgs::Vector3Stamped& _msg)
{
//const geometry_msgs::Vector3& _msg
      //prediction(_msg.time);

      //correction(_msg);

      //new_detection=true;

      //x_t_ << _msg.x, _msg.y,0,0;
      //radius = _msg.z;
  //prediction(const ros::Time &__ts);
  //correction();


}*/

void TrackerNode::StateSpaceCallback(const geometry_msgs::Vector3Stamped& _msg){
      new_detection = true;
      //last_prediction_ts =_msg.header.stamp;
      prediction(_msg.header.stamp);
      correction(_msg.vector);



}

void TrackerNode::prediction(const ros::Time &__ts){

  /*  un mètode que sigui prediction(const time & __ts), i que és capaç de
  fer una predicció des de last_prediction_ts fins al __ts que li entra
  com argument. A més, aquest mètode actualitza el last_prediction_ts amb
  __ts. Aquí */
    if (last_prediction_ts.toSec() == 0){
      std::cout << "First prediction" << std::endl;
    }
    else{
      At_ = (__ts - last_prediction_ts).toSec();///1000000000;
    }


    //std::cout << "prediction" << std::endl;
    //std::cout << "__ts: "  << __ts.toSec() <<std::endl;
    //std::cout << "last_prediction_ts_ " << last_prediction_ts.toSec() << std::endl;
    //std::cout << "At_: "<< At_ << std::endl;
    F_t_ << 1,0,At_,0,
            0,1,0,At_,
            0,0,1,0,
            0,0,0,1;
    //std::cout << "x_t_" << x_t_ << std::endl;
    x_t_ = F_t_*x_t_ + G_t_*u_t_;
    C_x_t_ = F_t_ * C_x_t_ * F_t_.transpose() + C_n_x_t_; //ojo amb la C_x_t_

    last_prediction_ts = __ts;
    std::cout << "F_t_" << F_t_ << std::endl;
    std::cout << "x_t_: " << x_t_ << std::endl;
    //std::cout << "At_" << At_ << std::endl;
    //std::cout << "lpt"<< last_prediction_ts  << std::endl;
}

void TrackerNode::publish(){
  geometry_msgs::Vector3 vector;
  vector.x = x_t_[0];
  vector.y = x_t_[1];
  //vector.z = radius_;
  Kalmann_Filter_.publish(vector);
  //std::cout << "vector" << vector.x << "" << vector.y;
}

void TrackerNode::correction(const geometry_msgs::Vector3& __detection){
//un mètode de correction(const detectionType & __detection) que li entra una detecció.

  std::cout << "correction" << std::endl;
  z_t << __detection.x, __detection.y;

  z_t_= H_t_*x_t_;
  Eigen::Matrix <float,2,2> part;
  part= (H_t_*C_x_t_*H_t_.transpose()) + C_n_z_t_;
  Eigen::Matrix <float,4,2> part2;
  part2=H_t_.transpose()*part.inverse();
  Eigen::Matrix <float,4,2> part3;
  K_t_=C_x_t_*part2;
  //x_t = x_t_ + K_t_ * (z_t - z_t_);
  x_t_ = x_t_ + K_t_ * (z_t - z_t_);
  Eigen::Matrix <float,4,4> I_4;
  I_4 <<  1,0,0,0,
          0,1,0,0,
          0,0,1,0,
          0,0,0,1;
  C_x_t = (I_4-K_t_*H_t_)*C_x_t_*(I_4-K_t_*H_t_) + K_t_*C_n_z_t_*K_t_.transpose();

  //std::cout << "x_t_2" << x_t_ << std::endl;
  //std::cout << "z_t_" << z_t_ << std::endl;

  this->new_detection = false;
  TrackerNode::publish();

}

double TrackerNode::getRate() const
{
    return rate_;
}
