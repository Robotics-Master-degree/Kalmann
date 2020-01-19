#include "tracker_node.h"


TrackerNode::TrackerNode():
  nh_(ros::this_node::getName())
  //img_tp_(nh_)
{
  //loop rate [hz], Could be set from a yaml file
  rate_=10;
  At_ =1;
  x_0_ << 10.0,10.0,0.0,0.0;
  C_0_x_ << 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 100, 0,
           0, 0, 0, 100;
  F_t_ << 1,0,At_,0,
          0,1,0,At_,
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



  //std::cout << x_0_ << '\n';
  /*x_0_.x=0;
  x_0_.y=0;
  x_0_.z=0;
  x_0_.w=0;*/
  state_space_topic_ = nh_.subscribe("/ros_img_processor/state_space", 1, &TrackerNode::StateSpaceCallback, this);
  Kalmann_Filter_ =nh_.advertise<geometry_msgs::Vector3>("Kalmann_Filter", 1);

}

TrackerNode::~TrackerNode()
{
    //
}


void TrackerNode::StateSpaceCallback(const geometry_msgs::Vector3& _msg)
{

  x_t_ << _msg.x, _msg.y,0,0;
  radius = _msg.z;
  prediction();
  correction();


}

void TrackerNode::prediction(){

   //Compute them if they are not constantˆxt=Ftxt−1+Gtut//Prior state estimateCtx=FtCt−1x(Ft)T+Ctnx//Prior state covariance
  //std::cout << "F_t" << F_t_ << "Finished" << std::endl;
  //std::cout << "x_t_"<< x_t_ << std::endl;

  //Eigen::Matrix<float,4,1> hola = F_t_*x_t_;// + G_t_ * u_t_;
  x_t_ = F_t_*x_t_ + G_t_*u_t_;
  C_x_t_ = F_t_ * C_x_t_ * F_t_.transpose() + C_n_x_t_; //ojo amb la C_x_t_
  //std::cout << "C_x_t_"<< C_x_t_ << std::endl;
  //std::cout << "F_t_"<< F_t_ << std::endl;
}

void TrackerNode::correction(){
//ORRECTIONHt//Compute it if it is not constantˆzt=Htˆxt//Compute the expected measurementKt=Ctx(Ht)T(HtCtx(Ht)T+Ctnz)−1//Compute the gainˆxt=ˆxt+Kt(zt−ˆzt)//Posterior state estimateCtx= (I−KtHt)Ctx(I−KtHt)T+KtCtnz(Kt)T//Update the covariance of the state estimate
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
  //cv::Point center = cv::Point(cvRound(x_t[0][0]), cvRound(x[0][1]);
  //cv::circle(cv_img_out_.image, center, radius, cv::Scalar(0,255,0), 3, 8, 0 );// circle perimeter in red

  //std::cout << "x_t_" << x_t << std::endl;


}

double TrackerNode::getRate() const
{
    return rate_;
}
