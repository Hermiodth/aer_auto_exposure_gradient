#ifndef EXP_ROS_NODE_H_
#define EXP_ROS_NODE_H_

#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>
#include <libgen.h>
#include <math.h>
#include <mutex>
#include <sys/time.h>
#include <string>
#include <sstream>
#include <cstdlib>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.hpp>

// #include <dynamic_reconfigure/BoolParameter.h>
// #include <dynamic_reconfigure/DoubleParameter.h>
// #include <dynamic_reconfigure/IntParameter.h>
// #include <dynamic_reconfigure/StrParameter.h>
// #include <dynamic_reconfigure/GroupState.h>
// #include <dynamic_reconfigure/Reconfigure.h>
// #include <dynamic_reconfigure/Config.h>

#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>

#ifdef WITH_PLOTTER
#include <plotter_ros2/plotter.hpp>
#endif

//#include <aer_auto_exposure_gradient/Dehaze.h>

#define GAMMAS_COUNT 7
#define POLYNOME_DEGREE 2

namespace exp_node {

class ExpNode : public rclcpp::Node {
 public:

  ExpNode(const rclcpp::NodeOptions & options) : rclcpp::Node("exp_node", options), callback_start_time(nullptr), it_(nullptr) {
    auto node_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    init(node_ptr);
  }
  void init(std::shared_ptr<rclcpp::Node> node_ptr);

 private:
 
  void gnulot(double * coeff_curve);
  void CameraCb(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
  void optimizerCb();
  void optimizeGradient();
  void optimizeSimple();
  void optimizeShim();
  double image_gradient_gamma(cv::Mat &src_img, int j);
  void ChangeParam (double shutter_new, double gain_new);
  
  double * curveFit (double x[7], double y[7]);
  double * curveFitLogQuadratic(double x[7], double y[7]);
  double  findRoots1 (double a[6], double check);
  void generate_LUT ();
  bool check_rate = false;
  double frame_rate_req = 10.0; // maximum 80 fps

  double gamma[GAMMAS_COUNT]={1.0/1.9, 1.0/1.5, 1.0/1.2, 1.0, 1.2, 1.5, 1.9};
  double metric[GAMMAS_COUNT];
  double max_metric;
  double max_gamma, alpha, expNew, expCur, shutter_cur, shutter_new, gain_cur;//, gain_new;
  double upper_shutter_limit, lower_shutter_limit;
  int startup_delay;
  double kp; // contorl the speed to convergence
  double d = 0.1, R; // parameters used in the nonliear function in Shim's 2018 paper 				
  int gamma_index; // index to record the location of the optimum gamma value
  bool gain_flag = false;
  std::string image_topic;
  std::string shutter_update_method;
  std::string shim_update_function;
  //std::string service_call ="camera/spinnaker_camera_nodelet/set_parameters";
  //std::string exp_param_call = "camera/spinnaker_camera_nodelet/exposure_time";
  //std::string gain_param_call = "camera/spinnaker_camera_nodelet/gain";

  double grad_k;
  double gamma_x_offset_ = 0.0;
  std::string curve_fit_method_;

  // Parameters that correlated to Shim's Gradient Metric
  double met_act_thresh = 0.06;
  double lamda = 1000.0; // The lamda value used in Shim's 2014 paper as a control parameter to adjust the mapping tendency (larger->steeper)

  bool do_sweep;

  //ros::NodeHandle nh_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber sub_camera_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr shutter_speed_us_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gain_db_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gamma_est_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gradient_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gradient_clipped_pub_;

  std::shared_ptr<rclcpp::Time> callback_start_time;
  std::shared_ptr<rclcpp::Time> zeroing_duration;
  std::shared_ptr<rclcpp::Time> last_camera_process_time_;

  int test_shutter_speed;
  int true_best_shutter_speed;
  double metric_tmp;

#ifdef WITH_PLOTTER
  bool enable_plotter;
  std::shared_ptr<plotter_ros2::Plotter> plotter_gamma;
  std::shared_ptr<plotter_ros2::Plotter> plotter_sweep;
  std::vector<double> sweep_shutters_;
  std::vector<double> sweep_metrics_;
  rclcpp::TimerBase::SharedPtr sweep_republish_timer_;
#endif

  int img_proc_loop_hz_;

  // Optimizer timer state
  double coeff_[POLYNOME_DEGREE + 1];  // curve-fit coefficients shared with optimizerCb
  double shutter_at_camera_ = 0.0;     // shutter speed when last image was captured
  bool   new_camera_data_   = false;   // flag: new curve-fit data available
  int optimizer_loop_hz_;
  std::mutex optimizer_mutex_;
  rclcpp::TimerBase::SharedPtr optimizer_timer_;

  // Gradient optimizer state (optimizer thread only, no mutex needed)
  double gamma_est_ = 1.0;  // current gamma estimate, resets to 1.0 on each new image
};

}

#endif
