#include "aer_auto_exposure_gradient/auto_exp.h"



namespace exp_node 
{
	cv::Mat lookUpTable_01 (1, 256, CV_8U);
	cv::Mat lookUpTable_05 (1, 256, CV_8U);
	cv::Mat lookUpTable_08 (1, 256, CV_8U);
	cv::Mat lookUpTable_1(1, 256, CV_8U);
	cv::Mat lookUpTable_12(1, 256, CV_8U);
	cv::Mat lookUpTable_15(1, 256, CV_8U);
	cv::Mat lookUpTable_19(1, 256, CV_8U);
	cv::Mat lookUpTable_metric(1, 256, CV_8U);

	//ExpNode::ExpNode () : rclcpp::Node("exp_node"), callback_start_time(nullptr)

	void ExpNode::init(std::shared_ptr<rclcpp::Node> node_ptr){
		it_ = std::make_shared<image_transport::ImageTransport>(node_ptr);

		declare_parameter<std::string>("image_topic", "camera/image_raw");
		get_parameter("image_topic", image_topic);
    	RCLCPP_INFO(get_logger(), "image topic: %s", image_topic.c_str());

		declare_parameter<int>("lower_shutter_speed_limit", 1000);
		int lower_shutter_limit_param;
		get_parameter("lower_shutter_speed_limit", lower_shutter_limit_param);
		lower_shutter_limit = (double)lower_shutter_limit_param/1000000.0;
    	RCLCPP_INFO(get_logger(), "lower shutter speed limit: %.0f", lower_shutter_limit);

		//declare_parameter<int>("upper_shutter_speed_limit", 32754);
		declare_parameter<int>("upper_shutter_speed_limit", 70000);
		int upper_shutter_limit_param;
		get_parameter("upper_shutter_speed_limit", upper_shutter_limit_param);
		upper_shutter_limit = (double)upper_shutter_limit_param/1000000.0;
    	RCLCPP_INFO(get_logger(), "upper shutter speed limit: %.0f", upper_shutter_limit);

		declare_parameter<double>("kp", 0.02);
		get_parameter("kp", kp);
    	RCLCPP_INFO(get_logger(), "kp param: %.2f", kp);

		declare_parameter<double>("grad_k", 1.0);
		get_parameter("grad_k", grad_k);
    	RCLCPP_INFO(get_logger(), "grad_k: %.2f", grad_k);

		declare_parameter<int>("initial_shutter_speed", 5000);
		int initial_shutter_speed;
		get_parameter("initial_shutter_speed", initial_shutter_speed);
		shutter_cur = (double)initial_shutter_speed/1000000.0;	// from microseconds to seconds
    	RCLCPP_INFO(get_logger(), "initial shutter speed: %i", initial_shutter_speed);

		declare_parameter<double>("initial_gain", 0.0);
		get_parameter("initial_gain", gain_cur);
    	RCLCPP_INFO(get_logger(), "initial gain: %.2f", gain_cur);

		declare_parameter<int>("startup_delay", 1);
		get_parameter("startup_delay", startup_delay);
    	RCLCPP_INFO(get_logger(), "startup delay: %i", startup_delay);

		declare_parameter<int>("img_proc_loop_hz", 1);
		get_parameter("img_proc_loop_hz", img_proc_loop_hz_);
    	RCLCPP_INFO(get_logger(), "img_proc_loop_hz: %i", img_proc_loop_hz_);

		declare_parameter<int>("optimizer_loop_hz", 10);
		get_parameter("optimizer_loop_hz", optimizer_loop_hz_);
		RCLCPP_INFO(get_logger(), "optimizer loop hz: %i", optimizer_loop_hz_);

        // std::cout <<"the  image topic given in launch file? :"<< nh.getParam("/service_call", service_call)<<"\n";
        // std::cout <<"the value of service call val is : "<< service_call<<"\n";
        // std::cout <<"the  image topic given in launch file? :"<< nh.getParam("/exp_param_call", exp_param_call)<<"\n";
        // std::cout <<"the value of exp param is : "<< exp_param_call<<"\n";
        // std::cout <<"the  image topic given in launch file? :"<< nh.getParam("/gain_param_call", gain_param_call)<<"\n";
        // std::cout <<"the value of gain param is : "<< gain_param_call<<"\n";
        
        // cv::namedWindow("view", cv2::CV_WINDOW_NORMAL); // comment in implement
		cv::namedWindow("view"); // comment in implement

    	generate_LUT();
    	sub_camera_ = it_->subscribe(image_topic, 1, &ExpNode::CameraCb, this);

		declare_parameter<std::string>("shutter_speed_apply_topic", "expose_us");
		std::string shutter_speed_topic;
		get_parameter("shutter_speed_apply_topic", shutter_speed_topic);
    	RCLCPP_INFO(get_logger(), "shutter speed apply topic: %s", shutter_speed_topic.c_str());
		shutter_speed_us_pub = this->create_publisher<std_msgs::msg::Int32>(shutter_speed_topic, 10);

		declare_parameter<std::string>("gain_apply_topic", "gain_db");
		std::string gain_topic;
		get_parameter("gain_apply_topic", gain_topic);
    	RCLCPP_INFO(get_logger(), "gain apply topic: %s", gain_topic.c_str());
		gain_db_pub = this->create_publisher<std_msgs::msg::Float32>(gain_topic, 10);

		declare_parameter<std::string>("shutter_update_method", "simple");
		get_parameter("shutter_update_method", shutter_update_method);
		RCLCPP_INFO(get_logger(), "shutter update method: %s", shutter_update_method.c_str());

		declare_parameter<std::string>("shim_update_function", "2018");
		get_parameter("shim_update_function", shim_update_function);
		RCLCPP_INFO(get_logger(), "shim update function: %s", shim_update_function.c_str());

		test_shutter_speed = lower_shutter_limit;
		metric_tmp = 0;

		// Initialize shared optimizer state
		max_gamma   = 1.0;
		gamma_index = 3;   // index of gamma == 1.0 (neutral)
		for (int i = 0; i < POLYNOME_DEGREE + 1; i++) coeff_[i] = 0.0;

		int optimizer_period_ms = static_cast<int>(1000.0 / optimizer_loop_hz_);
		optimizer_timer_ = create_wall_timer(
			std::chrono::milliseconds(optimizer_period_ms),
			std::bind(&ExpNode::optimizerCb, this)
		);

#ifdef WITH_PLOTTER
		declare_parameter<bool>("enable_plotter", true);
		get_parameter("enable_plotter", enable_plotter);
		if (enable_plotter) {
			plotter_gamma = std::make_shared<plotter_ros2::Plotter>(
				node_ptr,
				"plot_example",
				800,
				600,
				cv::Scalar(255, 255, 255)
			);
		}
#endif
	}

	void ExpNode::optimizerCb() {

		if (shutter_update_method == "gradient") {
			optimizeGradient();
		} else if (shutter_update_method == "shim") {
			optimizeShim();
		} else { // "simple"
			optimizeSimple();
		}

		if (shutter_new > upper_shutter_limit) {
			gain_flag = true;
			shutter_new = upper_shutter_limit;
		} else if (shutter_new < lower_shutter_limit) {
			gain_flag = true;
			shutter_new = lower_shutter_limit;
		} else {
			gain_flag = false;
		}

		RCLCPP_INFO(get_logger(), "shutter_new: %.0f us", shutter_new * 1000000.0);

		ChangeParam(shutter_new, 0.0);
		shutter_cur = shutter_new;
	}

	void ExpNode::optimizeGradient(){
		double local_max_gamma;
		int    local_gamma_index;
		double local_coeff[POLYNOME_DEGREE + 1];
		{
			std::lock_guard<std::mutex> lock(optimizer_mutex_);
			local_max_gamma   = max_gamma;
			local_gamma_index = gamma_index;
			for (int i = 0; i < POLYNOME_DEGREE + 1; i++) local_coeff[i] = coeff_[i];
		}

		double D = 2*local_coeff[0] + local_coeff[1];
		shutter_new = shutter_cur + 0.0001 * grad_k * D;
	}

	void ExpNode::optimizeSimple(){
		int    local_gamma_index;
		{
			std::lock_guard<std::mutex> lock(optimizer_mutex_);
			local_gamma_index = gamma_index;
		}
		shutter_new = shutter_cur + 0.5*1000.0*(local_gamma_index - 3)/1000000.0;
	}

	void ExpNode::optimizeShim(){
		double local_max_gamma;
		{
			std::lock_guard<std::mutex> lock(optimizer_mutex_);
			local_max_gamma   = max_gamma;
		}
		alpha = 1.0;

		expCur = log2(7.84 / (shutter_cur * pow(2, gain_cur/6.0)));
		if (shim_update_function == "2014") {
			expNew = (1 + kp * alpha * (1 - local_max_gamma)) * expCur;
		} else { // "2018"
			double gamma_nudge = 0.0;
			if (local_max_gamma >= 1.0) R = -pow((local_max_gamma - (1.0 - gamma_nudge)), 2) + 1;
			else                        R =  pow((local_max_gamma - (1.0 - gamma_nudge)), 2) + 1;
			expNew = (1 + alpha * kp * (R - 1)) * expCur;
		}
		shutter_new = 7.84 / pow(2, expNew);
	}
	
	void ExpNode::CameraCb (const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
		//// code for measuring the true optimal exposure time

		// if(!zeroing_duration) zeroing_duration = std::make_shared<rclcpp::Time>(now());
		// if((now() - *zeroing_duration.get()).seconds() < 1) {
		// 	ChangeParam(0, 0.0);
		// 	return;
		// }

		// if(test_shutter_speed < upper_shutter_limit*1000000.0) {
		// 	cv::Mat image1;
		// 	try {
		// 		image1 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
		// 	} catch (cv_bridge::Exception& e) {
		// 		RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
		// 	}

		// 	cv::Mat image2;
		// 	cv::Size size(342,408);
		// 	cv::resize(image1, image2, size);

		// 	double test_metric = image_gradient_gamma(image2, 3);
		// 	RCLCPP_INFO(get_logger(), "test shutter: %i, test metric: %f", test_shutter_speed, test_metric);
		// 	if(test_metric > metric_tmp) {
		// 		true_best_shutter_speed = test_shutter_speed;
		// 		metric_tmp = test_metric;
		// 	}
		// 	RCLCPP_INFO(get_logger(), "best shutter: %i, test metric: %f", true_best_shutter_speed, metric_tmp);

		// 	ChangeParam(((double)test_shutter_speed)/1000000.0, 0.0);
		// 	test_shutter_speed += 1000;

		// 	usleep(300000);

		// 	return;
		// }

		if(!callback_start_time) callback_start_time = std::make_shared<rclcpp::Time>(now());
		if((now() - *callback_start_time.get()).seconds() < startup_delay) {
			RCLCPP_INFO(get_logger(), "startup delay: %i; will wait for %f and publishing initial shutter speed of %.0f ms and gain of %.2f",
			startup_delay, (now() - *callback_start_time.get()).seconds(), shutter_cur * 1000000.0, gain_cur);
			return;
		}

		// Non-blocking rate limit: skip if not enough time has elapsed since last processing
		if (last_camera_process_time_ &&
		    (now() - *last_camera_process_time_).seconds() < 1.0 / img_proc_loop_hz_) {
			return;
		}

		try {
			check_rate = false;

			cv::Mat image_capture;
			try {
				image_capture = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
			} catch (cv_bridge::Exception& e) {
				RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
			}

			cv::Mat image_current;
			//cv::Size size(512,612); // may want to try size(408,342) if speed is limited
			cv::Size size(342,408);
			cv::resize(image_capture, image_current, size);
			//image_capture = image_current;

			///////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Call the image processing funciton here (i.e. the gamma processing), returning a float point gamma value
			///////////////////////////////////////////////////////////////////////////////////////////////////////////

			// calculate the upper limit of shutter speed [unit:microsecond]
			// if ((1.0/frame_rate_req)*1000000.0 > upper_shutter_limit_param){        
			// 	upper_shutter_limit = upper_shutter_limit_param;
			// }
			// else{
			// 	upper_shutter_limit = round(1000000.0/frame_rate_req);
			// }

			// loop to call image_gradient_gamma function to obtain image gradient of each gamma
			// manually adjust the possible gamma values and the number of gamma to use
			for (int i = 0; i < GAMMAS_COUNT; ++i){
				metric[i]= image_gradient_gamma(image_current, i)/1000000; // passing the corresponding index
				//RCLCPP_INFO(get_logger(), "metric for gamma %f: %f", gamma[i], metric[i]); // comment
			}
							
			// loop to find out the index that correspond to the optimum/maximum gamma value
			double temp = -1.0;				
			for(int i = 0; i < GAMMAS_COUNT; i++){
				if (metric[i] > temp){
					temp = metric[i];
					gamma_index = i;
				}
			}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////  Curve Fitting  ///////////////////////////////////////////////

			// Call the curve fitting function to find out coefficient
			double * coeff_curve;
			coeff_curve = curveFit(gamma, metric);

	#ifdef WITH_PLOTTER
		if (enable_plotter) {
			plotter_gamma->clear();
			plotter_gamma->plot(
				gamma,
				metric,
				GAMMAS_COUNT,
				'*',
				2,
				cv::Scalar(255, 0, 0)
				);

			const int POINTS_COUNT = 10 * GAMMAS_COUNT;
			std::unique_ptr<double[]> x = std::make_unique<double[]>(POINTS_COUNT);
			std::unique_ptr<double[]> y = std::make_unique<double[]>(POINTS_COUNT);

			double x_min = gamma[0];
			double x_max = gamma[GAMMAS_COUNT - 1];
			double step = (x_max - x_min) / (POINTS_COUNT - 1);

			for (int i = 0; i < POINTS_COUNT; i++) {
				x[i] = x_min + i * step;
				double a = coeff_curve[0];
				double b = coeff_curve[1];
				double c = coeff_curve[2];
				y[i] = a * x[i] * x[i] + b * x[i] + c;
			}

			plotter_gamma->plot(
				x.get(),
				y.get(),
				POINTS_COUNT,
				'-',
				2,
				cv::Scalar(0, 0, 255)
				);

			plotter_gamma->publish();
		}
#endif

			double coeff[POLYNOME_DEGREE+1];
			for ( int i = 0; i < POLYNOME_DEGREE+1; i++) {
					
				coeff[i] = *(coeff_curve+i);
				//RCLCPP_INFO(get_logger(), "coeff %i is: %f", i, coeff[i]);
			}

			max_gamma = findRoots1(coeff, metric[gamma_index]); // calling function findRoots1 to find opt_gamma
			//RCLCPP_INFO(get_logger(), "opt_gamma now is:  %f", max_gamma);
			
			double metric_check = 0.0;
			for (int i=0; i < POLYNOME_DEGREE+1; i++) {
				metric_check = metric_check + coeff[i] * pow(max_gamma, POLYNOME_DEGREE-i);
			}				
			//RCLCPP_INFO(get_logger(), "metric_check = %f", metric_check);

			if (max_gamma < 1.0/1.9 || max_gamma > 1.9)	{
				// find out the optimum gamma value associated with highest image gradient
				max_gamma = gamma[gamma_index];
			}
			else if (metric[gamma_index] > metric_check) {
				max_gamma = gamma[gamma_index];
			}

			RCLCPP_INFO(get_logger(), "current opt met: %f; met at 1.0: %f", metric[gamma_index], metric[3]);

			// Store curve-fit coefficients and gamma result for the optimizer timer
			{
				std::lock_guard<std::mutex> lock(optimizer_mutex_);
				for (int i = 0; i < POLYNOME_DEGREE + 1; i++) coeff_[i] = coeff[i];
				// gamma_index and max_gamma are already updated as members above
			}

			last_camera_process_time_ = std::make_shared<rclcpp::Time>(now());
		}
		catch (cv_bridge::Exception& e) {
			RCLCPP_ERROR(get_logger(), "Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
		}
	}

	double ExpNode::image_gradient_gamma(cv::Mat &src_img, int j) {
		// Accepting the raw image and the index of gamma value as input argument

		cv::Mat res = src_img.clone();
		///////////////////// The following computes the image gradient of the gamma-processed image /////////////////////
		cv::Mat grad_x, grad_y;
		cv::Mat abs_grad_x, abs_grad_y, dst_img;
		cv::Mat weight_ori = cv::Mat::ones(res.rows,res.cols,CV_64FC1);

		// Using the corresponding index to find out the correct lookuptable to use.
		// This first lookup table transformation performs normalization of the image to [0,1]
		// interval and changes the image's gamma.
		if (j == 0){
			cv::LUT(src_img, lookUpTable_01, res);
		}
		else if (j == 1){
			cv::LUT(src_img, lookUpTable_05, res);
		}
		else if (j == 2){
			cv::LUT(src_img, lookUpTable_08, res);
		}
		else if (j == 3){
			cv::LUT(src_img, lookUpTable_1, res);
		}
		else if (j == 4){
			cv::LUT(src_img, lookUpTable_12, res);
		}
		else if (j == 5){
			cv::LUT(src_img, lookUpTable_15, res);
		}
		else if (j == 6){
			cv::LUT(src_img, lookUpTable_19, res);
		}

		// Define variables that will be used in the sobel gradient determination function
		int scale = 1;
		int delta = 0;
		int ddepth = CV_8UC1;

		// Call the Sobel function to determine gradient image in x and y direction

		// Gradient X
		cv::Sobel(res, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
		cv::convertScaleAbs(grad_x, abs_grad_x);

		// Gradient Y
		cv::Sobel(res, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
		cv::convertScaleAbs(grad_y, abs_grad_y);

		cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, dst_img);

		////////////////////// The following computes the gradient metric based on the gradient image ///////////////////////////

		// Method 1: simple sumation of total gradient
		//double metric= cv::sum(dst_img)[0]; // simple sum of total gradient as metric (Alternative 1 of the Shim's metric) 
		
		// Method 2: Shim's 2014 gradient metric function
		// Using the metric equation given in Shim's 2014 paper
		/* This second lookup table mapping streghtens smaller gradients and keeps the higher gradients as they are.
		   For example, if signa = 15 and lamda = 1000:
			- gradients with value smaller than 64 are thrown away
			- some example gradients value mapping:
				- 16 -> 64
				- 30 -> 178
				- 50 -> 201
				- 100 -> 226
				- 200 -> 247
				- 250 -> 254
				- 255 -> 255
		*/ 
		cv::LUT(dst_img, lookUpTable_metric, res);
		res.convertTo(res, CV_64FC1);
		
		double metric= cv::sum(res)[0];

		//cv::imshow("image_to_show",dst_img); // comment later
		return metric;
	}

	void ExpNode::ChangeParam (double shutter_new, double gain_new) {
		std_msgs::msg::Int32 shutter_speed_msg;
		shutter_speed_msg.data = shutter_new * 1000000; // from seconds to microseconds
		shutter_speed_us_pub->publish(shutter_speed_msg);

		std_msgs::msg::Float32 gain_msg;
		gain_msg.data = gain_new;
		gain_db_pub->publish(gain_msg);
	}

    // void ExpNode::ChangeParam (double shutter_new, double gain_new) // may have input of the updated gain, exposure time settings
    // {
    //     dynamic_reconfigure::ReconfigureRequest srv_req;
    //     dynamic_reconfigure::ReconfigureResponse srv_resp;
    //     dynamic_reconfigure::BoolParameter acq_fps_bool; //enable "acquisition_frame_rate_enable"
    //     dynamic_reconfigure::StrParameter gain_auto_str,exp_auto_str,wb_auto_str; // Auto gain, white balance and exposure off
    //     dynamic_reconfigure::DoubleParameter acq_fps_double, exp_time_double, gain_double, exp_auto_upper_double;
    //     dynamic_reconfigure::Config conf;

	// // set constant frame rate
    //     acq_fps_bool.name = "acquisition_frame_rate_enable"; // maximum frame rate: 80 fps
    //     acq_fps_bool.value = true;
    //     conf.bools.push_back(acq_fps_bool);

	// // trun off built-in auto exposure
    //     exp_auto_str.name = "exposure_auto"; // shut off auto exposure
    //     exp_auto_str.value = "Off";
    //     conf.strs.push_back(exp_auto_str);

	// // turn off auto gain
    //     gain_auto_str.name = "auto_gain"; // shut off auto gain adjustment
    //     gain_auto_str.value = "Off";
    //     conf.strs.push_back(gain_auto_str);

	// /*
    //     wb_auto_str.name = "auto_white_balance"; // shut off auto white balance
    //     wb_auto_str.value = "Off";
    //     conf.strs.push_back(wb_auto_str);
	// */

	// // Set required frame rate
    //     acq_fps_double.name = "acquisition_frame_rate"; // maximum frame rate: 80 fps
    //     acq_fps_double.value = frame_rate_req; //change frame rate as needed
    //     conf.doubles.push_back(acq_fps_double);

	// // Update exposure time
    //     exp_time_double.name = "exposure_time"; // Maximum range: 0 to 32754 [unit: micro-seconds]
    //     exp_time_double.value = shutter_new; ///////////////////////////// using function return value
    //     conf.doubles.push_back(exp_time_double);

	// // Update gain
    //     gain_double.name = "gain"; // Maximum range: -10 to 30
    //     gain_double.value = gain_new; ///////////////////////////////////// using function return value
    //     conf.doubles.push_back(gain_double);

	// // calculate and set highest possible exposure time (the camera has a limit of 32754 [unit: microsecond])
    //     exp_auto_upper_double.name = "auto_exposure_time_upper_limit";
	// if ((1.0/frame_rate_req)*1000000.0 > 32754.0){        
	// 	exp_auto_upper_double.value = 32754.0;
	// }
	// else{
	// 	exp_auto_upper_double.value = 1000000.0/frame_rate_req;
	// }
    //     conf.doubles.push_back(exp_auto_upper_double);

    //     srv_req.config = conf;

    //     //ros::service::call("/blackfly/spinnaker_camera_nodelet/set_parameters",srv_req, srv_resp);
	// ros::service::call(service_call,srv_req, srv_resp);
    // }

	void ExpNode::generate_LUT (){
		

		// Need to keep the same gamma values as in imageCallBack
		double gamma[7]={1.0/1.9, 1.0/1.5, 1.0/1.2 ,1.0, 1.2, 1.5, 1.9}; 		
		
		double sigma = 255.0 * met_act_thresh; // The sigma value is used in Shim's 2014 paper as a activation threshhold, the paper used a value of 0.06    
    	//double lamda = 1000.0; // The lamda value used in Shim's 2014 paper as a control parameter to adjust the mapping tendency (larger->steeper) 


		uchar* p;
		uchar* q = lookUpTable_metric.ptr();

		for (int j = 0; j < 7; j++){
			if (j == 0){
				p = lookUpTable_01.ptr();
			}
			else if (j == 1){
				p = lookUpTable_05.ptr();
			}
			else if (j == 2){
				p = lookUpTable_08.ptr();
			}
			else if (j == 3){
				p = lookUpTable_1.ptr();
			}
			else if (j == 4){
				p = lookUpTable_12.ptr();
			}
			else if (j == 5){
				p = lookUpTable_15.ptr();
			}
			else if (j == 6){
				p = lookUpTable_19.ptr();
			}
			
			for( int i = 0; i < 256; ++i) {
				p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, 1/gamma[j]) * 255.0);
	
				// The following if statement to create a lookup table based on the activation threshold value in Shim's 2014 paper
				if (i >= sigma){
					q[i] = 255 * ( ( log10( lamda * ((i-sigma)/255.0) + 1) ) / ( log10( lamda * ((255.0-sigma)/255.0) + 1) ) );
				}
				else{
					q[i] = 0;
				}
	
			} // end of for loop with index i

		}// end of for loop with index j

   		
	} // end of generate_LUT()
	

	// double  ExpNode::findRoots1 (double a[6], double check)
	// {
	// 	static double roots1[4];
	// 	double ad[5];
	// 	double opt_gamma = 997.0, met_temp;
	// 	ad[4] = 5 * a[0];
	// 	ad[3] = 4 * a[1];
	// 	ad[2] = 3 * a[2];
	// 	ad[1] = 2 * a[3];
	// 	ad[0] = a[4];
	// 	Eigen::MatrixXd companion_mat (4, 4);

	// 	for (int n = 0; n < 4; n++)
	// 	{
	// 		for (int m = 0; m < 4; m++)
	// 			{
	// 			 if (n == m + 1)
	// 			 	companion_mat (n, m) = 1.0;
	// 			 if (m == 4 - 1)
	// 				companion_mat (n, m) = -ad[n] / ad[4];
	// 		} // end of for loop with index m
	// 	} // end of for loop with index n

	// 	Eigen::MatrixXcd eig = companion_mat.eigenvalues ();
	// 	for (int i = 0; i < 4; i++)
	// 	{
	// 	 	met_temp = 0.0; // met_temp is used to check if the root can return a larger metric      
	// 	 	if (std::imag (eig (i)) == 0) // if statement to determine whether or not root is true
	// 	  	{
	// 	  		roots1[i] = std::real(eig (i));	 
	// 	  	}
	// 	  	else
	// 	  	{

	// 	  		roots1[i] = 1000; // if root is imaginary, assign an overshoot value
	// 	  	}
	// 	  if ((roots1[i] < 2.0) && (roots1[i] > 0.5)) //check if the calculated root is within range (.5,2)
	// 	  	{
	// 	  		for (int j=0; j<6; j++) 
	// 	  		{
	// 	  			met_temp = met_temp + a[j] * pow(roots1[i],5-j);
	// 	  		}
	// 	  		if (met_temp > check) // if the root can return a metric that is greater than current metric
	// 	  		{
	// 	  			opt_gamma = roots1[i];
	// 	  			RCLCPP_INFO(get_logger(), "in function maximum metric is: %f", met_temp);
	// 	  		}
	// 	  	}
	// 		RCLCPP_INFO(get_logger(), "eig(i) is: %f, ima: %f", std::real(eig(i)), std::imag(eig(i)));
	// 	} // end of for loop with index i
	// 	return opt_gamma;
	// } // END of function of findRoots1()

// double ExpNode::findRoots1(double a[3], double check)
// {
// 	double lowest_gamma = gamma[0];
// 	double highest_gamma = gamma[GAMMAS_COUNT-1];
// 	double neutral_gamma = 1.0;

//     // double opt_gamma = 997.0, met_temp;
// 	double opt_gamma = 1.0, met_temp;
    
//     // Derivative of quadratic ax^2 + bx + c is: 2ax + b
//     // Setting derivative = 0: 2ax + b = 0
//     // Root: x = -b/(2a)
    
//     double derivative_root;
    
//     // Check if we have a valid quadratic (a[0] != 0)
//     if (std::abs(a[0]) < 1e-10)
//     {
//         RCLCPP_WARN(get_logger(), "Coefficient a[0] too small, not a valid quadratic");
//         return opt_gamma;
//     }

// 	// If the polynomial is convex, we are way off from the optimal gamma.
// 	// We need to just pick the gamma with the largest metric.
// 	double derrivative_at_gamma_1 = 2*a[0] + a[1];
// 	RCLCPP_INFO(get_logger(), "derrivative_at_gamma_1: %f", derrivative_at_gamma_1);
// 	if (a[0] > 0) {
// 		// First, determine if the function is increasing or decreasing
// 		RCLCPP_INFO(get_logger(), "PARABOLA IS CONVEX!");
// 		// we have to multipoly it by the derrivative itself because it would oscillate arround the peak
// 		// if(derrivative_at_gamma_1 >= 0) return highest_gamma;
// 		// if(derrivative_at_gamma_1 < 0) return lowest_gamma;
// 		return 1.0;
// 	}
    
//     // Calculate the critical point (where derivative = 0)
//     derivative_root = -a[1] / (2.0 * a[0]);
    
//     //RCLCPP_INFO(get_logger(), "Critical point at x = %f", derivative_root);
    
//     // Check if the root is within range (0.5, 2.0)
//     if ((derivative_root <= highest_gamma) && (derivative_root >= lowest_gamma))
//     {
//         // Evaluate the polynomial at this point
//         met_temp = a[0] * derivative_root * derivative_root + 
//                    a[1] * derivative_root + 
//                    a[2];
        
//         //RCLCPP_INFO(get_logger(), "Metric at critical point: %f", met_temp);
        
//         // Check if this gives a better metric than current
//         //if (met_temp > check)	// this can cause some jumping of the opt_gamma value; we rather choose little bit suboptimal, but stable value
//         {
//             opt_gamma = derivative_root;
//             //RCLCPP_INFO(get_logger(), "Found better maximum metric: %f at x = %f", 
//             //           met_temp, opt_gamma);
//         }
//     }
//     else
//     {
//         RCLCPP_INFO(get_logger(), "Critical point %f outside range (0.5, 2.0)", derivative_root);
// 		// if(derivative_root >= highest_gamma) opt_gamma = highest_gamma;
// 		// if(derivative_root <= lowest_gamma) opt_gamma = lowest_gamma;
//     }
    
//     return opt_gamma;
// } // END of function findRoots1()

double ExpNode::findRoots1(double a[3], double check)
{
    double lowest_gamma = gamma[0];
    double highest_gamma = gamma[GAMMAS_COUNT-1];
    double opt_gamma = 1.0;
    
    // Check if we have a valid quadratic (a[0] != 0)
    if (std::abs(a[0]) < 1e-10)
    {
        RCLCPP_WARN(get_logger(), "Coefficient a[0] too small, not a valid quadratic");
        return 1.0;  // Return neutral
    }

    double derrivative_at_gamma_1 = 2*a[0] + a[1];
    
    // If the polynomial is convex, the curve fit is poor for finding a maximum
    // Use a SMALL correction based on derivative direction instead of jumping to extremes
    if (a[0] > 0) {
        RCLCPP_INFO(get_logger(), "PARABOLA IS CONVEX - using small correction");
        // Small step in the direction indicated by derivative, not extreme values
        if (derrivative_at_gamma_1 >= 0) 
            return 1.05;  // Small step toward brighter (was 1.9!)
        else 
            return 0.95;  // Small step toward darker (was 0.526!)
    }
    
    // Calculate the critical point (where derivative = 0)
    double derivative_root = -a[1] / (2.0 * a[0]);
    
    // Check if the root is within valid range
    if ((derivative_root <= highest_gamma) && (derivative_root >= lowest_gamma))
    {
        opt_gamma = derivative_root;
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Critical point %f outside range - using small correction", derivative_root);
        // Instead of returning extremes, return a small correction
        if (derivative_root > highest_gamma)
            return 1.05;  // Small step toward brighter
        else
            return 0.95;  // Small step toward darker
    }
    
    return opt_gamma;
}

// double * ExpNode::curveFit (double x[7], double y[7])
// { static double coff[6];
//   int i, j, k, n, N;

//   n = 5;
  
//   Eigen::MatrixXd A(7,6);
//   Eigen::MatrixXd b(7,1);
   
//   for (i = 0; i <7; i++)
//       for (j = 5; j>=0; j--)
// 	{A(i,5-j) = pow (x[i], j);}
    
//   for (i = 0; i <7; i++)
//    {  b(i,0) = y[i]; } 
  
//   Eigen::MatrixXd A1 = A.transpose()*A;
//   Eigen::MatrixXd b1 = A.transpose()*b; 
//   //Eigen::MatrixXd Q =A1.colPivHouseholderQr().solve(b1);
//   Eigen::MatrixXd Q =A1.inverse()*b1;

//   for(i=0; i<n+1; i++)
//   {  coff[i] = Q(i);
// 	//std::cout << "\nx is: " << x[i] << "Q is: " << coff[i] << std::endl;
// }

//   return coff;
   
// } // END of function curveFit()

double * ExpNode::curveFit(double x[7], double y[7])
{ 
    static double coff[3];  // Only need 3 coefficients for degree 2
    int i;
  
    // Create matrices for degree 2 polynomial: y = a*x^2 + b*x + c
    Eigen::MatrixXd A(7, 3);
    Eigen::MatrixXd b(7, 1);
   
    // Fill matrix A with [x^2, x, 1] for each point
    for (i = 0; i < 7; i++)
    {
        A(i, 0) = x[i] * x[i];  // x^2
        A(i, 1) = x[i];         // x
        A(i, 2) = 1.0;          // constant term
    }
    
    // Fill vector b with y values
    for (i = 0; i < 7; i++)
    {
        b(i, 0) = y[i];
    }
  
    // Solve least squares problem directly (no normal equations)
    Eigen::MatrixXd Q = A.colPivHouseholderQr().solve(b);

    // Extract coefficients
    for (i = 0; i < 3; i++)
    {
        coff[i] = Q(i);
    }

    return coff;
} // END of function curveFit()

} //END OF THE WHOLE NAMESPACE

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(exp_node::ExpNode)