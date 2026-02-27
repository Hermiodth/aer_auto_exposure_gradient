#include "aer_auto_exposure_gradient/auto_exp.h"



namespace exp_node 
{
	//ExpNode::ExpNode () : rclcpp::Node("exp_node"), callback_start_time(nullptr)

	void ExpNode::init(std::shared_ptr<rclcpp::Node> node_ptr){
		it_ = std::make_shared<image_transport::ImageTransport>(node_ptr);

		declare_parameter<std::string>("image_topic", "camera/image_raw");
		get_parameter("image_topic", image_topic);
    	RCLCPP_INFO(get_logger(), "image topic: %s", image_topic.c_str());

		// --- Actuator configuration ---
		// Each actuator owns a contiguous slice of the normalized [0,1] optimizer output.
		// The slices are laid out as: [shutter | gain | LED] from 0 to 1.

		declare_parameter<double>("shutter_portion", 0.3);
		get_parameter("shutter_portion", shutter_portion_);
		RCLCPP_INFO(get_logger(), "shutter_portion: %.2f", shutter_portion_);

		declare_parameter<int>("shutter_max_us", 3000);
		int shutter_max_us;
		get_parameter("shutter_max_us", shutter_max_us);
		shutter_max_s_ = (double)shutter_max_us / 1000000.0;
		RCLCPP_INFO(get_logger(), "shutter_max: %d µs", shutter_max_us);

		declare_parameter<double>("gain_portion", 0.5);
		get_parameter("gain_portion", gain_portion_);
		RCLCPP_INFO(get_logger(), "gain_portion: %.2f", gain_portion_);

		declare_parameter<double>("gain_max", 12.0);
		get_parameter("gain_max", gain_max_);
		RCLCPP_INFO(get_logger(), "gain_max: %.2f", gain_max_);

		declare_parameter<double>("led_portion", 0.0);
		get_parameter("led_portion", led_portion_);
		RCLCPP_INFO(get_logger(), "led_portion: %.2f", led_portion_);

		declare_parameter<double>("led_max", 40.0);
		get_parameter("led_max", led_max_);
		RCLCPP_INFO(get_logger(), "led_max: %.2f W", led_max_);

		// Build the ordered slice list from the actuator_order parameter.
		// Default: shutter first, then gain, then LED.
		declare_parameter<std::vector<std::string>>("actuator_order",
		    std::vector<std::string>{"shutter", "gain", "led"});
		std::vector<std::string> actuator_order_param;
		get_parameter("actuator_order", actuator_order_param);

		// Store originals for proportional scaling when a dynamic limit arrives
		shutter_max_s_original_   = shutter_max_s_;
		shutter_portion_original_ = shutter_portion_;

		actuator_slices_.clear();
		for (const auto& name : actuator_order_param) {
			ActuatorSlice slice;
			if (name == "shutter") {
				slice.type      = ActuatorSlice::Type::SHUTTER;
				slice.portion   = shutter_portion_;
				slice.max_value = shutter_max_s_;
			} else if (name == "gain") {
				slice.type      = ActuatorSlice::Type::GAIN;
				slice.portion   = gain_portion_;
				slice.max_value = gain_max_;
			} else if (name == "led") {
				slice.type      = ActuatorSlice::Type::LED;
				slice.portion   = led_portion_;
				slice.max_value = led_max_;
			} else {
				RCLCPP_WARN(get_logger(), "Unknown actuator '%s' in actuator_order — skipping", name.c_str());
				continue;
			}
			actuator_slices_.push_back(slice);
			RCLCPP_INFO(get_logger(), "  actuator slot %zu: '%s'  portion=%.2f  max=%.2f",
			            actuator_slices_.size(), name.c_str(), slice.portion, slice.max_value);
		}
		// --- End actuator configuration ---

		declare_parameter<double>("kp", 0.02);
		get_parameter("kp", kp);
    	RCLCPP_INFO(get_logger(), "kp param: %.2f", kp);

		declare_parameter<double>("grad_k", 1.0);
		get_parameter("grad_k", grad_k);
    	RCLCPP_INFO(get_logger(), "grad_k: %.2f", grad_k);

		declare_parameter<bool>("do_sweep", false);
		get_parameter("do_sweep", do_sweep);
		RCLCPP_INFO(get_logger(), "do_sweep: %s", do_sweep ? "yes" : "no");

		declare_parameter<double>("gamma_x_offset", 0.0);
		get_parameter("gamma_x_offset", gamma_x_offset_);
    	RCLCPP_INFO(get_logger(), "gamma_x_offset: %.2f", gamma_x_offset_);

		declare_parameter<std::string>("curve_fit_method", "quadratic");
		get_parameter("curve_fit_method", curve_fit_method_);
		RCLCPP_INFO(get_logger(), "curve_fit_method: %s", curve_fit_method_.c_str());

		declare_parameter<double>("initial_exposure_level", 0.1);
		get_parameter("initial_exposure_level", exposure_level_cur_);
		exposure_level_at_camera_ = exposure_level_cur_;
		exposure_level_new_       = exposure_level_cur_;
    	RCLCPP_INFO(get_logger(), "initial_exposure_level: %.3f", exposure_level_cur_);

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
        
		declare_parameter<double>("gamma_range", 1.7);
		get_parameter("gamma_range", gamma_range_);
		RCLCPP_INFO(get_logger(), "gamma_range: %.2f", gamma_range_);

		declare_parameter<int>("gamma_num_points", 3);
		get_parameter("gamma_num_points", gamma_num_points_);
		RCLCPP_INFO(get_logger(), "gamma_num_points: %i", gamma_num_points_);

		// Generate gamma array in log-space: 1/gamma_range ... 1.0 ... gamma_range
		gamma_.resize(gamma_num_points_);
		metric_.resize(gamma_num_points_, 0.0);
		gamma_luts_.resize(gamma_num_points_);
		for (int i = 0; i < gamma_num_points_; i++) {
			gamma_luts_[i] = cv::Mat(1, 256, CV_8U);
			gamma_[i] = (gamma_num_points_ == 1)
				? 1.0
				: std::pow(gamma_range_, 2.0 * i / (gamma_num_points_ - 1) - 1.0);
		}

		// Find neutral gamma index (closest to 1.0)
		gamma_neutral_index_ = 0;
		double min_dist = std::abs(gamma_[0] - 1.0);
		for (int i = 1; i < gamma_num_points_; i++) {
			double dist = std::abs(gamma_[i] - 1.0);
			if (dist < min_dist) { min_dist = dist; gamma_neutral_index_ = i; }
		}
		RCLCPP_INFO(get_logger(), "gamma_neutral_index: %i (gamma=%.4f)", gamma_neutral_index_, gamma_[gamma_neutral_index_]);

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

		declare_parameter<std::string>("led_apply_topic", "");
		std::string led_topic;
		get_parameter("led_apply_topic", led_topic);
		if (!led_topic.empty() && led_portion_ > 0.0) {
			led_pub_ = this->create_publisher<std_msgs::msg::Float32>(led_topic, 10);
			RCLCPP_INFO(get_logger(), "led apply topic: %s", led_topic.c_str());
		} else {
			RCLCPP_INFO(get_logger(), "LED actuator disabled (led_portion=%.2f, topic='%s')", led_portion_, led_topic.c_str());
		}

		declare_parameter<std::string>("shutter_limit_topic", "");
		std::string shutter_limit_topic;
		get_parameter("shutter_limit_topic", shutter_limit_topic);
		if (!shutter_limit_topic.empty()) {
			shutter_limit_sub_ = this->create_subscription<std_msgs::msg::Int32>(
				shutter_limit_topic, 10,
				std::bind(&ExpNode::shutterLimitCb, this, std::placeholders::_1));
			RCLCPP_INFO(get_logger(), "shutter limit topic: %s", shutter_limit_topic.c_str());
		} else {
			RCLCPP_INFO(get_logger(), "Dynamic shutter limit disabled (shutter_limit_topic not set)");
		}

		gamma_est_pub_         = this->create_publisher<std_msgs::msg::Float32>("gradient/gamma_est", 10);
		gradient_pub_          = this->create_publisher<std_msgs::msg::Float32>("gradient/D", 10);
		gradient_clipped_pub_  = this->create_publisher<std_msgs::msg::Float32>("gradient/D_clipped", 10);

		declare_parameter<std::string>("shutter_update_method", "simple");
		get_parameter("shutter_update_method", shutter_update_method);
		RCLCPP_INFO(get_logger(), "shutter update method: %s", shutter_update_method.c_str());

		declare_parameter<std::string>("shim_update_function", "2018");
		get_parameter("shim_update_function", shim_update_function);
		RCLCPP_INFO(get_logger(), "shim update function: %s", shim_update_function.c_str());

		declare_parameter<int>("sweep_steps", 100);
		get_parameter("sweep_steps", sweep_steps_);
		RCLCPP_INFO(get_logger(), "sweep_steps: %i", sweep_steps_);

		declare_parameter<double>("simple_step_size", 0.01);
		get_parameter("simple_step_size", simple_step_size_);
		RCLCPP_INFO(get_logger(), "simple_step_size: %.4f", simple_step_size_);

		test_sweep_step_ = 0;
		metric_tmp = 0;

		// Initialize shared optimizer state
		max_gamma   = 1.0;
		gamma_index = gamma_neutral_index_;
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
			plotter_gamma->setTitle(curve_fit_method_);
			plotter_gamma->setTitleFontSize(3.5);
			plotter_gamma->setTickFontSize(2.5);
			plotter_gamma->setLegendFontSize(3.0);

			plotter_sweep = std::make_shared<plotter_ros2::Plotter>(
				node_ptr,
				"plot_sweep",
				800,
				600,
				cv::Scalar(255, 255, 255)
			);
			plotter_sweep->setTitle("gamma sweep");
			plotter_sweep->setTitleFontSize(3.5);
			plotter_sweep->setTickFontSize(2.5);
			plotter_sweep->setLegendFontSize(3.0);
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

		exposure_level_new_ = std::clamp(exposure_level_new_, 0.0, 1.0);
		RCLCPP_INFO(get_logger(), "exposure_level_new: %.4f", exposure_level_new_);

		ChangeParam(exposure_level_new_);
		exposure_level_cur_ = exposure_level_new_;
	}

	void ExpNode::optimizeGradient(){
		double local_coeff[POLYNOME_DEGREE + 1];
		double local_exposure_level_at_camera;
		bool   local_new_camera_data;
		{
			std::lock_guard<std::mutex> lock(optimizer_mutex_);
			for (int i = 0; i < POLYNOME_DEGREE + 1; i++) {
				local_coeff[i] = coeff_[i];
			}
			local_exposure_level_at_camera = exposure_level_at_camera_;
			local_new_camera_data          = new_camera_data_;
			new_camera_data_               = false;
		}

		// Reset gamma estimate whenever a new image has been processed
		if (local_new_camera_data) {
			gamma_est_ = 1.0;
		}

		// Gradient of the fitted curve evaluated at the current gamma estimate.
		// gamma_est_ changes each optimizer step, so D changes too.
		// gamma_x_offset_ shifts the convergence point (in x-space for quadratic,
		// in log-x-space for log_quadratic).
		double D;
		if (curve_fit_method_ == "log_quadratic") {
			// df/dx = (2A*ln(x) + B) / x; offset applied in log-space
			D = (2 * local_coeff[0] * (std::log(gamma_est_) - gamma_x_offset_) + local_coeff[1])
			    / gamma_est_;
		} else {
			// df/dx = 2A*x + B (quadratic)
			D = 2 * local_coeff[0] * (gamma_est_ - gamma_x_offset_) + local_coeff[1];
		}
		RCLCPP_INFO(get_logger(), "GRADIENT at gamma=%.3f: %.2f", gamma_est_, D);
		double max_grad = 0.5;
		double D_clipped = std::clamp(D, -max_grad, max_grad);

		// Gradient ascent step in gamma space
		gamma_est_ += grad_k * D_clipped;

		// Scale exposure level multiplicatively, anchored on the level active when
		// the current curve fit was computed (same idea as before, now in [0,1] space)
		exposure_level_new_ = local_exposure_level_at_camera * gamma_est_;

		std_msgs::msg::Float32 msg;
		msg.data = static_cast<float>(gamma_est_);
		gamma_est_pub_->publish(msg);
		msg.data = static_cast<float>(D);
		gradient_pub_->publish(msg);
		msg.data = static_cast<float>(D_clipped);
		gradient_clipped_pub_->publish(msg);
	}

	void ExpNode::optimizeSimple(){
		int local_gamma_index;
		{
			std::lock_guard<std::mutex> lock(optimizer_mutex_);
			local_gamma_index = gamma_index;
		}
		// Step in normalized [0,1] space; simple_step_size_ controls convergence speed
		exposure_level_new_ = exposure_level_cur_ + simple_step_size_ * (local_gamma_index - gamma_neutral_index_);
	}

	void ExpNode::optimizeShim(){
		double local_max_gamma;
		{
			std::lock_guard<std::mutex> lock(optimizer_mutex_);
			local_max_gamma = max_gamma;
		}
		alpha = 1.0;

		// Work in a virtual EV space derived from the normalized exposure level:
		//   EV = log2(1 / exposure_level)   →   exposure_level = 2^(-EV)
		// Higher EV ↔ lower exposure level ↔ darker image, consistent with the
		// original formula where higher expCur meant shorter shutter (less light).
		double level = std::max(exposure_level_cur_, 1e-6);
		expCur = std::log2(1.0 / level);

		if (shim_update_function == "2014") {
			expNew = (1 + kp * alpha * (1 - local_max_gamma)) * expCur;
		} else { // "2018"
			double gamma_nudge = 0.0;
			if (local_max_gamma >= 1.0) R = -pow((local_max_gamma - (1.0 - gamma_nudge)), 2) + 1;
			else                        R =  pow((local_max_gamma - (1.0 - gamma_nudge)), 2) + 1;
			expNew = (1 + alpha * kp * (R - 1)) * expCur;
		}
		exposure_level_new_ = 1.0 / std::pow(2.0, expNew);
	}
	
	void ExpNode::CameraCb (const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
		//// code for measuring the true optimal exposure time

		// if(!zeroing_duration) zeroing_duration = std::make_shared<rclcpp::Time>(now());
		// if((now() - *zeroing_duration.get()).seconds() < 1) {
		// 	ChangeParam(0, 0.0);
		// 	return;
		// }

		//Run a sweep over [0,1] exposure levels to see where the true optimum lies
		if(do_sweep){
			if(test_sweep_step_ < sweep_steps_) {
				double test_level = (sweep_steps_ > 1)
					? (double)test_sweep_step_ / (sweep_steps_ - 1)
					: 0.5;

				cv::Mat image1;
				try {
					image1 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
				} catch (cv_bridge::Exception& e) {
					RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
				}

				cv::Mat image2;
				cv::Size size(342,408);
				cv::resize(image1, image2, size);

				double test_metric = image_gradient_gamma(image2, 3);
				RCLCPP_INFO(get_logger(), "sweep step: %i/%i (level=%.3f), metric: %f",
				            test_sweep_step_, sweep_steps_, test_level, test_metric);
				if(test_metric > metric_tmp) {
					true_best_exposure_level_ = test_level;
					metric_tmp = test_metric;
				}
				RCLCPP_INFO(get_logger(), "best level: %.3f, best metric: %f", true_best_exposure_level_, metric_tmp);

	#ifdef WITH_PLOTTER
				if (enable_plotter) {
					sweep_levels_.push_back(test_level);
					sweep_metrics_.push_back(test_metric);
				}
	#endif

				ChangeParam(test_level);
				test_sweep_step_++;

	#ifdef WITH_PLOTTER
				if (enable_plotter && test_sweep_step_ >= sweep_steps_) {
					plotter_sweep->clear();
					plotter_sweep->plot(
						sweep_levels_.data(),
						sweep_metrics_.data(),
						(int)sweep_levels_.size(),
						'*',
						2,
						cv::Scalar(255, 0, 0),
						"metric val"
					);
					double best_x[1] = { true_best_exposure_level_ };
					double best_y[1] = { metric_tmp };
					plotter_sweep->plot(best_x, best_y, 1, '*', 6, cv::Scalar(0, 0, 255), "best gamma");
					plotter_sweep->publish();
					RCLCPP_INFO(get_logger(), "Sweep done. Best exposure level: %.3f. Starting periodic republish.", true_best_exposure_level_);
					sweep_republish_timer_ = create_wall_timer(
						std::chrono::seconds(1),
						[this]() { plotter_sweep->publish(); }
					);
				}
	#endif

				usleep(300000);

				return;
			}
		}

		if(!callback_start_time) callback_start_time = std::make_shared<rclcpp::Time>(now());
		if((now() - *callback_start_time.get()).seconds() < startup_delay) {
			RCLCPP_INFO(get_logger(), "startup delay: %i; will wait for %f s; current exposure_level: %.3f",
			startup_delay, (now() - *callback_start_time.get()).seconds(), exposure_level_cur_);
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
			for (int i = 0; i < gamma_num_points_; ++i){
				metric_[i] = image_gradient_gamma(image_current, i)/1000000.0; // passing the corresponding index
				//RCLCPP_INFO(get_logger(), "metric for gamma %f: %f", gamma_[i], metric_[i]); // comment
			}

			// loop to find out the index that correspond to the optimum/maximum gamma value
			double temp = -1.0;
			for(int i = 0; i < gamma_num_points_; i++){
				if (metric_[i] > temp){
					temp = metric_[i];
					gamma_index = i;
				}
			}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////  Curve Fitting  ///////////////////////////////////////////////

			// Call the curve fitting function to find out coefficient
			double * coeff_curve;
			if (curve_fit_method_ == "log_quadratic")
				coeff_curve = curveFitLogQuadratic(gamma_, metric_);
			else
				coeff_curve = curveFitQuadratic(gamma_, metric_);

	#ifdef WITH_PLOTTER
		if (enable_plotter) {
			plotter_gamma->clear();
			plotter_gamma->plot(
				gamma_.data(),
				metric_.data(),
				(int)gamma_.size(),
				'*',
				2,
				cv::Scalar(255, 0, 0),
				"metric"
				);

			const int POINTS_COUNT = 10 * gamma_num_points_;
			std::unique_ptr<double[]> x = std::make_unique<double[]>(POINTS_COUNT);
			std::unique_ptr<double[]> y = std::make_unique<double[]>(POINTS_COUNT);

			double x_min = gamma_.front();
			double x_max = gamma_.back();
			double step = (x_max - x_min) / (POINTS_COUNT - 1);

			for (int i = 0; i < POINTS_COUNT; i++) {
				x[i] = x_min + i * step;
				double a = coeff_curve[0];
				double b = coeff_curve[1];
				double c = coeff_curve[2];
				if (curve_fit_method_ == "log_quadratic") {
					double u = std::log(x[i]);
					y[i] = a * u * u + b * u + c;
				} else {
					y[i] = a * x[i] * x[i] + b * x[i] + c;
				}
			}

			plotter_gamma->plot(
				x.get(),
				y.get(),
				POINTS_COUNT,
				'-',
				2,
				cv::Scalar(0, 0, 255),
				curve_fit_method_
				);

			plotter_gamma->publish();
		}
#endif

			double coeff[POLYNOME_DEGREE+1];
			for ( int i = 0; i < POLYNOME_DEGREE+1; i++) {
					
				coeff[i] = *(coeff_curve+i);
				//RCLCPP_INFO(get_logger(), "coeff %i is: %f", i, coeff[i]);
			}

			max_gamma = findRoots1(coeff, metric_[gamma_index]); // calling function findRoots1 to find opt_gamma
			//RCLCPP_INFO(get_logger(), "opt_gamma now is:  %f", max_gamma);
			
			double metric_check = 0.0;
			if (curve_fit_method_ == "log_quadratic") {
				double u = std::log(max_gamma);
				metric_check = coeff[0] * u * u + coeff[1] * u + coeff[2];
			} else {
				for (int i = 0; i < POLYNOME_DEGREE + 1; i++)
					metric_check += coeff[i] * pow(max_gamma, POLYNOME_DEGREE - i);
			}
			//RCLCPP_INFO(get_logger(), "metric_check = %f", metric_check);

			if (max_gamma < gamma_.front() || max_gamma > gamma_.back()) {
				// find out the optimum gamma value associated with highest image gradient
				max_gamma = gamma_[gamma_index];
			}
			else if (metric_[gamma_index] > metric_check) {
				max_gamma = gamma_[gamma_index];
			}

			RCLCPP_INFO(get_logger(), "current opt met: %f; met at 1.0: %f", metric_[gamma_index], metric_[gamma_neutral_index_]);

			// Store curve-fit coefficients and gamma result for the optimizer timer
			{
				std::lock_guard<std::mutex> lock(optimizer_mutex_);
				for (int i = 0; i < POLYNOME_DEGREE + 1; i++) coeff_[i] = coeff[i];
				exposure_level_at_camera_ = exposure_level_cur_;
				new_camera_data_   = true;
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
		cv::LUT(src_img, gamma_luts_[j], res);

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
		cv::LUT(dst_img, lut_metric_, res);
		res.convertTo(res, CV_64FC1);
		
		double metric= cv::sum(res)[0];

		//cv::imshow("image_to_show",dst_img); // comment later
		return metric;
	}

	void ExpNode::shutterLimitCb(const std_msgs::msg::Int32::ConstSharedPtr& msg) {
		int new_max_us = msg->data;
		if (new_max_us <= 0) {
			RCLCPP_WARN(get_logger(), "Received invalid shutter limit: %d µs — ignoring", new_max_us);
			return;
		}

		double new_max_s  = new_max_us / 1000000.0;
		double new_portion = shutter_portion_original_ * (new_max_s / shutter_max_s_original_);
		new_portion = std::clamp(new_portion, 0.0, shutter_portion_original_);

		std::lock_guard<std::mutex> lock(actuator_mutex_);
		shutter_max_s_   = new_max_s;
		shutter_portion_ = new_portion;
		for (auto& slice : actuator_slices_) {
			if (slice.type == ActuatorSlice::Type::SHUTTER) {
				slice.max_value = new_max_s;
				slice.portion   = new_portion;
				break;
			}
		}
		RCLCPP_INFO(get_logger(), "Shutter limit updated: max=%d µs (%.4f s), portion=%.3f",
		            new_max_us, new_max_s, new_portion);
	}

	void ExpNode::ChangeParam(double exposure_level) {
		exposure_level = std::clamp(exposure_level, 0.0, 1.0);

		// Walk through the actuator slices in the configured order.
		// Each slice [cursor, cursor+portion) is mapped to [0, max_value] for that actuator.
		// Once the optimizer value falls below cursor, that actuator outputs 0.
		double cursor = 0.0;
		double shutter_s = 0.0, gain = 0.0, led = 0.0;

		std::lock_guard<std::mutex> lock(actuator_mutex_);
		for (const auto& slice : actuator_slices_) {
			double frac = 0.0;
			if (slice.portion > 0.0 && exposure_level > cursor) {
				frac = std::min(exposure_level - cursor, slice.portion) / slice.portion;
			}
			double value = frac * slice.max_value;
			cursor += slice.portion;

			switch (slice.type) {
				case ActuatorSlice::Type::SHUTTER: shutter_s = value; break;
				case ActuatorSlice::Type::GAIN:    gain      = value; break;
				case ActuatorSlice::Type::LED:     led       = value; break;
			}
		}

		std_msgs::msg::Int32 shutter_msg;
		shutter_msg.data = static_cast<int>(shutter_s * 1000000.0);  // s → µs
		shutter_speed_us_pub->publish(shutter_msg);

		std_msgs::msg::Float32 gain_msg;
		gain_msg.data = static_cast<float>(gain);
		gain_db_pub->publish(gain_msg);

		if (led_pub_) {
			std_msgs::msg::Float32 led_msg;
			led_msg.data = static_cast<float>(led);
			led_pub_->publish(led_msg);
		}

		RCLCPP_INFO(get_logger(), "ChangeParam: level=%.4f → shutter=%d µs, gain=%.2f dB, led=%.2f W",
		            exposure_level, shutter_msg.data, gain, led);
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
		double sigma = 255.0 * met_act_thresh;
		lut_metric_ = cv::Mat(1, 256, CV_8U);
		uchar* q = lut_metric_.ptr();

		for (int j = 0; j < gamma_num_points_; j++){
			uchar* p = gamma_luts_[j].ptr();
			for (int i = 0; i < 256; ++i) {
				p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, 1.0 / gamma_[j]) * 255.0);
				if (j == 0) { // compute metric LUT only once
					if (i >= sigma){
						q[i] = 255 * (log10(lamda * ((i-sigma)/255.0) + 1)) / (log10(lamda * ((255.0-sigma)/255.0) + 1));
					} else {
						q[i] = 0;
					}
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
    double lowest_gamma  = gamma_.front();
    double highest_gamma = gamma_.back();
    double opt_gamma     = 1.0;

    if (std::abs(a[0]) < 1e-10) {
        RCLCPP_WARN(get_logger(), "Coefficient a[0] too small, not a valid fit");
        return 1.0;
    }

    if (curve_fit_method_ == "log_quadratic") {
        // Coefficients [A, B, C] represent A*ln(x)^2 + B*ln(x) + C.
        // Maximum (for concave-down, A < 0) at: ln(x) = -B/(2A) → x = exp(-B/(2A)).
        if (a[0] > 0) {
            // Convex in log-space: use derivative direction at x=1 (ln(1)=0)
            RCLCPP_INFO(get_logger(), "LOG-QUAD IS CONVEX - using small correction");
            return (a[1] >= 0) ? 1.05 : 0.95;
        }
        double ln_opt = -a[1] / (2.0 * a[0]);
        opt_gamma = std::exp(ln_opt);
    } else {
        // Quadratic: A*x^2 + B*x + C, maximum at x = -B/(2A).
        double derrivative_at_gamma_1 = 2 * a[0] + a[1];
        if (a[0] > 0) {
            RCLCPP_INFO(get_logger(), "PARABOLA IS CONVEX - using small correction");
            return (derrivative_at_gamma_1 >= 0) ? 1.05 : 0.95;
        }
        opt_gamma = -a[1] / (2.0 * a[0]);
    }

    if (opt_gamma < lowest_gamma || opt_gamma > highest_gamma) {
        RCLCPP_INFO(get_logger(), "Critical point %f outside range - using small correction", opt_gamma);
        return (opt_gamma > highest_gamma) ? 1.05 : 0.95;
    }

    return opt_gamma;
}

// double * ExpNode::curveFit (double x[GAMMAS_COUNT], double y[GAMMAS_COUNT])
// { static double coff[6];
//   int i, j, k, n, N;

//   n = 5;
  
//   Eigen::MatrixXd A(GAMMAS_COUNT,6);
//   Eigen::MatrixXd b(GAMMAS_COUNT,1);
   
//   for (i = 0; i < GAMMAS_COUNT; i++)
//       for (j = 5; j>=0; j--)
// 	{A(i,5-j) = pow (x[i], j);}
    
//   for (i = 0; i < GAMMAS_COUNT; i++)
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

// Fits f(x) = a*(ln(x)-b)^2 + c by substituting u=ln(x), yielding A*u^2 + B*u + C.
// Stored coefficients [A, B, C] encode: A=a, B=-2ab, C=ab^2+c.
// Optimal x: exp(-B / (2A)).  Derivative df/dx = (2A*ln(x) + B) / x.
double * ExpNode::curveFitLogQuadratic(const std::vector<double>& x, const std::vector<double>& y)
{
    static double coff[3];
    int i;
    int n = (int)x.size();

    Eigen::MatrixXd A(n, 3);
    Eigen::MatrixXd b(n, 1);

    for (i = 0; i < n; i++) {
        double u = std::log(x[i]);
        A(i, 0) = u * u;  // ln(x)^2
        A(i, 1) = u;      // ln(x)
        A(i, 2) = 1.0;
    }
    for (i = 0; i < n; i++) b(i, 0) = y[i];

    Eigen::MatrixXd Q = A.colPivHouseholderQr().solve(b);
    for (i = 0; i < 3; i++) coff[i] = Q(i);

    return coff;
} // END of function curveFitLogQuadratic()

double * ExpNode::curveFitQuadratic(const std::vector<double>& x, const std::vector<double>& y)
{ 
    static double coff[3];  // Only need 3 coefficients for degree 2
    int i;
    int n = (int)x.size();
  
    // Create matrices for degree 2 polynomial: y = a*x^2 + b*x + c
    Eigen::MatrixXd A(n, 3);
    Eigen::MatrixXd b(n, 1);
   
    // Fill matrix A with [x^2, x, 1] for each point
    for (i = 0; i < n; i++)
    {
        A(i, 0) = x[i] * x[i];  // x^2
        A(i, 1) = x[i];         // x
        A(i, 2) = 1.0;          // constant term
    }
    
    // Fill vector b with y values
    for (i = 0; i < n; i++)
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