# aer_auto_exposure_gradient

## Description
This package is our implementation of Gradient based auto exposure by [Shim et al. 2018](https://ieeexplore.ieee.org/document/8379436). We have extended this algorithm to include rules for gain compensation and tested it on self driving car [Zeus.](https://www.autodrive.utoronto.ca/) We have subimitted our results to [17th Conference on Computer and Robot Vision](http://www.computerrobotvision.org/). This package interfaces with Blackfly S cameras (model number: BFSU3-51S5C-C) equipped with Sony IMX250 CMOS global shutter sensors with a 2448 by 2048 resolution and a 70.74 dB dynamic range.
## Installation 

This package was written for Black Fly series cameras. The setup guidlines for the camera drivers can be found [here](https://flir.app.boxcn.net/v/SpinnakerSDK) and in order to setup ros interface for the cameras use [this](https://github.com/ros-drivers/flir_camera_driver).
In order to install this package simply clone the repo and do `catkin build`. Note: This package was developed and tested on ROS Kinetic.

## Usage
In order to use the package follow these instructions:

1. Update the config file with the relevant parameters like the name of image topic, frame rate etc.
2. Launch the ros camera driver in order to get images from your camera.
3. Then use `roslaunch aer_auto_exposure_gradient exp_node.launch`.

## Video

Click on the image below for video.

[![](http://img.youtube.com/vi/vGS4-n6Pf30/0.jpg)](http://www.youtube.com/watch?v=vGS4-n6Pf30 "Video")

## Detailed algorithm description

The algorithm has two parts — a metric and an optimizer. The metric calculates the amount of gradient information for the current image at different gamma correction values. The metric is taken from the original repo, created by Shim et al. The optimizer adjusts the camera shutter speed, gain, and auxiliary illuminator power. The optimizer has been changed from Shim's to a simpler gradient ascend optimizer. The original Shim's optimizer can be enabled instead of the gradient ascend one, but it was not thoroughly tested, since the gradient ascend optimizer works well. The function used for fitting the effect of a simulated gamma correction value on the metric has also been changed from Shim's fifth-order polynomial to the more predictable log-quadratic function.

The optimizer output has also been modified and expanded. The optimizer now works internally in a normalized range with dimensionless quantities. It's value is then mapped onto three possible "actuators" — the camera's shutter speed (a.k.a. exposure time), camera's gain, and the desired output power of an auxiliary LED illuminator. This reflects the intended usage: visual-inertial odometry on a UAV in challenging lighting conditions, including complete darkness.

You will benfit the most from the algorithm implemented in this repository in high dynamic environments/scenes. The camera's builtin exposure control typically works on the basis of maintaining the average image brightness value close to the 127 (if we assume image values in the range 0-255). On the other hand, algorithm implemented in this repository does not care about image brightness. The core idea is this - if there is a large overexposed or underexposed area which provides no significant gradient information, the algorithm ignores it and keeps it overexposed or underexposed, while maintaining the brightness optimal for the other regions of the image with the most gradient information.

This is meant to improve the overall performance of all gradient-based computer vision algorithms. That means that it's computational footprint should be low. This algorithm typically consumes approximately 3% to 7% of the one core/thread on the typical PC/laptop. You can tune this algorithm to achieve best tradeoff mainly by changing metric computation loop frequency (```img_proc_loop_hz``` parameter) and number of gamma-corrected images produced (```gamma_num_points``` parameter). After changing metric computation loop frequency, also the optimizer loop frequency (```optimizer_loop_hz``` parameter) and gradient step size (```grad_k``` parameter) need to be changed.

### Metric

The algorithm uses image gamma correction as the approximation of how changing the camera's parameters affects image brightness and amount of gradient information in it. When we obtain the new image, we compute a few images wth different gamma correction - some dimmer and some brighter. For each gamma-corrected image, we compute the amount of gradient information by simple sum of the pixels after applying Sobel operator. For more information, see the paper Shim et. al. But unlike the Shim's paper, we do not use 5th order polynomial for gamma correction values fitting, but rather log-quadratic function. This function behaves better in our opinion and approximates the gamma function very well. See picture below.

![Effect of changing gamma correction value on the metric.](gamma_plot.png "Effect of changing gamma correction value on the metric.")

How many gamma-corrected images are used is configurable by the parameters ```gamma_num_points``` and ```gamma_range```. We found out that 5 points are enough, but this can be reduced to 3 if computational cost is critical. Using more than 5 points generally brings no advantage. Also the range of 2 is usually satisfactory, but it is possible that the range has to be tuned when the algorithm operates in the very high or very low dynamic environments freqently (more HDR or LDR than our testing environment).

If the user needs to make the image slightly more overexposed or underexposed, it is possible to set the param ```gamma_x_offset```. This offsets the optimum seen by the gradient descend algorithm. The slight disadvantage of the used algorithm is that the simulated gamma correction produces slightly under-exposed images - the true gradient information maximum is usually achieved for the slightly higher image brightness compared to what is given by this gamma-correction simulation. So if the user really wants the true maximum, this parameter can partially help to achieve that.

### Optimizer

The optimizer is just a simple gradient-based optimizer. There are two loops - the metric computation loop and the optimizer loop. Optimizer loop usually runs on a higher frequency than metric computation loop. This is advantageous for the two reasons. The first is that the the output of the algorithm should be close to continuous (sudden large changes in the image brightness can disturb visual algorithms frontends, like those of the VIO or VSLAM). Using gradient based optimizer acts as a kind of interpolation of the rough values provided by the metric computation loop. Second, the metric computation loop is computationally demanding, since it computes gradient information for the image using Sobel operator for several gamma-corrected images in one step (by default 5 images are used). Once we have the gamma function, we have an idea which direction should the camera parameters move, so we can improve image just by running optimizer for a while.

We found experimentally that running the optimizer loop at frequency 10-times higher than metric computation loop is sufficient. Of course, it depends on the gradient step size (```grad_k``` param). The good procedure is to set the optimizer loop to 10-times the frequency of the metric computation loop, then tune the gradient step size. For us, the metric computation loop at 2Hz and optimizer loop at 20Hz works well.

### Output actuators mapping.

Assume that, during the day, the camera is operating with some optimal shutter speed, a gain of 0dB, and the auxiliary illuminator off. When ambient light gets dimmer (dusk begins or the UAV enters an indoor environment), the image also gets dimmer and gradient information fades away. There are a few options: increase the camera's exposure time, increase the camera's gain, or enable the external LED illuminator.

The best first step is to increase the exposure time. Increasing gain is possible, but it also amplifies image noise. Turning on the illuminator is possible too, but it costs energy from the UAV's battery. However, exposure time cannot be increased indefinitely, because the UAV is moving and longer exposure time causes more motion blur. Therefore, an exposure time limit must be defined. As ambient light gets even dimmer and the limit is reached, either gain or illuminator power must be increased. The order of those activations is up to the user. The next output is not increased until the current one is saturated.

For example, with the following configuration:

```yaml
# --- Actuator slices: order defines priority (first = used first) ---
# Portions does not have to add up to the 100%.
'actuator_order': ['shutter', 'gain', 'led'],
'shutter_portion': 0.5,          # 50 % of [0,1] drives shutter
'shutter_max_us': 5000,           # shutter range: 0 – 5000 µs
'gain_portion': 0.5,             # 50 % of [0,1] drives gain
'gain_max': 12.0,                # gain range: 0 – 12 dB
'led_portion': 0.5,              # 50 % of [0,1] drives LED
'led_max': 40.0,                 # LED range: 0 – 40 W
```

and the initial exposure level:

```yaml
# --- Optimizer initial state ---
'initial_exposure_level': 0.1,   # normalized [0,1] starting point
```

When the algorithm starts, the exposure level is 0.1, which represents 1/5 of the shutter speed range — a shutter speed of 1000 µs. The algorithm will likely increase it from there. As ambient light gets dimmer, the 5000 µs limit is reached. Shutter speed is then held at that limit and gain begins to increase, up to its limit of 12 dB. If ambient light dims further, the only remaining option is to increase the illuminator power output. The user can change the order of outputs activation, but it is logical that camera exposure time is first.

The actuator portions do not have to add up to 1. The user is free to set the portions and max values as desired. However, it is good practice to define portions that sum to 1 for easier tuning. One important consideration when configuring outputs is gain matching. Each output — exposure time, gain, and illuminator — has a different effect on final image brightness, so portions and max values should be chosen such that they have similar effective gains. If this is not done, oscillations can occur when the algorithm transitions from one output to another. For example, if shutter_portion is 0.5 with shutter_max_us of 5000, but gain_portion is only 0.1 with gain_max of 12 dB, the gain output will almost certainly be too sensitive and will cause large oscillations when its portion is reached. This can be mitigated by lowering the gradient optimizer gain (grad_k parameter), but that would slow the response of all outputs. The best approach is to tune individual portions relative to each other together with the gradient optimizer gain. Currently, no automated calibration process is implemented for this tuning.

### External exposure time setting

When UAV is moving too fast or too close to the environment surface, it is possible that image gets too blurry to be used as input into any computer vision algorithm. Therefore, we need to lower the exposure time in those cases. On the other hand, if the UAV is moving in bigger height or slowly, we can increase exposure time and lower the amount of noise and auxiliary illuminator power needed. The topic name can be configured through ```shutter_limit_topic``` parameter and it's type is ```Int32```.

Note: In the example abowe, the sum of all outputs portions is 1.5, so the internal optimizer output will change between 0 and 1.5. If the exposure time is increased through the topic, let's say to the double of the default 5000 us, to the 10 000 us, it's portion changes to 1.0, so the optimizer internal output will move between 0 and 2.0.

Note: If your algorithm is sensitive to image brightness changes, you can preprocess your image using histogram equalization methods. They elliminate almost all of the possible flickering caused by setting camera parameters or illuminator power. So this algorithm only reduces the amount of noise in this case.