# aer_auto_exposure_gradient

## Description

This is a reimplemented and extended version of the gradient-based auto-exposure algorithm by [Shim et al. 2018](https://ieeexplore.ieee.org/document/8379436). The original implementation can be found [here](https://github.com/ishaanmht/aer_auto_exposure_gradient).

## Detailed Algorithm Description

The algorithm has two parts — a metric and an optimizer. The metric calculates the amount of gradient information in the current image at different gamma correction values. The metric is taken from the original paper by Shim et al. The optimizer adjusts the camera shutter speed, gain, and auxiliary illuminator power. The optimizer has been changed from Shim's to a simpler gradient ascent optimizer. The original Shim optimizer can be enabled in place of the gradient ascent one, but it was not thoroughly tested, since the gradient ascent optimizer performs well. The function used to fit the effect of a simulated gamma correction value on the metric has also been changed from Shim's fifth-order polynomial to the more predictable log-quadratic function.

The optimizer output has also been modified and expanded. The optimizer now works internally in a normalized range with dimensionless quantities. Its value is then mapped onto three possible "actuators" — the camera's shutter speed (a.k.a. exposure time), the camera's gain, and the desired output power of an auxiliary LED illuminator. This reflects the intended usage: visual-inertial odometry on a UAV in challenging lighting conditions, including complete darkness.

You will benefit the most from the algorithm implemented in this repository in high-dynamic-range environments/scenes. The camera's built-in exposure control typically works by maintaining the average image brightness value close to 127 (assuming image values in the range 0–255). The algorithm implemented in this repository, on the other hand, does not aim for a target image brightness. The core idea is this: if there is a large overexposed or underexposed area that provides no significant gradient information, the algorithm ignores it and keeps it overexposed or underexposed, while maintaining the brightness that is optimal for the regions of the image with the most gradient information.

This is intended to improve the overall performance of gradient-based computer vision algorithms, which means its computational footprint should be low. This algorithm typically consumes approximately 3%–7% of a single CPU core/thread on a typical PC or laptop. You can tune the algorithm to achieve the best tradeoff mainly by changing the metric computation loop frequency (```img_proc_loop_hz``` parameter) and the number of gamma-corrected images produced (```gamma_num_points``` parameter). After changing the metric computation loop frequency, the optimizer loop frequency (```optimizer_loop_hz``` parameter) and gradient step size (```grad_k``` parameter) should also be adjusted.

### Metric

The algorithm uses image gamma correction as an approximation of how changing the camera's parameters affects image brightness and the amount of gradient information in it. When a new image is obtained, a few images with different gamma corrections are computed — some dimmer and some brighter. For each gamma-corrected image, the amount of gradient information is computed as a simple sum of pixel values after applying the Sobel operator. For more information, see Shim et al. Unlike the Shim paper, however, we do not use a 5th-order polynomial for fitting the gamma correction values, but rather a log-quadratic function. This function behaves better in our experience and approximates the gamma curve very well. See the figure below.

![Effect of changing gamma correction value on the metric.](gamma_plot.png "Effect of changing gamma correction value on the metric.")

The number of gamma-corrected images used is configurable via the ```gamma_num_points``` and ```gamma_range``` parameters. We found that 5 points are sufficient, but this can be reduced to 3 if computational cost is critical. Using more than 5 points generally brings no advantage. A range of 2 is usually satisfactory, but it may need to be tuned when the algorithm frequently operates in very high or very low dynamic range environments (more HDR or LDR than our testing environment).

If the user needs to make the image slightly more overexposed or underexposed, the ```gamma_x_offset``` parameter can be set. This offsets the optimum seen by the gradient descent algorithm. A minor drawback of the algorithm is that the simulated gamma correction tends to produce slightly underexposed images — the true gradient information maximum is usually achieved at a slightly higher image brightness than what the gamma correction simulation suggests. This parameter can partially compensate for that if the user wants to target the true maximum.

### Optimizer

The optimizer is a simple gradient-based optimizer. There are two loops — the metric computation loop and the optimizer loop. The optimizer loop typically runs at a higher frequency than the metric computation loop. This is advantageous for two reasons. First, the algorithm's output should be close to continuous (sudden large changes in image brightness can disturb the front-ends of visual algorithms, such as those used in VIO or VSLAM). Using a gradient-based optimizer acts as a kind of interpolation of the coarse values provided by the metric computation loop. Second, the metric computation loop is computationally demanding, as it computes gradient information for the image using the Sobel operator over several gamma-corrected images in one step (by default, 5 images). Once the gamma function is known, we have an indication of which direction the camera parameters should move, so we can continue improving the image by running the optimizer alone for a while.

We found experimentally that running the optimizer loop at 10 times the frequency of the metric computation loop is sufficient. Of course, this depends on the gradient step size (```grad_k``` parameter). A good procedure is to set the optimizer loop to 10× the frequency of the metric computation loop, then tune the gradient step size. For our use case, a metric computation loop at 2 Hz and an optimizer loop at 20 Hz works well.

### Output Actuator Mapping

Assume that, during the day, the camera is operating at some optimal shutter speed, a gain of 0 dB, and with the auxiliary illuminator off. When ambient light decreases (at dusk or when the UAV moves indoors), the image gets dimmer and gradient information fades. There are a few options: increase the camera's exposure time, increase the camera's gain, or enable the external LED illuminator.

The best first step is to increase the exposure time. Increasing gain is possible, but it also amplifies image noise. Turning on the illuminator is an option too, but it draws energy from the UAV's battery. However, exposure time cannot be increased indefinitely, because the UAV is moving and longer exposure causes more motion blur. Therefore, an exposure time limit must be defined. As ambient light decreases further and that limit is reached, either gain or illuminator power must be increased. The order in which these are activated is up to the user. The next output is not increased until the current one is saturated.

For example, with the following configuration:

```yaml
# --- Actuator slices: order defines priority (first = used first) ---
# Portions do not have to add up to 100%.
'actuator_order': ['shutter', 'gain', 'led'],
'shutter_portion': 0.5,          # 50% of [0,1] drives shutter
'shutter_max_us': 5000,           # shutter range: 0–5000 µs
'gain_portion': 0.5,             # 50% of [0,1] drives gain
'gain_max': 12.0,                # gain range: 0–12 dB
'led_portion': 0.5,              # 50% of [0,1] drives LED
'led_max': 40.0,                 # LED range: 0–40 W
```

and the initial exposure level:

```yaml
# --- Optimizer initial state ---
'initial_exposure_level': 0.1,   # normalized [0,1] starting point
```

When the algorithm starts, the exposure level is 0.1, which represents 1/5 of the shutter speed range — a shutter speed of 1000 µs. The algorithm will likely increase it from there. As ambient light decreases, the 5000 µs limit is reached. Shutter speed is then held at that limit and gain begins to increase, up to its limit of 12 dB. If ambient light decreases further, the only remaining option is to increase the illuminator power output. The user can change the order of output activation, but it is logical to start with camera exposure time.

The actuator portions do not have to add up to 1. The user is free to set portions and max values as desired. However, it is good practice to define portions that sum to 1 for easier tuning. One important consideration when configuring outputs is gain matching. Each output — exposure time, gain, and illuminator — has a different effect on the final image brightness, so portions and max values should be chosen such that they have similar effective gains. If this is not done, oscillations can occur when the algorithm transitions from one output to another. For example, if `shutter_portion` is 0.5 with `shutter_max_us` of 5000, but `gain_portion` is only 0.1 with `gain_max` of 12 dB, the gain output will almost certainly be too sensitive and will cause large oscillations when its portion is reached. This can be mitigated by lowering the gradient optimizer gain (`grad_k` parameter), but that would slow the response of all outputs. The best approach is to tune individual portions relative to each other together with the gradient optimizer gain. No automated calibration process is currently implemented for this tuning.

### External Exposure Time Limit

When a UAV is moving too fast or too close to a surface, images can become too blurry to be useful as input to any computer vision algorithm. In those cases, the exposure time limit should be reduced. Conversely, if the UAV is moving at a greater height or slowly, the exposure time limit can be increased, reducing the amount of noise and the auxiliary illuminator power required. The topic name is configurable via the ```shutter_limit_topic``` parameter; the message type is ```Int32```.

Note: In the example above, the sum of all output portions is 1.5, so the internal optimizer output will range between 0 and 1.5. If the exposure time limit is increased via the topic — say, doubled from the default 5000 µs to 10 000 µs — its portion changes to 1.0, so the optimizer's internal output will range between 0 and 2.0.

Note: If your algorithm is sensitive to image brightness changes, you can preprocess your images using histogram equalization. This eliminates almost all flickering caused by changes in camera parameters or illuminator power, so this algorithm only reduces noise in that case.

## Possible Future Improvements

This is only the most basic implementation, using a simple metric — the sum of 
gradients. There is room for improvement. Possible directions:

1. Adding a percentile-based metric instead of a simple gradient sum, as described in [Zhang et al. 2017](https://ieeexplore.ieee.org/document/7989449). This makes the metric robust to noise by focusing on the mid-to-high gradient pixels, rather than being skewed by the large number of near-zero gradient pixels in flat or smooth regions.

2. Adding entropy weighting of the gradients, as described in [Kim et al. 2018](https://ieeexplore.ieee.org/document/8462881). This would make the saturation handling more principled — rather than the current approach of simply ignoring saturated regions, the metric would explicitly penalize them, preventing strong gradients near partially saturated areas from distorting the optimal exposure estimate.

3. Replacing the log-quadratic gamma simulation with a photometric response function calibrated to the specific camera, as described in [Zhang et al. 2017](https://ieeexplore.ieee.org/document/7989449). This would produce a more accurate prediction of how changing the exposure time affects the metric, potentially improving convergence speed and accuracy of the optimizer, especially in high dynamic range environments.

4. Automatic calibration of the output actuator gains, to avoid oscillations when the algorithm transitions between actuators (e.g. from shutter speed to gain, or from gain to LED illuminator). Currently, the portions and max values must be manually tuned to ensure similar effective gains across actuators. An automated calibration process could estimate these by briefly sweeping each actuator and measuring the effect on image brightness.

5. Automatic linking of the optimizer loop frequency and gradient step size to the metric computation loop frequency. Currently, optimizer_loop_hz and grad_k must be manually retuned whenever img_proc_loop_hz is changed. These could be automatically scaled relative to the metric computation frequency to reduce the manual tuning burden.