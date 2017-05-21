### PID Controller Reflections

A PID Controller continuously calculates an error value as a difference between the desired value and the measured value and continuously applies a correction based on Proportional P, Derivative D, and Integral terms I. This controller can be used to ensure a self driving car follows the desired path.

#### Parameter Tuning Approach

The parameters used in this project were found using the following steps:
1. Experiment with various values for the Proportional value P and select the one that performs best.
2. Keep P value fixed, experiment with various values for the Derivative parameter D and select the one that performs best.
3. Keep P and D value, experiment with various values for the Integral parameter I and select the one that performs best.

The approach was extremely simple but it worked surprisingly well.

#### Finding P parameter
The P parameter controls responsiveness to an error. A lower value means it takes longer to react to an error while a larger value results in a faster response but can also result in unstable behavior.

Below are the values that were attempted for P along with general observations about their performance.
- 1.0 - becomes unstable relatively quickly and leaves the track
- 2.0 - becomes unstable even faster. This is expected
- 0.5 - went farther than 1.0 although it went over the ledge and recovered a few times.
- 0.25 - seems too small to effectively control CTE. Seems like 0.5 is better.
- 0.375 - this one was funny. Left the road, went back a bit, left the road again, then got stuck.
- 0.625 -left the road pretty fast.

The value selected based on these results was P = 0.5

The video below the result of running the algorithm with the following parameters P=1.0, I=0.0, D=0.0. P value is too high and the car leaves the track relatively quickly.

[![High P Value](https://img.youtube.com/vi/H3BwvtBZtcw/0.jpg)](https://youtu.be/H3BwvtBZtcw)

The video below the result of running the algorithm with the following parameters P=0.5, I=0.0, D=0.0. Lowering P value made the car go farther.

[![Optimized P Value](https://img.youtube.com/vi/C5VoHKQmkiU/0.jpg)](https://youtu.be/C5VoHKQmkiU)

#### Finding D parameter
Using only the P parameter results in the algorithm oscillating around the desired value.To control these oscillations I adjusted D parameter. This has the effect of damping the oscillations with higher values resulting in faster damping. Values that were too high resulted in shaky driving.

Below are the values that were attempted for D along with general observations about their performance.

- 1.0 - better that just P but still not very smooth and does not complete the track.
- 10 - awesome! It completed the track! Increasing D really helped and made driving pretty smooth.
- 100 - makes the car follow the middle of the track pretty closely but is not nice to watch because
the car is making small adjustments all the time and it makes the video look really shaky. I believe this value is too high.
- 20 - Completes the track a few times. Drives smooth. Better than 10.
- 40 - Completes the track a few times. Seems to be steering more than with 20.
- 30 - Maybe a little better than 20.

The values selected based on these results was D = 30.0

This video shows the result of running the algorithm with the following parameters P=0.5, I=0.0, D=1.0. Adding a derivative term makes the car drive smoother but it still leaves the track.

[![Low D Value](https://img.youtube.com/vi/sK6HrrhuSTw/0.jpg)](https://youtu.be/sK6HrrhuSTw)


This video shows the result of running the algorithm with the following parameters P=0.5, I=0.0, D=30.0. The car drives much smoother and it completes the track!

[![Low D Value](https://img.youtube.com/vi/oxP_w9-_VdM/0.jpg)](https://youtu.be/oxP_w9-_VdM)

### Finding I parameter
The I terms takes into account both the magnitude and the duration of the error. This term compensates for bias such as steering drift because of misaligned tires. Note that a large I value results in unstable behavior.

Below are the values that were attempted for I along with the general observations about their performance.

- 1.0 - not a good idea. Car left the track almost right away. Let's try something smaller.
- 0.1 - still way too big. Car left the track quickly and seems to want to go in circles.
- 0.01 - maybe still too big. Does not seem to help and I think is worse than with no I term.
- 0.001 - seems to be OK. Not sure if it helps in this case.

I could not decisively tell if a small I parameter helps at all so in order to confirm that I added a bias of 0.5 to the steering value. Doing this with I = 0.0 resulted in the car driving off center as shown in the video below. Parameters used were P=0.5, I=0.0 and D=30.0 with 0.5 steering bias.

[![Bias with no Integral Term](https://img.youtube.com/vi/nfoXeCtF18s/0.jpg)](https://youtu.be/nfoXeCtF18s)

Setting I to 0.001 resulted in the car driving in the center of the track again as shown in the video below. Parameters used were: P=0.5, I=0.001, D=30.0 and 0.5 steering bias.

[![Bias with Integral Term](https://img.youtube.com/vi/G0ZgdJOga1U/0.jpg)](https://youtu.be/G0ZgdJOga1U)

#### Changing throttle value
Just for fun I also experimented with increasing throttle values to see where this approach brakes down.

Here are the results of this experiments:
 - 0.3 - this value provided in the starter code. The car drives around the track successfully reaching max speed of 32 mph.
 - 0.5 - The car drives around the truck but it touches the red and white stripes in a couple of spots. It reaches a max speed of 48 mph.
 - 0.6 - Driving is more erratic than at 0.5 and still touches red and white stripes in a couple of spots. Reaches a max speed of 54 mph.
 - 0.7 - Driving is even more erratic than at 0.6. Reaches a max speed of 58 mph and went off track on second lap.
 - 0.8 - Driving faster is fun! Reaches max speed of 64 mph, touches the ledge, recovers and completes one lap. Goes off the track on the second lap.

Obviously we don't want our Self Driving Car to take sharp corners at high speed. To control this it makes sense to apply brakes (negative throttle) or to reduce acceleration when the error is above a certain threshold. I did not experiment with this but I would suspect such an approach would work.

#### Conclusion

I found manually tuning the PID parameters to be very helpful because it helped me develop intuition about how these parameters work. An obvious alternative would be to use an algorithm that finds these parameters automatically.

This was a very simple and fun project. Sometimes simplest things are the best or as Leonardo Da Vinci said "Simplicity is the ultimate sophistication".
