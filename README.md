# indoor-localization

Indoor localization project which aim is to localize a robot during its movements in a mapped environment.

## Model
As base model we have used the one developed in Matlab by [Marco Ciccone](https://github.com/MarcoCiccone/2D-tracking-EKF).
His model use an Extended Kalman Filter (ekf) to track a robot, it is moving in a mapped environment.

Starting from his model we have tried to make it more realistic.

A delay between each poll has been introduced, it is equal to 400ms which is the same as the one of the real device we will test it in a real environment.

As further improvement, necessary due to the delay, the basestation interrogated are reduced to three.
These three stations are choosed exploiting the covariance matrix, which allows us to select those stations that will be closer in the next "poll session" using the information about velocity.

The model relies on measurements obtained using RSSI devices, the model used to calculate the values of the measurements has been obtained through real measurements. It could be changed and adapted to your needs.


## Implementation of the model
The model seems to be reliable so we have started the implementation. In the folder ```code``` we provide all the sources we will develop.

The basestations are based on a Raspberry Pi with Raspian installed on them.
The related code is developed in C++, which is a reasonable choice with constrained memory and speed.

We provide two executables:
* ```serial``` could be used to perform measurements.
* ```server``` replies to remote request for measurements.

The first one has been used to collect measurements and derive the realistic model used in the matlab model. The second one has been developed for a real application.

For what concern the so called Central-Station, we developed a ROS node.
In this case to speedup the developement, all the source are implemented in python.

A ROS-node has been created which tracks just one robot, or target. It behave as described in the matlab model with a substantial difference. Due to the unknown starting position of the target it will assumed to be the nearer basestation.

At each iteration the node will poll at most three basestation to collect their measurements, and estimate the position using EKF.
