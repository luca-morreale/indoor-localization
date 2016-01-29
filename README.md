# indoor-localization

Indoor localization project which aim is to localize a robot during its movements.

## Model
As base model we have used the one developed by [Marco Ciccone](https://github.com/MarcoCiccone/2D-tracking-EKF).  
Starting from that model our goal is to make it feasible for a real implementation, we will follow the below list:
* insert a little delay between each polling
* poll only the three nearer beacons
* poll only the three beacons which will be nearer exploiting the velocity

## Future improvements
As soon as the modelization phase will ends, the projects contemplate an implementation phase and 
a test phase to check the real feasibility of the project for daily use.
