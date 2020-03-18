# Capstone-Project-Obstacle-Avoidance-Strategy
This is undergraduate project, which uses Opticalflow and Neuroevolution to obtain Obstacle Avoidance Strategy

It can be divided into 3 parts:
Obstacle detection and Tracking, Obstacle Prediction, Obstacle Avoidance Strategy

Obstacle detection and tracking: Optical Flow is used to compute the real-time changes between two frames in a video. And because after calibration, we have transformation matrix of the camera coordinate and real world coordinate, therefore, after 3D reconstruction and using depth information as adjustment, we can compute the movement of obstacles to the car.

Obstacle prediction: We use a simple neural network take the changes of obstacles and time as input, obtain prediction position of obstacles as output, and we can predict the position of obstacles.
Reason of obstacle prediction: Because the vehicle has a delay time due to hardware of it, therefore, we need predict the position of obstacles, and make the avoidance strategy in advance.

Obstacle Avoidance Strategy: We use neuralevolution method to do this part, and it obtains more than 94% after modification.

