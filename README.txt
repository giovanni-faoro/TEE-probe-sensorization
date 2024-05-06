This is a repository for TEE probe sensorization. The code reported here is related to the IEEE TMRB paper "A Multi-sensorization Approach to Improve Safety in Transesophageal Echocardiography".

In the Sensor fusion folder, code for sensor data upsampling and import to ROS topics and finite state machine based EKF-IKF fusion is provided.

In the Sensor registration folder, code for the generalized Chardonnens' registration approach is provided. The main function is represented by Final Registration.m, while the other files contains support functions.