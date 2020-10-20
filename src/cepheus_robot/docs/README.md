
# Functions in files

cepheus_arms_operations.cpp
---------------------------
```cpp
double
produce_trajectory_point(double time,
			 double movement_duration,
			 double init_pos,
			 double set_point);

double
produce_trajectory_point_wrist (double time,
				double movement_duration,
				double init_pos,
				double set_point);

double
produce_sin_trajectory (double width,
			double period,
			double t);

double
produce_sin_trajectory_wrist   (double width,
				double period,
				double t);

void
moveLeftArmSin	(controller_manager::ControllerManager& cm,
		CepheusHW& robot,
		ros::Publisher right_shoulder_pub,
		ros::Publisher right_elbow_pub);

void
move_left_arm  (double set_point_shoulder,
		double set_point_elbow,
		double set_point_wrist,
		double movement_duration,
		controller_manager::ControllerManager& cm,
		CepheusHW& robot,
		ros::Publisher left_shoulder_pub,
		ros::Publisher left_elbow_pub);

void
move_right_arm (double set_point_shoulder,
		double set_point_elbow,
		double set_point_wrist,
		double movement_duration,
		controller_manager::ControllerManager& cm,
		CepheusHW& robot,
		ros::Publisher right_shoulder_pub,
		ros::Publisher right_elbow_pub);
```

cepheus_init_ctrls.cpp
---------------------

```cpp
void
start_standard_controllers     (ros::NodeHandle& nh,
				controller_manager::ControllerManager& cm,
				ros::Rate& loop_rate);

void
init_left_arm_and_start_controllers    (ros::NodeHandle& nh,
					controller_manager::ControllerManager& cm,
					CepheusHW& robot,
					ros::Publisher left_shoulder_pub,
					ros::Publisher left_elbow_pub,
					ros::Rate& loop_rate);

void
init_right_arm_and_start_controllers   (ros::NodeHandle& nh,
					controller_manager::ControllerManager& cm,
					CepheusHW& robot,
					ros::Publisher right_shoulder_pub,
					ros::Publisher right_elbow_pub,
					ros::Rate& loop_rate);
```

cepheus_interface.cpp (node)
---------------------


#### Callback functions:
```cpp
// cmd_thrust topic
void
thrusterCallback(const geometry_msgs::Vector3Stamped::ConstPtr& cmd);

// cmd_torque topic
void
torqueCallback(const std_msgs::Float64::ConstPtr& cmd);

// left_fsr topic
void
leftFsrCallback(const std_msgs::UInt8::ConstPtr& cmd);

// right_fsr topic
void
rightFsrCallback(const std_msgs::UInt8::ConstPtr& cmd);

// left_wrist_cmd topic
void
leftWristCallback(const std_msgs::Float64::ConstPtr& cmd);

// left_gripper_cmd topic
void
leftGripperCallback(const std_msgs::Float64::ConstPtr& cmd);

// left_gripper_action topic
void
leftGripperActionCallback(const std_msgs::Bool::ConstPtr& cmd);

// right_gripper_action topic
void
rightGripperActionCallback(const std_msgs::Bool::ConstPtr& cmd);
```
```cpp
// SIGINT handler
void ctrl_C_Handler(int sig);

// Trend line transforming force to d_theta for gripper
double fsr_trend_line(bool positive ,double pi_out);

// PI controller for left gripper (force controll)
void left_fsr_update();
```

cepheus_ctrl.cpp (node)
---------------------

```cpp
bool
reloadControllerLibraries(ros::NodeHandle &n);

bool
loadController(ros::NodeHandle &n, std::string c_name);

bool
startControllers(ros::NodeHandle &n,
		 std::vector<std::string>& controllers_to_start);

// res: nothing to return because failure stops 'ctrl' and interface hangs
void
startCtrl (const std_msgs::StringConstPtr &msg,
	   ros::NodeHandle &n, ros::Publisher &ctl_pub);
```
