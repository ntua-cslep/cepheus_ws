#include <dm7820_library.h>
#include "cepheus_hardware.h"

#define TORQUE_REDUCE 1.0

#define SH_MIN_FRICT 0.156
#define SH_SMALL_FRICT 0.02
#define ELB_MIN_FRICT 0.02


void print_binary(uint16_t x)
{
	for (int i = 0; i < 16; i++) {
		printf("%d ", (x & 0x8000) >> 15);
		x <<= 1;
	}
	printf("\n");
}

void CepheusHW::setThrustPwm(double *thrust, double min_duty, double max_duty)
{
	double duty[4];
	uint16_t dir[4];
	uint16_t port_2;

	for(int i=0; i<4; i++)
	{
		// Calculate the correct duty cycle
		// and the pin to output the pulse
		if(thrust[i] >= 0)
		{
			if (thrust[i] < max_thrust)
				duty[i] = thrust[i]/max_thrust;
			else
				duty[i] = 1;
			//select the non-inverted pin
			dir[i] = 1;
		}
		else
		{
			//0 is  the max duty for inverted
			if (thrust[i] > -max_thrust)
				duty[i] = 1 + (thrust[i]/max_thrust);
			else
				duty[i] = 0;
			//select the inverted pin
			dir[i] = 2;
		}

		//ensure that PWM is inside the limit 0-1 and has deadband areas removed
		if (dir[i] == 1)
		{
			if (duty[i] > max_duty)
				duty[i]=1;
			else if (duty[i] < min_duty)
				duty[i]=0;
		}
		else if (dir[i] == 2)
		{
			if ( (1 - duty[i]) > max_duty )
				duty[i]=0;
			else if ( (1 - duty[i]) < min_duty )
				duty[i]=1;
		}
		else
			ROS_INFO("PWM thrust calculation error");
	}

	//Seting which pin of the inverted and non inverted pin of a PWM will be used
	uint16_t output = dir[0]<<0 | dir[1]<<2 | dir[2]<<4 | dir[3]<<6;

	// dm7820_status = DM7820_StdIO_Get_Input(board, DM7820_STDIO_PORT_2, &port_2);
	// DM7820_Return_Status(dm7820_status, "DM7820_StdIO_get_port_state");
	// ROS_INFO("port 2: %x", port_2);
	//dm7820_status = DM7820_StdIO_Set_Output(board, DM7820_STDIO_PORT_2, 0);
	//DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Output()");

	// Set port's 2 // bits to peripheral output
	dm7820_status = DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_2, (output<<8), DM7820_STDIO_MODE_PER_OUT);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");
	//Set port's 2 bits to PWM output pulse width modulator peripheral
	dm7820_status = DM7820_StdIO_Set_Periph_Mode(board, DM7820_STDIO_PORT_2, (output<<8), DM7820_STDIO_PERIPH_PWM);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Periph_Mode()");

	//Reset port's 2 bits to STD IO bits in order not to act like an PWM
	dm7820_status = DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_2, ((~output)<<8), DM7820_STDIO_MODE_OUTPUT);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");


	dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_A, (uint32_t)(duty[0]*period) );
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
	dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_B, (uint32_t)(duty[1]*period) );
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
	dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_C, (uint32_t)(duty[2]*period) );
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
	dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_D, (uint32_t)(duty[3]*period) );
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
}

void CepheusHW::setHomePos(int i, float val)
{
	home_pos[i] = val;
}

/*void CepheusHW::setJointTorque(int sh,  int elb)
{
	cmd[5] = elb * 0.010/TORQUE_REDUCE;
	//cmd[4] = sh * 0.0020/TORQUE_REDUCE;
	cmd[4] = sh * 0.00015/TORQUE_REDUCE;

	fflush(stdout);
	//ROS_WARN("shoulder: %f, elbow: %f\n",cmd[4],cmd[5]);
}*/

void CepheusHW::setJointTorque(int sh,  int elb)
{
	if (!elb) cmd[4] =  sh *  SH_SMALL_FRICT;
	else cmd[4] = -elb * 0.005;
	cmd[5] =  elb * ELB_MIN_FRICT;
}

//------------ For new init -------------------------
//double e_sum = 0.0;
void CepheusHW::update_shoulder(double shoulder_rate, double des, double &shoulder_torque)
{

	// was 0.00158
	double kp = 0.0158;
	//double ki = 0.000001;

	double e = (des-shoulder_rate);
	//e_sum += e;
	//ROS_WARN("e: %lf, des: %lf, sh_rate(vel): %lf", e, des, shoulder_rate);

	shoulder_torque = kp*e;
}


void CepheusHW::update_elbow(double elbow_rate, double des, double &elbow_torque)
{
	double kp = 0.001;

	double e = (des-elbow_rate);
	elbow_torque=kp*e;
}

// 2020 Pelekoudas
void CepheusHW::update_left_elbow(double elbow_rate, double des, double &elbow_torque, double kp) {
        // double kp = 0.5;

        double e = (des-elbow_rate);
        elbow_torque=kp*e;
}


void CepheusHW::update_right_elbow(double elbow_rate, double des, double &elbow_torque, double kp) {
	// double kp = 1.0;

	double e = (des-elbow_rate);
	elbow_torque=kp*e;
}

//\\-----------------------------------------//\\

//-\\--- New init with velocity controll ---//-\\

//the velocity function is like this:/
//t2 is the duration of the accelaration
double velocity_for_joint_init(double t2, double t, bool positive) {

	//7 degrees per second
	//4->0.0.5 2020
	double vel_max_pos = (double)0.05/(double)180 * (double)M_PI;
	double vel_max_neg = (double)0.05/(double)180 * (double)M_PI;
	double vel_max = 0.0;

	if (positive)
		vel_max = vel_max_pos;
	else
		vel_max = vel_max_neg;

	double vel_des = 0.0;
	if (t <= t2)
		vel_des = vel_max/t2 * t;
	else
		vel_des = vel_max;

	// ROS_WARN("vel_des : %lf", vel_des);
	if (positive)
		return vel_des;
	else
		return -vel_des;
}

//------------------------------------------//

void CepheusHW::init_left_shoulder() {

	double shoulder_out, right_elbow_out;
	double des_shoulder, des_right_elbow;

	if(home_pos[LEFT_SHOULDER]) {

		ROS_INFO_STREAM("homing LEFT SHOULDER................");

		ros::Time init_time = ros::Time::now();
		ros::Duration timer;

		timer = ros::Time::now() - init_time;
		//ros::Duration prev_timer = timer;
		do {
			des_shoulder = velocity_for_joint_init(1, (double)timer.toSec(), false);
			des_right_elbow = velocity_for_joint_init(10, (double)timer.toSec(), false);

			update_shoulder(vel[LEFT_SHOULDER], des_shoulder, shoulder_out);
			//ROS_WARN("shoulder_out: %lf", shoulder_out);
			//update_right_elbow(vel[RIGHT_ELBOW], des_right_elbow, right_elbow_out, 0.01);
			cmd[RIGHT_ELBOW] = 0.0000001;//right_elbow_out;
			cmd[LEFT_SHOULDER] = shoulder_out;
			cmd[LEFT_ELBOW] = -0.000001;
			//ROS_WARN("LS cmd: %lf", cmd[LEFT_SHOULDER]);
			writeMotors();

			heartbeat();
			//readLimitSwitches();
			//ROS_WARN("DT: %lf", timer);
			//timer = ros::Time::now() - init_time;
			readEncoders(timer);
			timer = ros::Time::now() - init_time;
			//prev_timer = timer;
		} while(!isLimitReached(LEFT_SHOULDER));
		ROS_INFO_STREAM("homing of LEFT SHOULDER  succesful");
		offset_pos[LEFT_SHOULDER] = home_pos[LEFT_SHOULDER] + pos[LEFT_SHOULDER];
		ROS_WARN(">>> LS OFFSET: %lf", offset_pos[LEFT_SHOULDER]);
	}
	else
		ROS_WARN_STREAM("No homing performed for LEFT SHOULDER because no home position setted");
}



void CepheusHW::init_right_shoulder() {

	double shoulder_out;
	double des_shoulder;

	if (home_pos[RIGHT_SHOULDER] < 0) {

		ROS_INFO_STREAM("homing RIGHT SHOULDER................");

		ros::Time init_time = ros::Time::now();
		ros::Duration timer;

		timer = ros::Time::now() - init_time;
		do {
			des_shoulder = velocity_for_joint_init(30, (double)timer.toSec(), false);

			update_shoulder(vel[RIGHT_SHOULDER], des_shoulder, shoulder_out);
			cmd[RIGHT_SHOULDER] = shoulder_out;

			writeMotors();

			heartbeat();
			readLimitSwitches();
			readEncoders(timer);
			timer = ros::Time::now() - init_time;
		}
		while(!isLimitReached(RIGHT_SHOULDER));

		ROS_INFO_STREAM("homing of RIGHT SHOULDER  succesful");
		offset_pos[RIGHT_SHOULDER] = home_pos[RIGHT_SHOULDER] + pos[RIGHT_SHOULDER];

		ROS_WARN("offset %lf", offset_pos[RIGHT_SHOULDER]);
	}
	else
		ROS_WARN_STREAM("No homing performed for RIGHT SHOULDER because no home position setted");
	//offset_pos[RIGHT_SHOULDER] = 0.0;
}


// 2020 checked Joint 2
void CepheusHW::init_left_elbow() {
	//Homing left elbow moving left shoulder
	//and takina advantage of the movement transmission of the left arm

	double shoulder_out,elbow_out;
	double des_shoulder, des_elbow;

	if (home_pos[LEFT_ELBOW] > 0) {

		ROS_INFO_STREAM("homing LEFT ELBOW..............");


		ros::Time init_time = ros::Time::now();
		ros::Duration timer = ros::Time::now() - init_time;

		//e_sum = 0.0;
		do {

			//des_shoulder = velocity_for_joint_init(10, (double)timer.toSec(), false);
			des_elbow = velocity_for_joint_init(10, (double)timer.toSec(), true);

			//update_shoulder(vel[LEFT_SHOULDER], des_shoulder, shoulder_out);
			//cmd[LEFT_SHOULDER] = shoulder_out;
			update_left_elbow(vel[LEFT_ELBOW], des_elbow, elbow_out, 1);
			cmd[LEFT_ELBOW] = elbow_out;

			//ROS_WARN("cmd4 : %lf" , cmd[4]);
			writeMotors();

			heartbeat();

			//readLimitSwitches();
			readEncoders(timer);
			timer = ros::Time::now() - init_time;

		} while(!isLimitReached(LEFT_ELBOW));

		ROS_INFO_STREAM("homing of LEFT ELBOW  succesful");
		offset_pos[LEFT_ELBOW] = home_pos[LEFT_ELBOW] - pos[LEFT_ELBOW];
	}
	else
		ROS_WARN_STREAM("No homing performed for LEFT ELBOW because no home position setted");
}


void CepheusHW::init_right_elbow() {
	//Homing right elbow moving left shoulder
	//and taking advantage of the movement transmission of the right arm

	double shoulder_out, right_elbow_out, left_elbow_out;
	double des_shoulder, des_right_elbow, des_left_elbow;

	if (home_pos[RIGHT_ELBOW]) {

		ROS_INFO_STREAM("homing RIGHT ELBOW..............");

		ros::Time init_time = ros::Time::now();
		ros::Duration timer;
		//e_sum = 0.0;
		do{
			//2020 true right elbow
			//des_shoulder = velocity_for_joint_init(10, (double)timer.toSec(), true);
			des_right_elbow = velocity_for_joint_init(3, (double)timer.toSec(), false);
			des_left_elbow = velocity_for_joint_init(10, (double)timer.toSec(), false);
			//update_shoulder(vel[RIGHT_SHOULDER], des_shoulder, shoulder_out);
			//cmd[RIGHT_SHOULDER] = shoulder_out;
			update_right_elbow(vel[RIGHT_ELBOW], des_right_elbow, right_elbow_out, 1.0);
			cmd[RIGHT_ELBOW] = right_elbow_out;
			// 2020
			update_left_elbow(vel[LEFT_ELBOW], des_left_elbow, left_elbow_out, 0.5);
			cmd[LEFT_ELBOW] = left_elbow_out;
			//update_shoulder(vel[LEFT_SHOULDER], 0.0, shoulder_out);
			//cmd[LEFT_SHOULDER] = shoulder_out;
			//update_shoulder(vel[LEFT_ELBOW], 0.0, shoulder_out);
			//cmd[LEFT_ELBOW] = shoulder_out;
			//ROS_WARN("cmd7 : %lf" , cmd[RIGHT_ELBOW]);
			writeMotors();

			heartbeat();
			//readLimitSwitches();
			readEncoders(timer);
			timer = ros::Time::now() - init_time;

		} while(!isLimitReached(RIGHT_ELBOW));


		ROS_INFO_STREAM("homing of RIGHT ELBOW  succesful");
		offset_pos[RIGHT_ELBOW] = home_pos[RIGHT_ELBOW] + pos[RIGHT_ELBOW];
		ROS_WARN("offset %lf", offset_pos[RIGHT_ELBOW]);
	}
	else
		ROS_WARN_STREAM("No homing performed for RIGHT ELBOW because no home position setted");

	//offset_pos[RIGHT_ELBOW] = 0.0;
}


//------------------------------

void CepheusHW::write_left_wrist(double width){

	dm7820_status = DM7820_PWM_Set_Width(manipulator_board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_A,  width);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

}

void CepheusHW::write_left_finger(double width){

	dm7820_status = DM7820_PWM_Set_Width(manipulator_board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_B,  width);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
}

void CepheusHW::write_right_wrist(double width){

	dm7820_status = DM7820_PWM_Set_Width(manipulator_board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_C,  width);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

}

void CepheusHW::write_right_finger(double width){

	dm7820_status = DM7820_PWM_Set_Width(manipulator_board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_D,  width);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
}


//The gripper opens full
void CepheusHW::set_left_finger(double c){

	ros::Rate loop_rate(200);

	for (int i = LEFT_FINGER_MAX_ANGLE; i >= LEFT_FINGER_MIN_ANGLE; i--) {

		cmd[LEFT_GRIPPER] = (double)i;
		double div = (double)cmd[LEFT_GRIPPER]/(double)LEFT_FINGER_MAX_ANGLE;
		width[LEFT_GRIPPER] = (uint16_t)(div*PWM_FINGER_SERVO_RANGE + PWM_FINGER_SERVO_MIN_DT);

		write_left_finger(width[LEFT_GRIPPER]);

		loop_rate.sleep();
	}
}


//The gripper opens full
void CepheusHW::set_left_wrist(double c){

	ros::Rate loop_rate(200);

	for (int i = LEFT_WRIST_INIT_ANGLE; i <= LEFT_WRIST_MAX_ANGLE; i++) {

		cmd[LEFT_WRIST] = (double)i;
		double div = (double)cmd[LEFT_WRIST]/(double)LEFT_WRIST_MAX_ANGLE;
		width[LEFT_WRIST] = (uint16_t)(div*PWM_WRIST_SERVO_RANGE + PWM_WRIST_SERVO_MIN_DT);

		write_left_wrist(width[LEFT_WRIST]);

		loop_rate.sleep();
	}
}


void CepheusHW::command_right_wrist() {

	double a1,a2;
	while(1) {

		scanf("%lf %lf",&a1,&a2);

		if(a1 < a2){
			ros::Rate loop_rate(200);

			for (int i = a1; i <= a2; i++) {
				cmd[RIGHT_WRIST] = (double)i;
				double div = (double)cmd[RIGHT_WRIST]/(double)RIGHT_WRIST_MAX_ANGLE;
				width[RIGHT_WRIST] = (uint16_t)(div*PWM_WRIST_SERVO_RANGE + PWM_WRIST_SERVO_MIN_DT);

				write_right_wrist(width[RIGHT_WRIST]);
				loop_rate.sleep();
			}

		}
		else{
			ros::Rate loop_rate(200);

			for (int i = a1; i >= a2; i--) {
				cmd[RIGHT_WRIST] = (double)i;
				double div = (double)cmd[RIGHT_WRIST]/(double)RIGHT_WRIST_MAX_ANGLE;
				width[RIGHT_WRIST] = (uint16_t)(div*PWM_WRIST_SERVO_RANGE + PWM_WRIST_SERVO_MIN_DT);

				write_right_wrist(width[RIGHT_WRIST]);
				loop_rate.sleep();
			}
		}
	}
}

//The gripper opens full
void CepheusHW::init_left_finger() {

	ros::Rate loop_rate(200);

	for (int i = LEFT_FINGER_INIT_ANGLE; i <= LEFT_FINGER_MAX_ANGLE; i++) {

		cmd[LEFT_GRIPPER] = (double)i;
		double div = (double)cmd[LEFT_GRIPPER]/(double)LEFT_FINGER_MAX_ANGLE;
		width[LEFT_GRIPPER] = (uint16_t)(div*PWM_FINGER_SERVO_RANGE + PWM_FINGER_SERVO_MIN_DT);

		write_left_finger(width[LEFT_GRIPPER]);
		loop_rate.sleep();
	}
}

//The left wrist in middle position
void CepheusHW::init_left_wrist() {

	ros::Rate loop_rate(200);

	for(int i = LEFT_WRIST_MIN_ANGLE; i <= LEFT_WRIST_INIT_ANGLE; i++){

		cmd[LEFT_WRIST] = (double)i;
		double div = (double)cmd[LEFT_WRIST]/(double)LEFT_WRIST_MAX_ANGLE;
		width[LEFT_WRIST] = (uint16_t)(div*PWM_WRIST_SERVO_RANGE + PWM_WRIST_SERVO_MIN_DT);

		write_left_wrist(width[LEFT_WRIST]);
		loop_rate.sleep();
	}
}


//The gripper opens full
void CepheusHW::init_right_finger() {

	ros::Rate loop_rate(200);

	for (int i = RIGHT_FINGER_INIT_ANGLE; i <= RIGHT_FINGER_MAX_ANGLE; i++) {

		cmd[RIGHT_GRIPPER] = (double)i;
		double div = (double)cmd[RIGHT_GRIPPER]/(double)RIGHT_FINGER_MAX_ANGLE;
		width[RIGHT_GRIPPER] = (uint16_t)(div*PWM_FINGER_SERVO_RANGE + PWM_FINGER_SERVO_MIN_DT);

		write_right_finger(width[RIGHT_GRIPPER]);
		loop_rate.sleep();
	}
}

//The right wrist in middle position
void CepheusHW::init_right_wrist() {

	ros::Rate loop_rate(200);

	for (int i = RIGHT_WRIST_MIN_ANGLE; i <= RIGHT_WRIST_INIT_ANGLE; i++) {

		cmd[RIGHT_WRIST] = (double)i;
		double div = (double)cmd[RIGHT_WRIST]/(double)RIGHT_WRIST_MAX_ANGLE;
		width[RIGHT_WRIST] = (uint16_t)(div*PWM_WRIST_SERVO_RANGE + PWM_WRIST_SERVO_MIN_DT);

		write_right_wrist(width[RIGHT_WRIST]);
		loop_rate.sleep();
	}
}

void CepheusHW::set_right_finger(double c) {

	ros::Rate loop_rate(200);

	for(int i = RIGHT_FINGER_MAX_ANGLE; i >= RIGHT_FINGER_MIN_ANGLE; i--) {

		cmd[RIGHT_GRIPPER] = (double)i;
		double div = (double)cmd[RIGHT_GRIPPER]/(double)RIGHT_FINGER_MAX_ANGLE;
		width[RIGHT_GRIPPER] = (uint16_t)(div*PWM_FINGER_SERVO_RANGE + PWM_FINGER_SERVO_MIN_DT);

		write_right_finger(width[RIGHT_GRIPPER]);
		loop_rate.sleep();
	}
}

//The right wrist in middle position
void CepheusHW::set_right_wrist(double c) {

	ros::Rate loop_rate(200);

	for(int i = RIGHT_WRIST_MIN_ANGLE; i <= RIGHT_WRIST_MAX_ANGLE; i++) {

		cmd[RIGHT_WRIST] = (double)i;
		double div = (double)cmd[RIGHT_WRIST]/(double)RIGHT_WRIST_MAX_ANGLE;
		width[RIGHT_WRIST] = (uint16_t)(div*PWM_WRIST_SERVO_RANGE + PWM_WRIST_SERVO_MIN_DT);

		write_right_wrist(width[RIGHT_WRIST]);
		loop_rate.sleep();
	}
}



void CepheusHW::writeMotors()
{
	//ROS_INFO(" PWM_M_R  %d PWM_M_MIN %d\n",PWM_MOTOR_RANGE,PWM_MOTOR_MIN_DT);

	//REACTION WHEEL
	current[0] = (cmd[0]/0.0538);//cmd is in Nm
	eff[0] = cmd[0];
	//saturate to max current
	if (current[0] >= max_current[rw]) {
		current[0] = max_current[0];
		eff[0] = current[0]*0.0538;//eff is in Nm
	}
	else if (current[0] <=-max_current[0]) {
		current[0] =-max_current[0];
		eff[0] = current[0]*0.0538;//eff is in Nm
	}

	if (current[0] >= 0.0) 
	{
		dir[0] = 0;
		width[0] = (uint16_t)(current[0]*(ADC_PWM_PERIOD_COUNTS/max_current[0]));
	}
	else
	{
		dir[0] = 1;
		width[0] = (uint16_t)(-current[0]*(ADC_PWM_PERIOD_COUNTS/max_current[0]));
	}


	//MANIPULATOR MOTORS
	for (int i=4; i<8; i++)
	{

		// the stator is attached to the rotating part of joint
		// if (i == RIGHT_ELBOW && offset_pos[RIGHT_ELBOW]) {
		// 	cmd[i] = - cmd[i];
		// 	ROS_WARN("cmd[RIGHT_ELBOW]: %lf", cmd[i]);
		// }

		// if (i == LEFT_SHOULDER && offset_pos[LEFT_SHOULDER]) {
		// 	cmd[i] = - cmd[i];
		// 	ROS_WARN("cmd[LEFT_SHOULDER]: %lf", cmd[i]);
		// }


		current[i] = (cmd[i]/0.0452);

		/*if(i==RIGHT_ELBOW) {
			ROS_WARN("cmd[RIGHT_ELBOW] = %lf",cmd[RIGHT_ELBOW]);
			ROS_WARN("current[RIGHT_ELBOW] = %lf", current[RIGHT_ELBOW]);
		}
		if(i==LEFT_SHOULDER) {
                        ROS_WARN("cmd[LEFT_SHOULDER] = %lf",cmd[LEFT_SHOULDER]);
                        ROS_WARN("current[LEFT_SHOULDER] = %lf", current[LEFT_SHOULDER]);
                }*/
		// K = 0.0439, current -> torque
		// current[i] = (cmd[i]/0.0439);//cmd is in Nm

		// current[i] = (cmd[i]/0.0452);

		eff[i] = cmd[i];	// torque
		// saturate to max current
		if (current[i] >= max_current[i]) {
			current[i] = max_current[i];
			// max torque
			// eff[i] = current[i]*0.0439;//eff is in Nm
			eff[i] = current[i]*0.0452;//eff is in Nm
		}
		else if (current[i] <=-max_current[i]) {
			current[i] =-max_current[i];
			// eff[i] = current[i]*0.0439;//eff is in Nm
			eff[i] = current[i]*0.0452;//eff is in Nm
		}

		if(current[i] == 0){
			dir[i] = 0;
			width[i] = (uint16_t)(current[i]*(PWM_MOTOR_RANGE/max_current[i]));
			// width[i] = (uint16_t)(current[i]*(PWM_MOTOR_RANGE/max_current[i])) + PWM_MOTOR_MIN_DT;

		}
		else if (current[i] > 0.0)
		{
			dir[i] = 0;
			width[i] = (uint16_t)(current[i]*(PWM_MOTOR_RANGE/max_current[i])) + PWM_MOTOR_MIN_DT;
		}
		else
		{
			dir[i] = 1;
			width[i] = (uint16_t)(-current[i]*(PWM_MOTOR_RANGE/max_current[i])) + PWM_MOTOR_MIN_DT;
		}
/*
		if(i==RIGHT_ELBOW) {
                        ROS_WARN("width[RIGHT_ELBOW] = %lf", width[RIGHT_ELBOW]);
                        ROS_WARN("eff[RIGHT_ELBOW] = %lf", eff[RIGHT_ELBOW]);
                }*/
		//if(i==LEFT_SHOULDER) {
                  //      ROS_WARN("width[LEFT_SHOULDER] = %lf", width[LEFT_SHOULDER]);
                    //    ROS_WARN("eff[LEFT_SHOULDER] = %lf", eff[LEFT_SHOULDER]);
               // }

		// dir[i]= 1;
	}

	//RC SERVOS
	for (int i=8; i<12; i++)
	{
		//Left Finger(0-120 deg)
		if (i==LEFT_GRIPPER) {

			if(cmd[i] >= 0 && cmd[i] <= LEFT_FINGER_MAX_ANGLE ){

				//ROS_WARN("div %f",div);
				//ROS_WARN("WM cmd %lf",cmd[LEFT_GRIPPER]);

				double div = (double)cmd[i]/(double)LEFT_FINGER_MAX_ANGLE;
				width[i] = (uint16_t)(div*PWM_FINGER_SERVO_RANGE + PWM_FINGER_SERVO_MIN_DT);

				write_left_finger(width[i]);

			}
			else {
				width[i] = width[i];
				ROS_WARN("Servo commanded out of range. Command Ignored");
			}
		}

		if(i==RIGHT_GRIPPER){

			if(cmd[i] >= 0 && cmd[i] <= RIGHT_FINGER_MAX_ANGLE ){

				//ROS_WARN("div %f",div);
				//ROS_WARN("WM cmd %lf",cmd[LEFT_GRIPPER]);

				double div = (double)cmd[i]/(double)RIGHT_FINGER_MAX_ANGLE;
				width[i] = (uint16_t)(div*PWM_FINGER_SERVO_RANGE + PWM_FINGER_SERVO_MIN_DT);

				write_right_finger(width[i]);
			}
			else{
				width[i] = width[i];
				//ROS_WARN("Servo commanded out of range. Command Ignored");
			}

		}


		//Left Wrist (0-120 deg)
		if(i==LEFT_WRIST){

			if(cmd[i] >= 0 && cmd[i] <= LEFT_WRIST_MAX_ANGLE ){

				double div = (double)cmd[i]/(double)LEFT_WRIST_MAX_ANGLE;

				//ROS_WARN("cmd %lf",cmd[LEFT_WRIST]);
				width[i] = (uint16_t)(div*PWM_WRIST_SERVO_RANGE + PWM_WRIST_SERVO_MIN_DT);
				write_left_wrist(width[i]);

			}
			else {
				width[i] = width[i];
				ROS_WARN("Servo commanded out of range. Command Ignored");
			}

		}

		if(i==RIGHT_WRIST){

			if(cmd[i] >= 0 && cmd[i] <= RIGHT_WRIST_MAX_ANGLE ){

				double div = (double)cmd[i]/(double)RIGHT_WRIST_MAX_ANGLE;

				//ROS_WARN("cmd %lf",cmd[i]);
				width[i] = (uint16_t)(div*PWM_WRIST_SERVO_RANGE + PWM_WRIST_SERVO_MIN_DT);
				write_right_wrist(width[i]);

			}
			else {
				width[i] = width[i];
				ROS_WARN("Servo commanded out of range. Command Ignored");
			}

		}

	}

	//Seting direction (pins for Port 2)
	uint16_t directions = (dir[0]<<1) | (dir[1]<<3) | (dir[2]<<5) | (dir[3]<<7);
	dm7820_status = DM7820_StdIO_Set_Output(board, DM7820_STDIO_PORT_2, directions);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_motor_direction");

	//Seting direction (pins for Port 2)
	//uint16_t manipulator_directions = (dir[4]<<1) | (dir[5]<<3) | (dir[6]<<5) | (dir[7]<<7);
	uint16_t manipulator_directions = (dir[4]<<1) | (dir[5]<<3) | (dir[6]<<5) | (dir[7]<<7);
	//Try Panos Kwstas
	//uint16_t manipulator_directions = (dir[4]<<1) | (dir[5]<<3) | (dir[4]<<5) | (dir[7]<<7);

	dm7820_status = DM7820_StdIO_Set_Output(manipulator_board, DM7820_STDIO_PORT_2, manipulator_directions);
	DM7820_Return_Status(dm7820_status, "DM7820_manipulator_board_StdIO_Set_motor_direction");

	// Set output PWM width
	dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_A,  width[0]);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_motor_Width[0]");
	dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_B,  width[1]);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_motor_Width[1]");
	/*
	dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_C,  width[2]);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_motor_Width[2]");
	dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_D,  width[3]);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_motor_Width[3]");
	*/

	// Set output PWM width
	dm7820_status = DM7820_PWM_Set_Width(manipulator_board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_A,  width[4]);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_motor_Width[4]");
	dm7820_status = DM7820_PWM_Set_Width(manipulator_board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_B,  width[5]);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_motor_Width[5]");

	dm7820_status = DM7820_PWM_Set_Width(manipulator_board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_C,  width[6]);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_motor_Width[6]");
	dm7820_status = DM7820_PWM_Set_Width(manipulator_board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_D,  width[7]);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_motor_Width[7]");

	//Left wrist and gripper
	/*dm7820_status = DM7820_PWM_Set_Width(manipulator_board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_C,  width[8]);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_motor_Width[6]");
	dm7820_status = DM7820_PWM_Set_Width(manipulator_board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_D,  width[10]);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_motor_Width[7]");
	*/

	/*
	dm7820_status = DM7820_PWM_Set_Width(manipulator_board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_A,  width[8]); //0.5ms
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
	dm7820_status = DM7820_PWM_Set_Width(manipulator_board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_B,  width[10]);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
	*/


	/*dm7820_status = DM7820_PWM_Set_Width(manipulator_board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_C,  width[9]);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
	dm7820_status = DM7820_PWM_Set_Width(manipulator_board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_D,  width[11]);//PWM_HOBBY_SERVO_RANGE/2 + PWM_HOBBY_SERVO_MIN_DT);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
	*/
}

void CepheusHW::heartbeat()
{

	strobe = !strobe ;
	DM7820_StdIO_Strobe_Output(manipulator_board, DM7820_STDIO_STROBE_1, strobe);
	DM7820_StdIO_Strobe_Output(manipulator_board, DM7820_STDIO_STROBE_2, strobe);
	// DM7820_StdIO_Strobe_Output(board, DM7820_STDIO_STROBE_1, strobe);
	// DM7820_StdIO_Strobe_Output(board, DM7820_STDIO_STROBE_2, strobe);

}

void CepheusHW::enable()
{
	DM7820_StdIO_Strobe_Output(manipulator_board, DM7820_STDIO_STROBE_1, 0xFF);
	DM7820_StdIO_Strobe_Output(manipulator_board, DM7820_STDIO_STROBE_2, 0xFF);
}

void CepheusHW::disable()
{
	DM7820_StdIO_Strobe_Output(manipulator_board, DM7820_STDIO_STROBE_1, 0x00);
	DM7820_StdIO_Strobe_Output(manipulator_board, DM7820_STDIO_STROBE_2, 0x00);
}



bool CepheusHW::isLimitReached(int i)
{
	return limit[i];
}

uint16_t prev = 0;
void CepheusHW::readLimitSwitches()
{
	uint16_t input;
	dm7820_status = DM7820_StdIO_Get_Input (manipulator_board, DM7820_STDIO_PORT_0, &input);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Get_Input()");

	// printf("input:");
	// print_binary(input);
	// printf("masked1:");
	// print_binary(input&(1<<(int)LIMIT_L1));
	//if (prev != (input&(1<<LIMIT_L1))) ROS_WARN("DIFF!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
	//prev = input&(1<<LIMIT_L1);
	// printf("masked2:");
	// print_binary(input&(1<<(int)LIMIT_L2));
	//printf("%d\n", input&(1<<(int)LIMIT_L2) );

	if(!(input&(1<<(int)LIMIT_L1)))
	{
		// ROS_INFO("%d",input&(1<<LIMIT_L1));
		limit[LEFT_SHOULDER] = 1;
		// may print if sensor not connected
		ROS_WARN("limit 4 pressed");
	}
	else limit[LEFT_SHOULDER] = 0;

	if(!(input&(1<<(int)LIMIT_L2)))
	{
		// ROS_INFO("%d",input&(1<<LIMIT_L2));
		limit[LEFT_ELBOW] = 1;
		// may print if sensor not connected
		ROS_WARN("limit 5 pressed");
	}
	else limit[LEFT_ELBOW] = 0;

	if(!(input&(1<<(int)LIMIT_R1)))
	{
		// ROS_INFO("%d",input&(1<<LIMIT_L2));
		//limit[RIGHT_SHOULDER] = 1;
		limit[RIGHT_ELBOW] = 1;
		// may print if sensor not connected
		// ROS_WARN("limit 6 pressed");
	}
	else
		limit[RIGHT_ELBOW] = 0;
	//else limit[RIGHT_SHOULDER] = 0;

	/*if(!(input&(1<<(int)LIMIT_R2)))
	{
		//ROS_INFO("%d",input&(1<<LIMIT_L2));
		//limit[RIGHT_ELBOW] = 1;
		//ROS_WARN("limit 7 pressed");
	}
	else limit[RIGHT_ELBOW] = 0;
*/
}

void CepheusHW::readEncoders(ros::Duration dt)
{
	readLimitSwitches();
	// read robots joint state
	//Read encoder 0 channel A value
	dm7820_status = DM7820_IncEnc_Get_Independent_Value(board, DM7820_INCENC_ENCODER_0,
			DM7820_INCENC_CHANNEL_A,
			&encoder_1_val);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Independent_Value()");

	//Read encoder 0 channel B value
	dm7820_status = DM7820_IncEnc_Get_Independent_Value(board, DM7820_INCENC_ENCODER_0,
			DM7820_INCENC_CHANNEL_B,
			&encoder_2_val);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Independent_Value()");

	//Read encoder 1 channel A value
	dm7820_status = DM7820_IncEnc_Get_Independent_Value(board, DM7820_INCENC_ENCODER_1,
			DM7820_INCENC_CHANNEL_A,
			&encoder_3_val);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Independent_Value()");

	//Read encoder 1 channel B value
	dm7820_status = DM7820_IncEnc_Get_Independent_Value(board, DM7820_INCENC_ENCODER_1,
			DM7820_INCENC_CHANNEL_B,
			&encoder_4_val);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Independent_Value()");

	//handle overflow
	// Channel 0A Handling of encoder value overflow
	dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "Read positive overflow channel 0A");
	if (encoder_status)
		encoder_1_ovf++;

	dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "Read negative overflow channel 0A");
	if (encoder_status)
		encoder_1_ovf--;

	encoder_1 = (int64_t)encoder_1_val + UINT16_MAX*encoder_1_ovf;

	dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "Positive overflow clear channel 0A");
	if (encoder_status)
		error(EXIT_FAILURE, 0, "ERROR: Channel 0A positive overflow status not cleared");

	dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "negative overflow clear channel 0A");
	if (encoder_status)
		error(EXIT_FAILURE, 0, "ERROR: Channel 0A negative overflow status not cleared");


	// Channel 0B Handling of encoder value overflow
	dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_B_POSITIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "Read positive overflow channel 0B");
	if (encoder_status)
		encoder_2_ovf++;

	dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "Read negative overflow channel 0B");
	if (encoder_status)
		encoder_2_ovf--;

	encoder_2 = (int64_t)encoder_2_val + UINT16_MAX*encoder_2_ovf;

	dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_B_POSITIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "Positive overflow clear channel 0B");
	if (encoder_status)
		error(EXIT_FAILURE, 0, "ERROR: Channel 0B positive overflow status not cleared");

	dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "negative overflow clear channel 0B");
	if (encoder_status)
		error(EXIT_FAILURE, 0, "ERROR: Channel 0B negative overflow status not cleared");


	// Channel 1A Handling of encoder value overflow
	dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "Read positive overflow channel 1A");
	if (encoder_status)
		encoder_3_ovf++;

	dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "Read negative overflow channel 1A");
	if (encoder_status)
		encoder_3_ovf--;

	encoder_3 = (int64_t)encoder_3_val + UINT16_MAX*encoder_3_ovf;

	dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "Positive overflow clear channel 1A");
	if (encoder_status)
		error(EXIT_FAILURE, 0, "ERROR: Channel 1A positive overflow status not cleared");

	dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "negative overflow clear channel 1A");
	if (encoder_status)
		error(EXIT_FAILURE, 0, "ERROR: Channel 1A negative overflow status not cleared");


	// Channel 1B Handling of encoder value overflow
	dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_B_POSITIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "Read positive overflow channel 1B");
	if (encoder_status)
		encoder_4_ovf++;

	dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "Read negative overflow channel 1B");
	if (encoder_status)
		encoder_4_ovf--;

	encoder_4 = (int64_t)encoder_4_val + UINT16_MAX*encoder_4_ovf;

	dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_B_POSITIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "Positive overflow clear channel 1B");
	if (encoder_status)
		error(EXIT_FAILURE, 0, "ERROR: Channel 1B positive overflow status not cleared");

	dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "negative overflow clear channel 1B");
	if (encoder_status)
		error(EXIT_FAILURE, 0, "ERROR: Channel 1B negative overflow status not cleared");

	/**************************************************************************/
	/**************************MANIPULATOR BOARD ENCODER***********************/
	/**************************************************************************/

	// read robots joint state
	//Read encoder 0 channel A value
	dm7820_status = DM7820_IncEnc_Get_Independent_Value(manipulator_board, DM7820_INCENC_ENCODER_0,
			DM7820_INCENC_CHANNEL_A,
			&encoder_5_val);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Independent_Value()");

	//Read encoder 0 channel B value
	dm7820_status = DM7820_IncEnc_Get_Independent_Value(manipulator_board, DM7820_INCENC_ENCODER_0,
			DM7820_INCENC_CHANNEL_B,
			&encoder_6_val);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Independent_Value()");

	//Read encoder 1 channel A value
	dm7820_status = DM7820_IncEnc_Get_Independent_Value(manipulator_board, DM7820_INCENC_ENCODER_1,
			DM7820_INCENC_CHANNEL_A,
			&encoder_7_val);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Independent_Value()");

	//Read encoder 1 channel B value
	dm7820_status = DM7820_IncEnc_Get_Independent_Value(manipulator_board, DM7820_INCENC_ENCODER_1,
			DM7820_INCENC_CHANNEL_B,
			&encoder_8_val);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Independent_Value()");

	//handle overflow
	// Channel 0A Handling of encoder value overflow
	dm7820_status = DM7820_IncEnc_Get_Status(manipulator_board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "Read positive overflow channel 0A");

	if (encoder_status){ 

		encoder_5_ovf++;

		dm7820_status = DM7820_IncEnc_Get_Status(manipulator_board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER, &encoder_status);
		DM7820_Return_Status(dm7820_status, "Positive overflow clear channel 0A");
		if (encoder_status)
			error(EXIT_FAILURE, 0, "ERROR: Channel 0A positive overflow status not cleared");

	}

	dm7820_status = DM7820_IncEnc_Get_Status(manipulator_board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "Read negative overflow channel 0A");
	if (encoder_status){

		encoder_5_ovf--;

		dm7820_status = DM7820_IncEnc_Get_Status(manipulator_board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER, &encoder_status);
		DM7820_Return_Status(dm7820_status, "negative overflow clear channel 0A");
		if (encoder_status)
			error(EXIT_FAILURE, 0, "ERROR: Channel 0A negative overflow status not cleared");
	}

	encoder_5 = (int64_t)encoder_5_val + UINT16_MAX*encoder_5_ovf;



	// Channel 0B Handling of encoder value overflow
	dm7820_status = DM7820_IncEnc_Get_Status(manipulator_board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_B_POSITIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "Read positive overflow channel 0B");
	if (encoder_status){

		encoder_6_ovf++;

		dm7820_status = DM7820_IncEnc_Get_Status(manipulator_board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_B_POSITIVE_ROLLOVER, &encoder_status);
		DM7820_Return_Status(dm7820_status, "Positive overflow clear channel 0B");
		if (encoder_status)
			error(EXIT_FAILURE, 0, "ERROR: Channel 0B positive overflow status not cleared");

	}

	dm7820_status = DM7820_IncEnc_Get_Status(manipulator_board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "Read negative overflow channel 0B");
	if (encoder_status){
		encoder_6_ovf--;

		dm7820_status = DM7820_IncEnc_Get_Status(manipulator_board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER, &encoder_status);
		DM7820_Return_Status(dm7820_status, "negative overflow clear channel 0B");
		if (encoder_status)
			error(EXIT_FAILURE, 0, "ERROR: Channel 0B negative overflow status not cleared");

	}

	encoder_6 = (int64_t)encoder_6_val + UINT16_MAX*encoder_6_ovf;




	// Channel 1A Handling of encoder value overflow
	dm7820_status = DM7820_IncEnc_Get_Status(manipulator_board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "Read positive overflow channel 1A");
	if (encoder_status){

		encoder_7_ovf++;

		dm7820_status = DM7820_IncEnc_Get_Status(manipulator_board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER, &encoder_status);
		DM7820_Return_Status(dm7820_status, "Positive overflow clear channel 1A");
		if (encoder_status)
			error(EXIT_FAILURE, 0, "ERROR: Channel 1A positive overflow status not cleared");
	}

	dm7820_status = DM7820_IncEnc_Get_Status(manipulator_board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "Read negative overflow channel 1A");
	if (encoder_status) {

		encoder_7_ovf--;

		dm7820_status = DM7820_IncEnc_Get_Status(manipulator_board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER, &encoder_status);
		DM7820_Return_Status(dm7820_status, "negative overflow clear channel 1A");
		if (encoder_status)
			error(EXIT_FAILURE, 0, "ERROR: Channel 1A negative overflow status not cleared");
	}

	encoder_7 = (int64_t)encoder_7_val + UINT16_MAX*encoder_7_ovf;



	// Channel 1B Handling of encoder value overflow
	dm7820_status = DM7820_IncEnc_Get_Status(manipulator_board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_B_POSITIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "Read positive overflow channel 1B");
	if (encoder_status){

		encoder_8_ovf++;

		dm7820_status = DM7820_IncEnc_Get_Status(manipulator_board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_B_POSITIVE_ROLLOVER, &encoder_status);
		DM7820_Return_Status(dm7820_status, "Positive overflow clear channel 1B");
		if (encoder_status)
			error(EXIT_FAILURE, 0, "ERROR: Channel 1B positive overflow status not cleared");

	}

	dm7820_status = DM7820_IncEnc_Get_Status(manipulator_board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "Read negative overflow channel 1B");
	if (encoder_status){

		encoder_8_ovf--;

		dm7820_status = DM7820_IncEnc_Get_Status(manipulator_board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER, &encoder_status);
		DM7820_Return_Status(dm7820_status, "negative overflow clear channel 1B");
		if (encoder_status)
			error(EXIT_FAILURE, 0, "ERROR: Channel 1B negative overflow status not cleared");

	}

	encoder_8 = (int64_t)encoder_8_val + UINT16_MAX*encoder_8_ovf;




	// ROS_INFO("1: %d, 2: %d, 3: %d, 4: %d", encoder_1, encoder_2, encoder_3, encoder_4);
	//190:1 = 12167/64
	// Position Calculation radians
	pos[0]=  (double)encoder_1*2*M_PI/(4095) + offset_pos[0];
	pos[1]=  (double)encoder_2*2*M_PI/(4000) + offset_pos[1];
	pos[2]=  (double)encoder_3*2*M_PI/(4000) + offset_pos[2];
	pos[3]=  (double)encoder_4*2*M_PI/(4000) + offset_pos[3];

	// 121027.38703744316 rads = ?
	// rad = (encoder / (encoding type * CPT) * 2 * pi) / (reducer transmission ratio)  => !!!!encoding_type * CPT * reduction / 2*pi!!!
	// CPT (counts per turn) = 1000 [depends on motor]
	// encoding type = 4 [depends on motor]
	// reducer transmission ratio = 190  [190:1 = 12167/64]

	//New maxon motors (DCX22L EBKL 24V) with reduction 185.93:1, enc_type = 4, CPT=1000


	//pos[4]=  (double)(encoder_5/121027.38703744316) + offset_pos[4]; (old motors with 190:1 reduction)
	if (offset_pos[4])
		pos[4] = - ( (double)(encoder_6/118366.714)) + offset_pos[4];
	else
		pos[4] = ( (double)(encoder_6/118366.714) );//for new motors (silver)

	pos[5]= (double)(encoder_5/118366.714) + offset_pos[5];//for new motors (silver)

	//pos[5]=  (double)(encoder_6/121027.38703744316) - (double)(encoder_5/121027.38703744316) + offset_pos[5];
	//pos[5]=  (double)(encoder_6/121027.38703744316) - pos[4] + offset_pos[5];
	//printf("offset:%f\n",offset_pos[5]);
	//pos[5]=  (double)(encoder_6/121027.38703744316) + (double)(encoder_5/121027.38703744316) + offset_pos[5];
	//pos[6]=  (double)(encoder_7/121027.38703744316) + offset_pos[6];

	// pos 6 now reaction wheel 2020 update
	pos[6] = (double)encoder_7*2*M_PI/(4095) + offset_pos[6];
	pos[0]=pos[6];
	//pos[7]=  (double)(encoder_8/121027.38703744316) + offset_pos[7] - pos[6];
	//pos[7]=  (double)(encoder_8/121027.38703744316) - (double)(encoder_7/121027.38703744316) + offset_pos[7];
	if (offset_pos[7])
		pos[7] = - ( (double)(encoder_8/118366.714)) + offset_pos[7]; //for new motors (silver) 2020
	else
		pos[7] = ( (double)(encoder_8/118366.714));
	//ROS_INFO(" 6 %lf 7 %lf", (double)(encoder_7/121027.38703744316), (double)(encoder_8/121027.38703744316));
	//ROS_INFO("(1)pos[4]: %lf  | (2)pos[5]: %lf  | (3)pos[7]: %lf", pos[4], pos[5], pos[7]);

	// ROS_INFO("readEncoders: ls: %f, le: %f, rw: %f, re: %f", pos[4], pos[5], pos[6], pos[7]);

	//ROS_INFO("DT: %lf", dt.toSec());
	// Speed Calculation radians/sec
	for(int i=0; i<8; i++)
	{
		vel_new[i]= ((pos[i] - prev_pos[i])) / dt.toSec();
		//ROS_WARN("vel[LEFT_SHOULDER]: %lf", vel_new[4]);
		prev_pos[i] = pos[i];


		//vel[0] = vel_new[0];  // reaction wheel

	//	vel[4] = vel_new[4];  //left shoulder
	//	vel[5] = vel_new[5];  //left elbow

	//	vel[6] = vel_new[6];  //right shoulder
	//	vel[7] = vel_new[7];  //right elbow



		// for (int j=0; j<(FIR_LENGTH-1); j++)
		//   vel_fir[j][i] = vel_fir[j+1][i];

		// vel_fir[FIR_LENGTH-1][i] = vel_new[i];

		// double filtered=0;
		// for (int j=0; j<FIR_LENGTH; j++)
		//   filtered += vel_fir[j][i];

		// vel[i] = filtered/FIR_LENGTH;

	}
	vel[0] = vel_new[0];  // reaction wheel

	vel[4] = vel_new[4];  //left shoulder
	vel[5] = vel_new[5];  //left elbow

	vel[6] = vel_new[6];  //right shoulder
	vel[0] = vel[6];
	vel[7] = vel_new[7];  //right elbow


	// ROS_INFO("POS: 1: %f, 2: %f, 3: %f, 4: %f, 5: %f, 6: %f, 7: %f, 8: %f", pos[0], pos[1], pos[2], pos[3] ,pos[4] ,pos[5] ,pos[6] ,pos[7]);
}

void CepheusHW::setParam(double *max_cur, double f_thrust)
{
	this->max_thrust = f_thrust;
	for(int i=0; i<8; i++) {
		this->max_current[i] = max_cur[i];
		ROS_INFO("max_current[%d]=%f", i, max_current[i] );
	}
}

void CepheusHW::setCmd(int idx, double _cmd)
{
	cmd[idx] = _cmd;
}

double CepheusHW::getVel(int idx)
{
	return vel[idx];
}


CepheusHW::CepheusHW()
{
	//Panagiotis Mavridis
	//Initialzing the force sensor values to 0
	for(int i=0; i < FSR_NUM; i++){
		fsr_values[i] = 0;
	}
	//Pelekoudas init cmd values to 0;
	for (int i=4; i<8; i++)
		cmd[i] = 0.00000001;

	int16_t encoder_init_value = 0;

	// connect and register the reaction_wheel joint state interface
	hardware_interface::JointStateHandle state_handle_reaction_wheel("reaction_wheel_joint", &pos[0], &vel[0], &eff[0]);
	jnt_state_interface.registerHandle(state_handle_reaction_wheel);
	// connect and register the left_shoulder joint state interface
	hardware_interface::JointStateHandle state_handle_left_shoulder("left_shoulder", &pos[4], &vel[4], &eff[4]);
	jnt_state_interface.registerHandle(state_handle_left_shoulder);
	// connect and register the left_elbow joint state interface
	hardware_interface::JointStateHandle state_handle_left_elbow("left_elbow", &pos[5], &vel[5], &eff[5]);
	jnt_state_interface.registerHandle(state_handle_left_elbow);
	// connect and register right_wrist the joint state interface

	/*hardware_interface::JointStateHandle state_handle_left_wrist("left_wrist", &cmd[8], &force[0], &force[0]);
	  jnt_state_interface.registerHandle(state_handle_left_wrist);

	  hardware_interface::JointStateHandle state_handle_left_finger_joint("left_finger_joint", &cmd[10], &force[0], &force[0]);
	  jnt_state_interface.registerHandle(state_handle_left_finger_joint);
	 */

	// connect and register the right_shoulder joint state interface
	hardware_interface::JointStateHandle state_handle_right_shoulder("right_shoulder", &pos[6], &vel[6], &eff[6]);
	jnt_state_interface.registerHandle(state_handle_right_shoulder);
	// connect and register right_elbow the joint state interface
	hardware_interface::JointStateHandle state_handle_right_elbow("right_elbow", &pos[7], &vel[7], &eff[7]);
	jnt_state_interface.registerHandle(state_handle_right_elbow);
	// connect and register right_wrist the joint state interface
	// hardware_interface::JointStateHandle state_handle_right_wrist("right_wrist", &cmd[9], &force[1], &force[1]);
	// jnt_state_interface.registerHandle(state_handle_right_wrist);

	// hardware_interface::JointStateHandle state_handle_right_finger_joint("right_finger_joint", &cmd[11], &force[0], &force[0]);
	// jnt_state_interface.registerHandle(state_handle_right_finger_joint);

	registerInterface(&jnt_state_interface);

	// connect and register the reaction_wheel joint effort interface
	hardware_interface::JointHandle effort_handle_reaction_wheel(jnt_state_interface.getHandle("reaction_wheel_joint"), &cmd[0]);
	jnt_eff_interface.registerHandle(effort_handle_reaction_wheel);
	// connect and register the left_shoulder joint effort interface
	hardware_interface::JointHandle effort_handle_left_shoulder(jnt_state_interface.getHandle("left_shoulder"), &cmd[4]);
	jnt_eff_interface.registerHandle(effort_handle_left_shoulder);
	// connect and register the left_elbow joint effort interface
	hardware_interface::JointHandle effort_handle_left_elbow(jnt_state_interface.getHandle("left_elbow"), &cmd[5]);
	jnt_eff_interface.registerHandle(effort_handle_left_elbow);
	// connect and register the left_wrist joint effort interface

	/*hardware_interface::JointHandle position_handle_left_wrist(jnt_state_interface.getHandle("left_wrist"), &cmd[8]);
	  jnt_pos_interface.registerHandle(position_handle_left_wrist);
	// connect and register the left_grip joint effort interface
	hardware_interface::JointHandle position_handle_left_finger_joint(jnt_state_interface.getHandle("left_finger_joint"), &cmd[10]);
	jnt_pos_interface.registerHandle(position_handle_left_finger_joint);
	 */
	// connect and register the right_shoulder joint effort interface
	hardware_interface::JointHandle effort_handle_right_shoulder(jnt_state_interface.getHandle("right_shoulder"), &cmd[6]);
	jnt_eff_interface.registerHandle(effort_handle_right_shoulder);
	// connect and register the right_elbow joint effort interface
	hardware_interface::JointHandle effort_handle_right_elbow(jnt_state_interface.getHandle("right_elbow"), &cmd[7]);
	jnt_eff_interface.registerHandle(effort_handle_right_elbow);
	// connect and register the right_wrist joint effort interface
	// hardware_interface::JointHandle position_handle_right_wrist(jnt_state_interface.getHandle("right_wrist"), &cmd[9]);
	// jnt_pos_interface.registerHandle(position_handle_right_wrist);
	// // connect and register the left_grip joint effort interface
	// hardware_interface::JointHandle position_handle_right_finger_joint(jnt_state_interface.getHandle("right_finger_joint"), &cmd[11]);
	//   jnt_pos_interface.registerHandle(position_handle_right_finger_joint);

	registerInterface(&jnt_eff_interface);
	registerInterface(&jnt_pos_interface);


	//ENCODER CARD
	// TRY ACCESS CARDS
	uint32_t base_minor_number = 1;
	uint32_t manipulator_minor_number = 0;

	dm7820_status = DM7820_General_Open_Board(base_minor_number, &board);
	DM7820_Return_Status(dm7820_status, "Opening Device, base card");

	dm7820_status = DM7820_General_Open_Board(manipulator_minor_number, &manipulator_board);
	DM7820_Return_Status(dm7820_status, "Opening Device, manipulator card");

	//INCREMENTAL ENCODERS 0 & 1
	//disable encoders for safety
	dm7820_status = DM7820_IncEnc_Enable(board, DM7820_INCENC_ENCODER_0, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");

	dm7820_status = DM7820_IncEnc_Enable(board, DM7820_INCENC_ENCODER_1, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");

	//Set each port 0 0-5 and 8-13 bit to input which enables incremental encoder inputs
	dm7820_status = DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_0, 0xFFFF, DM7820_STDIO_MODE_INPUT);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

	//Set each port 1 0-5 and 8-13 bit to input which enables incremental encoder inputs
	dm7820_status = DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_1, 0xFFFF, DM7820_STDIO_MODE_INPUT);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

	//Set master clock to 25 MHz clock
	//Incremental encoder 0 initialization
	dm7820_status = DM7820_IncEnc_Set_Master(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_MASTER_25_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Master()");
	//Incremental encoder 1 initialization
	dm7820_status = DM7820_IncEnc_Set_Master(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_MASTER_25_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Master()");

	//Disable value register hold 0
	dm7820_status = DM7820_IncEnc_Enable_Hold(board, DM7820_INCENC_ENCODER_0, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable_Hold()");

	//Disable value register hold 1
	dm7820_status = DM7820_IncEnc_Enable_Hold(board, DM7820_INCENC_ENCODER_1, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable_Hold()");

	/*
	 * Configure the incremental encoder as follows: 1) disable all up
	 * transitions from modifying the counter, 2) set single-ended input mode,
	 * 3) disable the input filter, 4) set channels A and B to operate
	 * independently of each other, and 5) disable index input
	 */
	DM7820_INCENC_RESET_PHASE_FILTER(phase_filter);
	dm7820_status = DM7820_IncEnc_Configure(board,
			DM7820_INCENC_ENCODER_0,
			phase_filter,
			DM7820_INCENC_INPUT_SINGLE_ENDED,
			0x00,
			DM7820_INCENC_CHANNEL_INDEPENDENT,
			0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Configure()");

	//Set initial value for channel A counter
	dm7820_status = DM7820_IncEnc_Set_Independent_Value(board,
			DM7820_INCENC_ENCODER_0,
			DM7820_INCENC_CHANNEL_A,
			encoder_init_value);
	DM7820_Return_Status(dm7820_status,
			"DM7820_IncEnc_Set_Independent_Value_0_A");

	//Set initial value for channel B counter
	dm7820_status = DM7820_IncEnc_Set_Independent_Value(board,
			DM7820_INCENC_ENCODER_0,
			DM7820_INCENC_CHANNEL_B,
			encoder_init_value);
	DM7820_Return_Status(dm7820_status,
			"DM7820_IncEnc_Set_Independent_Value_0_B");



	/*
	 * Configure the incremental encoder as follows: 1) disable all down
	 * transitions from modifying the counter, 2) set single-ended input mode,
	 * 3) disable the input filter, 4) set channels A and B to operate
	 * independently of each other, and 5) disable index input
	 */
	DM7820_INCENC_RESET_PHASE_FILTER(phase_filter);
	dm7820_status = DM7820_IncEnc_Configure(board,
			DM7820_INCENC_ENCODER_1,
			phase_filter,
			DM7820_INCENC_INPUT_SINGLE_ENDED,
			0x00,
			DM7820_INCENC_CHANNEL_INDEPENDENT,
			0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Configure()");

	//Set initial value for channel A counter
	dm7820_status = DM7820_IncEnc_Set_Independent_Value(board,
			DM7820_INCENC_ENCODER_1,
			DM7820_INCENC_CHANNEL_A,
			encoder_init_value);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Independent_Value_1_A");

	//Set initial value for channel A counter
	dm7820_status = DM7820_IncEnc_Set_Independent_Value(board,
			DM7820_INCENC_ENCODER_1,
			DM7820_INCENC_CHANNEL_B,
			encoder_init_value);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Independent_Value_1_B");

	//incremental encoder 0 enable again
	dm7820_status = DM7820_IncEnc_Enable(board, DM7820_INCENC_ENCODER_0, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");

	//incremental encoder 1 enable again
	dm7820_status = DM7820_IncEnc_Enable(board, DM7820_INCENC_ENCODER_1, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");

	//Clear channel negative and positive rollover status flag without checking its state
	dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");
	dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");

	dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");
	dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_B_POSITIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");

	dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");
	dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");

	dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");
	dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_B_POSITIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");

	/*****************************************************************/
	/*************************MANIPULATOR CARD************************/
	/*****************************************************************/

	//INCREMENTAL ENCODERS 0 & 1
	//disable encoders for safety
	dm7820_status = DM7820_IncEnc_Enable(manipulator_board, DM7820_INCENC_ENCODER_0, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");

	dm7820_status = DM7820_IncEnc_Enable(manipulator_board, DM7820_INCENC_ENCODER_1, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");

	//Set each port 0 0-5 and 8-13 bit to input which enables incremental encoder inputs
	dm7820_status = DM7820_StdIO_Set_IO_Mode(manipulator_board, DM7820_STDIO_PORT_0, 0xFFFF, DM7820_STDIO_MODE_INPUT);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

	//Set each port 1 0-5 and 8-13 bit to input which enables incremental encoder inputs
	dm7820_status = DM7820_StdIO_Set_IO_Mode(manipulator_board, DM7820_STDIO_PORT_1, 0xFFFF, DM7820_STDIO_MODE_INPUT);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

	//Set master clock to 25 MHz clock
	//Incremental encoder 0 initialization
	dm7820_status = DM7820_IncEnc_Set_Master(manipulator_board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_MASTER_25_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Master()");
	//Incremental encoder 1 initialization
	dm7820_status = DM7820_IncEnc_Set_Master(manipulator_board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_MASTER_25_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Master()");

	//Disable value register hold 0
	dm7820_status = DM7820_IncEnc_Enable_Hold(manipulator_board, DM7820_INCENC_ENCODER_0, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable_Hold()");

	//Disable value register hold 1
	dm7820_status = DM7820_IncEnc_Enable_Hold(manipulator_board, DM7820_INCENC_ENCODER_1, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable_Hold()");

	/*
	 * Configure the incremental encoder as follows: 1) disable all up
	 * transitions from modifying the counter, 2) set single-ended input mode,
	 * 3) disable the input filter, 4) set channels A and B to operate
	 * independently of each other, and 5) disable index input
	 */

	DM7820_INCENC_RESET_PHASE_FILTER(phase_filter);
	dm7820_status = DM7820_IncEnc_Configure(manipulator_board,
			DM7820_INCENC_ENCODER_0,
			phase_filter,
			DM7820_INCENC_INPUT_SINGLE_ENDED,
			0x00,
			DM7820_INCENC_CHANNEL_INDEPENDENT,
			0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Configure()");

	//Set initial value for channel A counter
	dm7820_status = DM7820_IncEnc_Set_Independent_Value(manipulator_board,
			DM7820_INCENC_ENCODER_0,
			DM7820_INCENC_CHANNEL_A,
			encoder_init_value);
	DM7820_Return_Status(dm7820_status,
			"DM7820_IncEnc_Set_Independent_Value_0_A");

	//Set initial value for channel B counter
	dm7820_status = DM7820_IncEnc_Set_Independent_Value(manipulator_board,
			DM7820_INCENC_ENCODER_0,
			DM7820_INCENC_CHANNEL_B,
			encoder_init_value);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Independent_Value_0_B");

	/*
	 * Configure the incremental encoder as follows: 1) disable all down
	 * transitions from modifying the counter, 2) set single-ended input mode,
	 * 3) disable the input filter, 4) set channels A and B to operate
	 * independently of each other, and 5) disable index input
	 */
	DM7820_INCENC_RESET_PHASE_FILTER(phase_filter);
	dm7820_status = DM7820_IncEnc_Configure(manipulator_board,
			DM7820_INCENC_ENCODER_1,
			phase_filter,
			DM7820_INCENC_INPUT_SINGLE_ENDED,
			0x00,
			DM7820_INCENC_CHANNEL_INDEPENDENT,
			0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Configure()");

	//Set initial value for channel A counter
	dm7820_status = DM7820_IncEnc_Set_Independent_Value(manipulator_board,
			DM7820_INCENC_ENCODER_1,
			DM7820_INCENC_CHANNEL_A,
			encoder_init_value);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Independent_Value_1_A");

	//Set initial value for channel A counter
	dm7820_status = DM7820_IncEnc_Set_Independent_Value(manipulator_board,
			DM7820_INCENC_ENCODER_1,
			DM7820_INCENC_CHANNEL_B,
			encoder_init_value);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Independent_Value_1_B");

	//incremental encoder 0 enable again
	dm7820_status = DM7820_IncEnc_Enable(manipulator_board, DM7820_INCENC_ENCODER_0, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");

	//incremental encoder 1 enable again
	dm7820_status = DM7820_IncEnc_Enable(manipulator_board, DM7820_INCENC_ENCODER_1, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");

	//Clear channel negative and positive rollover status flag without checking its state
	dm7820_status = DM7820_IncEnc_Get_Status(manipulator_board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");
	dm7820_status = DM7820_IncEnc_Get_Status(manipulator_board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");

	dm7820_status = DM7820_IncEnc_Get_Status(manipulator_board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");
	dm7820_status = DM7820_IncEnc_Get_Status(manipulator_board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_B_POSITIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");

	dm7820_status = DM7820_IncEnc_Get_Status(manipulator_board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");
	dm7820_status = DM7820_IncEnc_Get_Status(manipulator_board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");

	dm7820_status = DM7820_IncEnc_Get_Status(manipulator_board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");
	dm7820_status = DM7820_IncEnc_Get_Status(manipulator_board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_B_POSITIVE_ROLLOVER, &encoder_status);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");

	encoder_5_ovf = 0;
	encoder_6_ovf = 0;
	encoder_7_ovf = 0;
	encoder_8_ovf = 0;

	encoder_5 = encoder_init_value;
	encoder_6 = encoder_init_value;
	encoder_7 = encoder_init_value;
	encoder_8 = encoder_init_value;

	//initialize prev_pos, offset_pos and filter buffer
	for(int i=4; i<8; i++)
	{
		home_pos[i] =0;
		pos[i] = 0;
		prev_pos[i] = pos[i];
		offset_pos[i] = 0;
		limit[i] = 0;

		//initialize FIR filter
		for(int j=0; j<FIR_LENGTH; j++)
			vel_fir[j][i]=0;
	}



	//device opened above

	/******************************************************************
	  motor digital to analog command using high freq PWM Init
	 *******************************************************************/
	//Disable pulse width modulators 0 to put them into a known state.
	//pulse width modulator should be disabled before programming it
	dm7820_status = DM7820_PWM_Enable(board, DM7820_PWM_MODULATOR_0, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_0_Disable()");

	// Set port's 2 (0,2,4,6) bits to peripheral output. For using as current control signal for motro controller
	dm7820_status = DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_2, 0x0055, DM7820_STDIO_MODE_PER_OUT);
	DM7820_Return_Status(dm7820_status, "DM7820_Set port2 (0,2,4,6) bits to peripheral output");

	//Set port's 2 (0,2,4,6) bits to PWM output pulse width modulator peripheral. For using as current control signal for motro controller
	dm7820_status = DM7820_StdIO_Set_Periph_Mode(board, DM7820_STDIO_PORT_2, 0x0055, DM7820_STDIO_PERIPH_PWM);
	DM7820_Return_Status(dm7820_status, "Set port2 (0,2,4,6) bits to PWM output");

	//set Port's 2 (1,3,5,7) bits to standard output for using it as direction signal of motor controller
	dm7820_status = DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_2, 0x00AA, DM7820_STDIO_MODE_OUTPUT);
	DM7820_Return_Status(dm7820_status, "Set Port's 2 (1,3,5,7) bits to standard output");

	//Set period master clock to 25 MHz clock
	dm7820_status = DM7820_PWM_Set_Period_Master(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_PERIOD_MASTER_25_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period_Master()");

	// Set pulse width modulator period to obtain frequency 25000000/ADC_PWM_PERIOD_COUNTS Hz
	dm7820_status = DM7820_PWM_Set_Period(board, DM7820_PWM_MODULATOR_0, ADC_PWM_PERIOD_COUNTS);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period()");

	//Set width master clock to 25 MHz clock
	dm7820_status = DM7820_PWM_Set_Width_Master(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_WIDTH_MASTER_25_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width_Master()");

	//zero out all pwms
	dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_A, 0);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
	dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_B, 0);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
	dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_C, 0);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
	dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_D, 0);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

	//enable PWM 0
	dm7820_status = DM7820_PWM_Enable(board, DM7820_PWM_MODULATOR_0, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Enable()");

	/*************************************************************************
	  THRUSTER CLOCK/PWM SETUP
	 ************************************************************************/

	/***Programmable clock 0 initialization***/

	//Disable PRGmble clock 0
	dm7820_status = DM7820_PrgClk_Set_Mode(board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_MODE_DISABLED);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");
	//maybe output init
	//Set master clock to 25 MHz clock
	dm7820_status = DM7820_PrgClk_Set_Master(board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_MASTER_25_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Master()");
	//Set clock start trigger to start immediately
	dm7820_status = DM7820_PrgClk_Set_Start_Trigger(board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_START_IMMEDIATE);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Start_Trigger()");
	//Set clock stop trigger so that clock is never stopped
	dm7820_status = DM7820_PrgClk_Set_Stop_Trigger(board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_STOP_NONE);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Stop_Trigger()");
	//Set clock period to obtain 50000 Hz frequency
	dm7820_status = DM7820_PrgClk_Set_Period(board, DM7820_PRGCLK_CLOCK_0, 500);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Period()");
	//Put clock into continuous mode and enable it
	dm7820_status = DM7820_PrgClk_Set_Mode(board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_MODE_CONTINUOUS);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");


	/****Thruster PWM 1 Init****/

	//Disable pulse width modulator 1 to put them into a known state
	//pulse width modulator should be disabled before programming it
	dm7820_status = DM7820_PWM_Enable(board, DM7820_PWM_MODULATOR_1, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_THRUSTER_PWM_Disable()");

	// Set port's 2 (8,9,10,11,12,13,14,15) bits to output.
	//this bits will used as PWM output but will be selected dynamicaly in the loop depending on the thrust direction
	dm7820_status = DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_2, 0x0FF00, DM7820_STDIO_MODE_OUTPUT);
	DM7820_Return_Status(dm7820_status, "Set port2 (8,9,10,11,12,13,14,15) bits to standard output");

	//Set period master clock to programmable clock 0 with freq 500 hz
	dm7820_status = DM7820_PWM_Set_Period_Master(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_PERIOD_MASTER_PROG_CLOCK_0);
	DM7820_Return_Status(dm7820_status, "DM7820_THRUSTER_PWM_Set_Period_Master()");

	// Set pulse width modulator period to obtain period 500/freq Hz
	period = (uint32_t)(50000/10);
	dm7820_status = DM7820_PWM_Set_Period(board, DM7820_PWM_MODULATOR_1, period);
	DM7820_Return_Status(dm7820_status, "DM7820_THRUSTER_PWM_Set_Period()");

	//Set width master clock to programmable clock 0 with freq 500 hz
	dm7820_status = DM7820_PWM_Set_Width_Master(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_WIDTH_MASTER_PROG_CLOCK_0);
	DM7820_Return_Status(dm7820_status, "DM7820_THRUSTER_PWM_Set_Width_Master()");

	// Set thrusters OFF initially
	dm7820_status = DM7820_StdIO_Set_Output(board, DM7820_STDIO_PORT_2, 0);
	DM7820_Return_Status(dm7820_status, "DM7820_SET_THRUSTER_OFF()");

	//zero out all pwms
	dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_A, 0);
	DM7820_Return_Status(dm7820_status, "DM7820_THRUSTER_PWM_Set_0");
	dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_B, 0);
	DM7820_Return_Status(dm7820_status, "DM7820_THRUSTER_PWM_Set_0");
	dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_C, 0);
	DM7820_Return_Status(dm7820_status, "DM7820_THRUSTER_PWM_Set_0");
	dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_D, 0);
	DM7820_Return_Status(dm7820_status, "DM7820_THRUSTER_PWM_Set_0");

	//enable PWM 1
	dm7820_status = DM7820_PWM_Enable(board, DM7820_PWM_MODULATOR_1, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_THRUSTER_PWM_Enable()");

	/******************************************************************
	  MANIPULATOR MOTOR PWM Init
	 *******************************************************************/
	//Disable pulse width modulators 0 to put them into a known state.
	//pulse width modulator should be disabled before programming it
	dm7820_status = DM7820_PWM_Enable(manipulator_board, DM7820_PWM_MODULATOR_0, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_0_Disable()");

	// Set port's 2 (0,2,4,6) bits to peripheral output. For using as current control signal for motro controller
	dm7820_status = DM7820_StdIO_Set_IO_Mode(manipulator_board, DM7820_STDIO_PORT_2, 0x0055, DM7820_STDIO_MODE_PER_OUT);
	DM7820_Return_Status(dm7820_status, "DM7820_Set port2 (0,2,4,6) bits to peripheral output");

	//Set port's 2 (0,2,4,6) bits to PWM output pulse width modulator peripheral. For using as current control signal for motro controller
	dm7820_status = DM7820_StdIO_Set_Periph_Mode(manipulator_board, DM7820_STDIO_PORT_2, 0x0055, DM7820_STDIO_PERIPH_PWM);
	DM7820_Return_Status(dm7820_status, "Set port2 (0,2,4,6) bits to PWM output");

	//set Port's 2 (1,3,5,7) bits to standard output for using it as direction signal of motor controller
	dm7820_status = DM7820_StdIO_Set_IO_Mode(manipulator_board, DM7820_STDIO_PORT_2, 0x00AA, DM7820_STDIO_MODE_OUTPUT);
	DM7820_Return_Status(dm7820_status, "Set Port's 2 (1,3,5,7) bits to standard output");

	//Set period master clock to 25 MHz clock
	dm7820_status = DM7820_PWM_Set_Period_Master(manipulator_board, DM7820_PWM_MODULATOR_0, DM7820_PWM_PERIOD_MASTER_25_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period_Master()");

	// Set pulse width modulator period to obtain frequency 25000000/PWM_MOTOR_PERIOD_COUNTS Hz
	dm7820_status = DM7820_PWM_Set_Period(manipulator_board, DM7820_PWM_MODULATOR_0, PWM_MOTOR_PERIOD_COUNTS);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period()");

	//Set width master clock to 25 MHz clock
	dm7820_status = DM7820_PWM_Set_Width_Master(manipulator_board, DM7820_PWM_MODULATOR_0, DM7820_PWM_WIDTH_MASTER_25_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width_Master()");

	//zero out all pwms
	dm7820_status = DM7820_PWM_Set_Width(manipulator_board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_A, PWM_MOTOR_MIN_DT); //10%
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
	dm7820_status = DM7820_PWM_Set_Width(manipulator_board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_B, PWM_MOTOR_MIN_DT);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
	dm7820_status = DM7820_PWM_Set_Width(manipulator_board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_C, PWM_MOTOR_MIN_DT);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
	dm7820_status = DM7820_PWM_Set_Width(manipulator_board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_D, PWM_MOTOR_MIN_DT);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

	//enable PWM 0
	dm7820_status = DM7820_PWM_Enable(manipulator_board, DM7820_PWM_MODULATOR_0, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Enable()");


	/*************************************************************************
	  RC SERVOS CLOCK/PWM SETUP
	 ************************************************************************/

	/***Programmable clock 0 initialization***/

	//Disable PRGmble clock 0
	dm7820_status = DM7820_PrgClk_Set_Mode(manipulator_board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_MODE_DISABLED);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");
	//maybe output init
	//Set master clock to 25 MHz clock
	dm7820_status = DM7820_PrgClk_Set_Master(manipulator_board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_MASTER_25_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Master()");
	//Set clock start trigger to start immediately
	dm7820_status = DM7820_PrgClk_Set_Start_Trigger(manipulator_board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_START_IMMEDIATE);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Start_Trigger()");
	//Set clock stop trigger so that clock is never stopped
	dm7820_status = DM7820_PrgClk_Set_Stop_Trigger(manipulator_board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_STOP_NONE);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Stop_Trigger()");
	//Set clock period to obtain 2,500,000 Hz frequency 25MHz / 10
	dm7820_status = DM7820_PrgClk_Set_Period(manipulator_board, DM7820_PRGCLK_CLOCK_0, 10);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Period()");
	//Put clock into continuous mode and enable it
	dm7820_status = DM7820_PrgClk_Set_Mode(manipulator_board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_MODE_CONTINUOUS);

	/******************************************************************
	  MANIPULATOR RC SERVO PWM Init
	 *******************************************************************/
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");
	//Disable pulse width modulators 1 to put them into a known state.
	//pulse width modulator should be disabled before programming it
	dm7820_status = DM7820_PWM_Enable(manipulator_board, DM7820_PWM_MODULATOR_1, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_0_Disable()");

	// Set port's 2 (0,2,4,6) bits to peripheral output. For using as current control signal for motro controller
	dm7820_status = DM7820_StdIO_Set_IO_Mode(manipulator_board, DM7820_STDIO_PORT_2, 0x5500, DM7820_STDIO_MODE_PER_OUT);
	DM7820_Return_Status(dm7820_status, "DM7820_Set port2 (0,2,4,6) bits to peripheral output");

	//Set port's 2 (0,2,4,6) bits to PWM output pulse width modulator peripheral. For using as current control signal for motro controller
	dm7820_status = DM7820_StdIO_Set_Periph_Mode(manipulator_board, DM7820_STDIO_PORT_2, 0x5500, DM7820_STDIO_PERIPH_PWM);
	DM7820_Return_Status(dm7820_status, "Set port2 (0,2,4,6) bits to PWM output");

	//set Port's 2 (1,3,5,7) bits to standard output for using it as direction signal of motor controller
	dm7820_status = DM7820_StdIO_Set_IO_Mode(manipulator_board, DM7820_STDIO_PORT_2, 0xAA00, DM7820_STDIO_MODE_OUTPUT);
	DM7820_Return_Status(dm7820_status, "Set Port's 2 (1,3,5,7) bits to standard output");

	//Set period master clock to 25 MHz clock
	dm7820_status = DM7820_PWM_Set_Period_Master(manipulator_board, DM7820_PWM_MODULATOR_1, DM7820_PWM_PERIOD_MASTER_PROG_CLOCK_0);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period_Master()");

	// Set pulse width modulator period to obtain frequency 25000/PWM_DIVIDER Hz
	dm7820_status = DM7820_PWM_Set_Period(manipulator_board, DM7820_PWM_MODULATOR_1, PWM_HOBBY_SERVO_PERIOD_COUNTS);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period()");

	//Set width master clock to 25 MHz clock
	dm7820_status = DM7820_PWM_Set_Width_Master(manipulator_board, DM7820_PWM_MODULATOR_1, DM7820_PWM_WIDTH_MASTER_PROG_CLOCK_0);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width_Master()");

	//zero out all pwms
	dm7820_status = DM7820_PWM_Set_Width(manipulator_board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_A, PWM_HOBBY_SERVO_MIN_DT + PWM_HOBBY_SERVO_RANGE/2); //0.5ms
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
	dm7820_status = DM7820_PWM_Set_Width(manipulator_board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_B, PWM_HOBBY_SERVO_MIN_DT + PWM_HOBBY_SERVO_RANGE/2);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
	dm7820_status = DM7820_PWM_Set_Width(manipulator_board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_C, PWM_HOBBY_SERVO_MIN_DT + PWM_HOBBY_SERVO_RANGE/2);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
	dm7820_status = DM7820_PWM_Set_Width(manipulator_board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_D, PWM_HOBBY_SERVO_MIN_DT + PWM_HOBBY_SERVO_RANGE/2);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

	//enable PWM 1
	dm7820_status = DM7820_PWM_Enable(manipulator_board, DM7820_PWM_MODULATOR_1, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Enable()");

	/*************************************************************************
	  GENERAL SETUP
	 *************************************************************************/

	//strobe outputs for heartbeat signal
	strobe=0;
	dm7820_status = DM7820_StdIO_Strobe_Mode (manipulator_board, DM7820_STDIO_STROBE_1, 1);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Mode()");
	dm7820_status = DM7820_StdIO_Strobe_Mode (manipulator_board, DM7820_STDIO_STROBE_2, 1);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Mode()");
	// dm7820_status = DM7820_StdIO_Strobe_Mode (board, DM7820_STDIO_STROBE_1, 1);
	// DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Mode()");
	// dm7820_status = DM7820_StdIO_Strobe_Mode (board, DM7820_STDIO_STROBE_2, 1);
	// DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Mode()");

	count=1;
	output_value=0;
}

void CepheusHW::safeClose()
{
	double thrust[4];

	for(int i=0;i<4;i++)
	{
		thrust[i]=0;
	}
	// bellow commented out, Pelekoudas 2020
	/*for(int i=0; i<8; i++){
		cmd[i]=0;
	}*/
	//writeMotors();
	setThrustPwm(thrust, 0.05, 0.95);

	dm7820_status = DM7820_General_Close_Board(board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_Close_Board()");

	dm7820_status = DM7820_General_Close_Board(manipulator_board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_Close_Board()");

	ROS_WARN("Hardware safely closed");
}
