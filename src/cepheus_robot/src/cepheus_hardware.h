#include <errno.h>
#include <error.h>
#include <signal.h>

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <dm7820_library.h>


#define LIMIT_L1 1 // limits switch pin in port 0
#define LIMIT_L2 3 // limits switch pin in port 0

#define LIMIT_R1 6 // limits switch pin in port 0
#define LIMIT_R2 7 // limits switch pin in port 0


#define FIR_LENGTH 2 
#define PWM_MOTOR_FREQ 5000
#define PWM_HOBBY_SERVO_FREQ 50 //Period is usually 20ms
#define PWM_THRUSTER_FREQ 10
#define ADC_PWM_FREQ 25000

#define PWM_MOTOR_PERIOD_COUNTS (25000000/PWM_MOTOR_FREQ)
#define PWM_MOTOR_RANGE (PWM_MOTOR_PERIOD_COUNTS-(PWM_MOTOR_PERIOD_COUNTS/5)) //PWM range must be 10%-90% for escon drivers so 80% of full range
#define PWM_MOTOR_MIN_DT (PWM_MOTOR_PERIOD_COUNTS/10)
#define PWM_MOTOR_MAX_DT (PWM_MOTOR_PERIOD_COUNTS - (PWM_MOTOR_PERIOD_COUNTS/10))

#define PWM_HOBBY_SERVO_PERIOD_COUNTS (2500000/PWM_HOBBY_SERVO_FREQ)
#define PWM_HOBBY_SERVO_RANGE  4750//PWM range must be 0.6ms-2.4ms = 1.8ms for the servos
#define PWM_HOBBY_SERVO_MIN_DT 1250 //0.6ms for period of 1/25MHz 
#define PWM_HOBBY_SERVO_MAX_DT 6000 //2.5ms for period of 1/25MHz 

#define ADC_PWM_PERIOD_COUNTS (25000000/ADC_PWM_FREQ)

#define MOVE_DURATION 2

//-----Panagiotis Mavridis 18/09/2018--------

//The number of force sensors used (left and right gripper)
#define FSR_NUM 2

//--Model: TO DO put website

//A servo motor expects to be updated every 20 ms

//------------------------------------------------------------
//Fingers Servos Info:
//The servo used here expects the duration of the pulse to be from 0.9 ms to 2.1 ms
//The minimum duty cycle is 0.9/20 = 4.5%
//The maximum duty cycle is 2.1/20 = 10.5%

#define LEFT_FINGER_INIT_ANGLE 60
#define LEFT_FINGER_MAX_ANGLE 120
#define LEFT_FINGER_MIN_ANGLE 0

#define RIGHT_FINGER_INIT_ANGLE 60
#define RIGHT_FINGER_MAX_ANGLE 120
#define RIGHT_FINGER_MIN_ANGLE 0

//PWM_FINGER_SERVO_MIN_DT = min duty cycle *  PWM_HOBBY_SERVO_PERIOD_COUNTS 
#define PWM_FINGER_SERVO_MIN_DT 2250 

//PWM_FINGER_SERVO_MAX_DT = max duty cycle *  PWM_HOBBY_SERVO_PERIOD_COUNTS
#define PWM_FINGER_SERVO_MAX_DT 5250 

#define PWM_FINGER_SERVO_RANGE  3000//PWM range must be 2.1ms - 0.9ms = 1.2ms for the servos

//-------------------------------

//------------------------------------------------------------
//Wrists Servos Info:
//The servo used here expects the duration of the pulse to be from 0.9 ms to 2.1 ms
//The minimum duty cycle is 0.9/20 = 4.5%
//The maximum duty cycle is 2.1/20 = 10.5%

#define LEFT_WRIST_INIT_ANGLE 60 //max_angle/2
#define LEFT_WRIST_MAX_ANGLE 120
#define LEFT_WRIST_MIN_ANGLE 0

#define RIGHT_WRIST_INIT_ANGLE 60 //max_angle/2
#define RIGHT_WRIST_MAX_ANGLE 120
#define RIGHT_WRIST_MIN_ANGLE 0
//PWM_WRIST_SERVO_MIN_DT = min duty cycle *  PWM_HOBBY_SERVO_PERIOD_COUNTS 
#define PWM_WRIST_SERVO_MIN_DT 2250 

//PWM_WRIST_SERVO_MAX_DT = max duty cycle *  PWM_HOBBY_SERVO_PERIOD_COUNTS
#define PWM_WRIST_SERVO_MAX_DT 5250 

#define PWM_WRIST_SERVO_RANGE  3000//PWM range must be 0.9ms - 2.1ms = 1.2ms for the servos

//-------------------------------


enum ManipulatorMapping{
		
	REACTION_WHEEL = 0,	//0
	unknown1,		//1
	unknown2,		//2
	unknown3,		//3
	LEFT_SHOULDER,		//4
	LEFT_ELBOW,		//5
	RIGHT_SHOULDER,		//6
	RIGHT_ELBOW,		//7
	LEFT_WRIST,		//8
	RIGHT_WRIST,		//9
	LEFT_GRIPPER,		//10
	RIGHT_GRIPPER		//11
};


class CepheusHW : public hardware_interface::RobotHW
{
	public:
		void heartbeat();
		void enable();
		void disable();
		void setThrustPwm(double*, double, double);
		void writeMotors();
		bool isLimitReached(int i);
		void setHomePos(int i, float val);
		void setJointTorque(int sh, int elb);
		void readLimitSwitches();
		uint8_t init_3();
		void readEncoders(ros::Duration);
		void setParam(double*, double);
		void setCmd(int,double);
		double getVel(int);
		void safeClose();
		CepheusHW();

		//Panagiotis mavridis
		void init_left_shoulder();
		void init_left_elbow();
		void init_left_finger();
		void init_left_wrist();

		void init_right_shoulder();
		void init_right_elbow();
		void init_right_finger();
		void init_right_wrist();
		
		void command_right_wrist();

		void set_left_finger(double);
		void set_left_wrist(double);
		void set_right_finger(double);
		void set_right_wrist(double);


		void write_left_wrist(double);
		void write_left_finger(double);
		void write_right_wrist(double);
		void write_right_finger(double);

		void update_shoulder(double, double, double&);
		void update_elbow(double, double, double&);
		// 2020 Pelekoudas
		void update_left_elbow(double, double, double&, double kp);
		void update_right_elbow(double, double, double&, double kp);

		void set_left_fsr_value(uint8_t val){
			fsr_values[0] = val;
		}

		void set_right_fsr_value(uint8_t val){
			fsr_values[1] = val;
		}


		uint8_t get_left_fsr_val(){
			return fsr_values[0];
		}

		uint8_t get_right_fsr_val(){
			return fsr_values[1];
		}


		void set_manipulator_width(int manipulator, uint16_t width_val){
			
			if(manipulator >=0 && manipulator <=12)
				width[manipulator] = width_val;
			else
				ROS_WARN("Cannot set width!. Manipulator %d does not exist!", manipulator);
		}
	
		double getCmd(int i){
			return cmd[i];
		}
	
		double getPos(int i){
			return pos[i];
		}
	
		void setOffset(int i, double offset){
			offset_pos[i] = offset;
		}

	private:
		bool homing(int, float);
		/***controller manager interface***/
		hardware_interface::JointStateInterface jnt_state_interface;
		hardware_interface::EffortJointInterface jnt_eff_interface;
		hardware_interface::PositionJointInterface jnt_pos_interface;
		

		enum motors{ rw, l1, l2, r1, r2}; 

		/*****Motors parameters*********/
		double max_current[8];
		double max_thrust;
		/*****SERVOS parameters*********/
		double force[4];

		//Panagiotis Mavridis
		uint8_t fsr_values[FSR_NUM];

		/******motors********/
		int limit[8];
		double cmd[12];
		uint16_t width[12];
		uint16_t dir[10];
		double current[8];
		double home_pos[8];
		double pos[8], prev_pos[8], offset_pos[8];
		double vel[8];
		double eff[8];
		double servo_pos[4];
		double vel_new[8];
		double vel_fir[FIR_LENGTH][8];
		//encoder values from card
		uint16_t encoder_1_val;
		uint16_t encoder_2_val;
		uint16_t encoder_3_val;
		uint16_t encoder_4_val;
		uint16_t encoder_5_val;
		uint16_t encoder_6_val;
		uint16_t encoder_7_val;
		uint16_t encoder_8_val;
		//encoder overflow counters
		int encoder_1_ovf;
		int encoder_2_ovf;
		int encoder_3_ovf;
		int encoder_4_ovf;
		int encoder_5_ovf;
		int encoder_6_ovf;
		int encoder_7_ovf;
		int encoder_8_ovf;
		//encoder values to transmit
		int encoder_1;
		int encoder_2;
		int encoder_3;
		int encoder_4;
		int encoder_5;
		int encoder_6;
		int encoder_7;
		int encoder_8;
		/******thrusters********/
		uint32_t period;

		/*******IO board********/
		DM7820_Board_Descriptor *board;
		DM7820_Board_Descriptor *manipulator_board;
		DM7820_Error dm7820_status;
		uint8_t encoder_status;
		dm7820_incenc_phase_filter phase_filter;
		int status;


		int count;
		uint16_t output_value;
		bool strobe;
};
