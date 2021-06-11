#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <geometry_msgs/WrenchStamped.h>

#include <std_msgs/Empty.h>


using namespace std;

double average_forces[3];
double average_torques[3];
double sum_of_values[6];
double num_of_available;

void callback_1(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    num_of_available += 1;
	geometry_msgs::WrenchStamped temp;
	temp = *msg;
    cout << num_of_available << ": " << temp.wrench.force.x << endl;
    sum_of_values[0] += temp.wrench.force.x;
    sum_of_values[1] += temp.wrench.force.y;
    sum_of_values[2] += temp.wrench.force.z;
    sum_of_values[3] += temp.wrench.torque.x;
    sum_of_values[4] += temp.wrench.torque.y;
    sum_of_values[5] += temp.wrench.torque.z;
}

int main(int argn, char* args[])
{
    ros::init(argn, args, "callback_q_subscriber");
    ros::NodeHandle nh_1;
    ros::CallbackQueue queue_1;
	ros::Rate loop_rate(200);

    nh_1.setCallbackQueue(&queue_1);

    ros::Subscriber s_1 = nh_1.subscribe("/rokubimini/ft_sensor0/ft_sensor_readings/wrench", 100, callback_1);

    sleep(2);
    int i = 20;
    while(ros::ok()) {
        num_of_available = 0;
        for (int i=0; i < 3; i++)
            sum_of_values[i] = 0;
        queue_1.callAvailable(ros::WallDuration(1.0));
        average_forces[0] = sum_of_values[0] / num_of_available;
        average_forces[1] = sum_of_values[1] / num_of_available;
        average_forces[2] = sum_of_values[2] / num_of_available;
        average_torques[0] = sum_of_values[3] / num_of_available;
        average_torques[1] = sum_of_values[4] / num_of_available;
        average_torques[2] = sum_of_values[5] / num_of_available;
        cout << "average_forces: " << average_forces[0] << " " << average_forces[1] << " " << average_forces[2] << endl;
        cout << "average_torques: " << average_torques[0] << " " << average_torques[1] << " " << average_torques[2] << endl << endl;
		loop_rate.sleep();
    }

    return 0;   
}