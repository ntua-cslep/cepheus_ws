
/*
	Classes for velocities profiles and goemetric constraints used in planner and constants regarding the robot
*/

const double FMAX_THRUST = 0.17;//Newton 20% duty cucle
const double CHASER_MASS = 13.5;//kg
const double WS_RADIUS = 0.385;//m
const double ROBOT_RADIUS = 0.15;//m
const double ASSIST_ROBOT_DIST_FROM_CENTER = 0.4;//m
const double CIRCLE_RADIUS = 0.4;//m radius of the circular object around the target

#define VEL_PROF_1 1
#define VEL_PROF_2 2
#define VEL_PROF_3 3

//the duration after the Tmeet, at which the base ctrl will be disabled and the arm will start moving to capture the target
const double TIME_TO_DISABLE_CTRL = 3.0; //seconds


class Geometric_Constraints{

        double table_length_x;
        double table_length_y;
        double robot_radious;
        double robot_ws;
        double assist_robot_length;//it's square

        double constraint_for_x;//the constraint derived from the above
        double constraint_for_y;

        public:

        Geometric_Constraints(double x,
                        double y,
                        double r,
                        double ws,
                        double arl)
                :table_length_x(x), table_length_y(y), robot_radious(r), robot_ws(ws), assist_robot_length(arl)
        {
                double abs_d = robot_radious + robot_ws + assist_robot_length;

                constraint_for_x = table_length_x - abs_d;
                constraint_for_y = table_length_y - abs_d;

        }

        bool in_constraints(double des_x, double des_y){

                if(des_x < constraint_for_x && des_y < constraint_for_y)
                        return true;
                else
                        return false;

        }
};

//Structs containing the nessesary information for every velocity profile used
typedef struct Prf1{

        double t1;
        double t2;
        double xdes_target;
	double v0_chaser;
        double xt1;
        double vt1;
        double xdes_chaser;
        double duration_with_active_ctrl;

        void set_vals(  double t1,
                        double t2,
                        double xdes_target,
			double v0_chaser,
                        double xt1,
                        double vt1,
                        double xdes_chaser)
        {
                this->t1 = t1;
                this->t2 = t2;
                this->xdes_target = xdes_target;
		this->v0_chaser = v0_chaser;
                this->xt1 = xt1;
                this->vt1 = vt1;
                this-> xdes_chaser =  xdes_chaser;
                this->duration_with_active_ctrl = t1 + (t2 - t1) + TIME_TO_DISABLE_CTRL;
        }

        void print(){

                ROS_INFO("Profile 1 Params:");
                std::cout<<"\t t1: "<<t1<<std::endl;
                std::cout<<"\t t2: "<<t2<<std::endl;
		std::cout<<"\t v0_chaser: "<<v0_chaser<<std::endl;
                std::cout<<"\t xt1: "<<xt1<<std::endl;
                std::cout<<"\t vt1: "<<vt1<<std::endl;
                std::cout<<"\t xdes_chaser: "<<xdes_chaser<<std::endl;
                std::cout<<"\t xdes_target: "<<xdes_target<<std::endl;

                std::cout<<"\t Total Time: "<<duration_with_active_ctrl<<"\n"<<std::endl;
        }
}Prf1;

typedef struct Prf2{

        double t1;
        double a_ch;
        double xdes_chaser;
        double duration_with_active_ctrl;

        void set_vals(  double t1,
                        double xdes_chaser,
                        double a_ch)
        {
                this->t1 = t1;
                this->xdes_chaser = xdes_chaser;
                this-> a_ch = a_ch;
                this->duration_with_active_ctrl = t1 + TIME_TO_DISABLE_CTRL;
        }

        void print(){

                ROS_INFO("Profile 2 Params:");
                std::cout<<"\t t1: "<<t1<<std::endl;
                std::cout<<"\t xdes_chaser: "<<xdes_chaser<<std::endl;
                std::cout<<"\t a_ch: "<<a_ch<<std::endl;

                std::cout<<"\t Total Time: "<<duration_with_active_ctrl<<"\n"<<std::endl;
        }

}Prf2;

typedef struct Prf3{

        double t1;
        double t2;
        double t3;
        double a3;
        double Vt1;
        double Xt1;
        double Xt2;
        double xdes_chaser;
        double xdes_target;
        double duration_with_active_ctrl;

        void set_vals(  double t1,
                        double t2,
                        double t3,
                        double a3,
                        double Vt1,
                        double Xt1,
                        double Xt2,
                        double xdes_chaser,
                        double xdes_target)
        {
                this->t1 = t1;
                this->t2 = t2;
                this->t3 = t3;
                this->a3 = a3;
                this->Xt1 = Xt1;
                this->Xt2 = Xt2;
                this->Vt1 = Vt1;
                this-> xdes_chaser =  xdes_chaser;
                this->xdes_target = xdes_target;
                this->duration_with_active_ctrl = t1 + (t2 - t1) + (t3 - t2) + TIME_TO_DISABLE_CTRL;
        }

        void print(){

                ROS_INFO("Profile 3 Params:");
                std::cout<<"\t t1: "<<t1<<std::endl;
                std::cout<<"\t t2: "<<t2<<std::endl;
                std::cout<<"\t t3: "<<t3<<std::endl;
                std::cout<<"\t a3: "<<a3<<std::endl;
                std::cout<<"\t Xt1: "<<Xt1<<std::endl;
                std::cout<<"\t Xt2: "<<Xt2<<std::endl;
                std::cout<<"\t Vt1: "<<Vt1<<std::endl;
                std::cout<<"\t xdes_chaser: "<<xdes_chaser<<std::endl;
                std::cout<<"\t xdes_target: "<<xdes_target<<std::endl;

                std::cout<<"\t Total Time: "<<duration_with_active_ctrl<<"\n"<<std::endl;

        }

}Prf3;

