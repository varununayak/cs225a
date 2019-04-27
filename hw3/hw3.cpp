#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>
#include <fstream>
#include <cmath>

#define QUESTION_1   1
#define QUESTION_2   2
#define QUESTION_3   3
#define QUESTION_4   4


#define Pi 3.14159


// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";
const string robot_name = "PANDA";

// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
// - write
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
const string CONTROLLER_RUNING_KEY = "sai2::cs225a::controller_running";

unsigned long long controller_counter = 0;

int main() {

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.15);
	VectorXd command_torques = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);

	robot->Jv(Jv, link_name, pos_in_link);
	robot->taskInertiaMatrix(Lambda, Jv);
	robot->dynConsistentInverseJacobian(J_bar, Jv);
	robot->nullspaceMatrix(N, Jv);

	Vector3d x;//quantity to store current task space position
	Vector3d xdot;//quantiy to store current task space velocity

	//Quantities for gravity compensation and coriolis/cfugal compensation
	Eigen::VectorXd g(dof); // Empty Gravity Vector
	Eigen::VectorXd b(dof); // coriolis and cfugal vector


	double dt = 0.001; // 1/freq

	//open file to store csv
	std::ofstream traj_file;
	traj_file.open("/media/varun/Work/Academics/_Spring 2019/CS 225A/cs225a_hw3/data2g.csv");


	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	redis_client.set(CONTROLLER_RUNING_KEY, "1");
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		robot->updateModel();

		// **********************
		// WRITE YOUR CODE AFTER
		// **********************
		int controller_number = QUESTION_2;  

		robot->gravityVector(g); //update gravity vector
		robot->coriolisForce(b); // update coriolis/cfugal vector
		robot->Jv(Jv, link_name, pos_in_link);
		robot->taskInertiaMatrix(Lambda, Jv);
		robot->dynConsistentInverseJacobian(J_bar, Jv);
		robot->nullspaceMatrix(N, Jv);
		robot->position(x,link_name,pos_in_link); //position of end effector
		robot->linearVelocity(xdot,link_name,pos_in_link); //velocity of end effector
		auto q = robot->_q;
		auto qdot = robot->_dq;

		//define trajectory vector
		VectorXd x_desired_traj(3);

		//
		VectorXd x_desired(3);

		//joint limits 
		VectorXd q_upper(7); VectorXd q_lower(7);
		q_upper << 165,100,165,-30,165,210,165; q_upper *= Pi/180.0;
		q_lower << -165,-100,-165,-170,-165,0,-165;  q_lower *= Pi/180.0;

		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{
			double kp=100; double kv=20; double kpj = 50; double kvj = 14;	//set the gains

			//set q_desired
			VectorXd q_desired(7);
			q_desired << 0,0,0,0,0,0,0;

			//define initial point of the trajectory
			VectorXd x_desired_traj_init(3);
			x_desired_traj_init << 0.3,0.1,0.5;

			//compute the desired trajectory at time 't' by increment with N dt steps
			
			x_desired_traj(0) = x_desired_traj_init(0) + 0.1*sin(Pi*controller_counter*dt);
			x_desired_traj(1) = x_desired_traj_init(1) + 0.1*cos(Pi*controller_counter*dt);
			x_desired_traj(2) = x_desired_traj_init(0) + 0;

			VectorXd xdot_traj(3);
			xdot_traj(0) = 0.1*Pi*cos(Pi*controller_counter*dt);
			xdot_traj(1) = 0.1*Pi*(-sin(Pi*controller_counter*dt));
			xdot_traj(2) = 0;

			VectorXd xddot_traj(3);
			xddot_traj(0) = 0.1*Pi*Pi*(-sin(Pi*controller_counter*dt));
			xddot_traj(1) = 0.1*Pi*Pi*(-cos(Pi*controller_counter*dt));


			//compute the control torques using the control law:
			command_torques = Jv.transpose()*( Lambda*(xddot_traj -kp*(x-x_desired_traj)-kv*(xdot-xdot_traj) ) ) 
						+ N.transpose()*( -kpj*(q - q_desired) - kvj*(qdot)   ) 
						+g;
		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2)
		{

			double kp=100; double kv=20; double kpj = 50; double kvj = 14;	//set the gains
		

			//desired position
			x_desired << -0.65, -0.45, 0.7;

			//joint limits mid:
			VectorXd qmid(7);
			qmid  = (q_upper+q_lower)*0.5;

			command_torques = Jv.transpose()*( Lambda*(-kp*(x-x_desired) - kv*xdot)  )
						+N.transpose()*(  -kvj*qdot)
						 -kpj*(q-qmid)
						+g; 
		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{

			command_torques.setZero();
		}

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{

			command_torques.setZero();
		}

		//store the trajectory in the csv file (order matters)
		traj_file << x(0) << "," << x(1) << "," << x(2) << "," <<
		x_desired(0) << "," << x_desired(1) << "," << x_desired(2) << "," <<
		q(3) << "," << q(5) << "," <<
		q_upper(3) << "," << q_lower(3) << "," << 
		q_upper(5) << "," << q_lower(5) <<  endl;

		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

	traj_file.close(); //close the output file

	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNING_KEY, "0");

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
