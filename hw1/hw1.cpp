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
#define QUESTION_5   5

// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm_controller.urdf";
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
	//printf(" Size %s \n",to_string(initial_q.size()).c_str()); 

	VectorXd desired_q;
	desired_q.resize(7);
	desired_q << 90,-45,0,-125,0,80,0;
	desired_q = desired_q*3.14159/180;
	//printf(" desired q %s \n", to_string(desired_q(5)).c_str()); 

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	//to store the trajectory
	std::ofstream traj_file;
	traj_file.open("/media/varun/Work/Academics/_Spring 2019/CS 225A/cs225a_hw1/joint_trajectory_6.csv");

	auto m11 = robot->_M(0,0); //for tuning the gains in part 1 and 2 when you do not use the mass matrix in control law

	Eigen::VectorXd g(dof); // Empty Gravity Vector
	Eigen::VectorXd b(dof); // coriolis and cfugal vector

	//for extra credit
	Eigen::VectorXd payLoad(dof); //extra payload vector
	float payload_mass = 2.5; 
	Eigen::VectorXd payload_force(3);
	payload_force(0) = 0; payload_force(1) = 0; payload_force(2) = 9.81*payload_mass;
	std::string ee_link_name = "link7"; // Link of the "Task" or "End Effector"
	Eigen::MatrixXd ee_jacobian(3,dof); // Empty Jacobian Matrix sized to right size
	Eigen::Vector3d ee_pos_in_link = Eigen::Vector3d(0.0, 0.0, 0.17); //positon of mass in link
	Eigen:MatrixXd massCorrection(dof,dof);	// mass correction term
	Eigen::VectorXd bCorrection(dof);	// coriolis and cfugal payload correction term



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

		int controller_number = QUESTION_5;  // change to the controller of the question you want : QUESTION_1, QUESTION_2, QUESTION_3, QUESTION_4, QUESTION_5

		robot->gravityVector(g); // Fill in  gravity vectory
		robot->coriolisForce(b); // Fill in coriolis/cfugal vector

		//for extra credit
		robot->Jv(ee_jacobian,ee_link_name,ee_pos_in_link); //get jacobian
		payLoad = ee_jacobian.transpose()*payload_force;	//compute joint torques due to payload
		massCorrection = payload_mass*ee_jacobian.transpose()*ee_jacobian;	//computing mass correction using standard MM formula


		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{
			double kp = 400.0;      // chose your p gain
			double kv = 2*kp/(sqrt(kp/m11)) ;      // chose your d gain: kv is approx 52
			printf(" Kv = %f \n", kv);

			VectorXd q_desired = desired_q;   // change to the desired robot joint angles for the question
			auto tau = -kp*(robot->_q - desired_q) - kv*robot->_dq; 
			command_torques = tau ;  // change to the control torques you compute
		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2)
		{	
			double kp = 400.0;      // chose your p gain
			double kv = 2*kp/(sqrt(kp/m11)) ;      // chose your d gain: kv is approx 52

			VectorXd q_desired = desired_q;   // change to the desired robot joint angles for the question
			auto tau = -kp*(robot->_q - desired_q) - kv*robot->_dq + g; 
			command_torques= tau;
		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{

			double kp = 400.0;      // chose your p gain
			double kv = 40.0;      // chose your d gain

			VectorXd q_desired = desired_q;   // change to the desired robot joint angles for the question
			auto tau = robot->_M*(-kp*(robot->_q - desired_q) - kv*robot->_dq) + g; 
			command_torques= tau;
		}

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{

			double kp = 400.0;      // chose your p gain
			double kv = 40.0;      // chose your d gain

			VectorXd q_desired = desired_q;   // change to the desired robot joint angles for the question
			auto tau = robot->_M*(-kp*(robot->_q - desired_q) - kv*robot->_dq) + g + b; 
			command_torques= tau;
		}

		// ---------------------------  question 5 ---------------------------------------
		// implementing the extra credit controller here
		if(controller_number == QUESTION_5)
		{

			double kp = 400.0;      // chose your p gain
			double kv = 40.0;      // chose your d gain

			//cout << "Payload: "<< payLoad.transpose() << endl;

			VectorXd q_desired = desired_q;   // change to the desired robot joint angles for the question
			auto tau = (robot->_M + massCorrection )*(-kp*(robot->_q - desired_q) - kv*robot->_dq) + g + b + payLoad; 
			command_torques= tau;
		}
		
		traj_file << robot->_q(0) << "," << robot->_q(2) << "," << robot->_q(3)  <<"\n"; 

		//cout << robot->_q(0) << "," << robot->_q(2) << "," << robot->_q(3) <<  endl; 
		cout<< g.transpose() <<endl;

		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

		if(controller_counter>2000) break;

	}

	traj_file.close();
	printf("Reached Here \n");

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
