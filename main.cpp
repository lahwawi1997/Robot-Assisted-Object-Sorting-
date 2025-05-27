/** 
* This code doesnt consider the time delay caused by the communication and the solving process;
* The failure to solve is considered but it is assumed the time it costs is ignorable even if it fails to solve;
**/



#include "batch_solver.h"
#include <iostream>
#include "ros/ros.h"
#include "utility.h"
#include <visualization_msgs/Marker.h>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include "mymsg/neighborpos.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include <Eigen/Dense>
#include <signal.h>
#include <std_msgs/Float32.h>

# define PI 3.14159265358979323846

size_t N = 25;
size_t m = 2;// represent the num of neighbors for this vehicle
double Hz = 2.0;
size_t update_shift = 0;

double xr;
double yr;
double thetar ;

double xr_1 =0;
double yr_1 =0;
double thetar1 ;

double xr_2 = 0;
double yr_2 = 0;
double thetar2 ;

std::mutex neig_mtx;
std::mutex init_mtx;


double xinit ;
double yinit ;
double thetainit ;

double x;
double y;


// ******protected by neig_mtx10
std::vector<std::vector<std::vector<double>>> neig;
 

std::vector<std::vector<double>> pre_states(N+1,std::vector<double>(3.0,0.0));
std::vector<std::vector<double>> pre_inputs(N+1,std::vector<double>(2.0,0.0));


bool solve_success = false;

// each vehicle may have different solving time for the intial solution;
// some of them might be quite long; So these flags are used for synchronous purpose;
bool first_solution_v1 = false;
bool first_solution_v2 = false;

double d = 0.11;
double ts = 0.5;//1.0 / Hz;//0.3
double safety_dist = 0.15;

Eigen::Matrix4d Twc; // camera frame in world frame

std::vector<std::vector<double>> obst;

std::shared_ptr<BatchSolver> bs(new BatchSolver(N,xr, yr, thetar, d, xinit,yinit, thetainit, ts, safety_dist,obst,neig));

ros::Publisher vehicle_pub;// for visulization
ros::Publisher markerArray;
ros::Publisher neig_pub;// tells other robots my pos
ros::Publisher control_pub;
ros::Subscriber sub1;
ros::Subscriber sub2;
ros::Subscriber localization_sub;


void UpdateVisualize();
void UpdateNeighborsPos();
void Initialize(ros::NodeHandle& n);
void NeighborCallback1(const mymsg::neighborpos& msg);
void NeighborCallback2(const mymsg::neighborpos& msg);
void LocalizationCallback(const apriltag_ros::AprilTagDetectionArray& msg);


////////////////////////////////We added the below part
void xcallback(const std_msgs::Float32::ConstPtr&msg)
{
	x = msg->data;
	ROS_INFO("Received x : %f", msg->data);
	// std::cout<<"x_python : "<<x<<std::endl;

}

void ycallback(const std_msgs::Float32::ConstPtr&msg)
{
	y = msg->data;
	ROS_INFO("Received y : %f", msg->data);

}
////////////////////////////////////////////////////



void mySigintHandler(int sig)
{
	std::cout << "Experiment stopped by user. Stopping Robot..." << std::endl;
	geometry_msgs::Vector3 msg_con;	
	msg_con.x = 0;
	msg_con.y = 0;
	msg_con.z = 0.0;
	control_pub.publish(msg_con);
	std::cout<<"Check point mySigintHandler"<<std::endl;
	std::cout << "THE END." << std::endl;
	ros::shutdown();
}

int main(int argc,char* argv[]){
	ros::init(argc,argv,"vehicle03");
	ros::NodeHandle n;
	ros::NodeHandle nh;
	signal(SIGINT,mySigintHandler);
   	ros::Rate loop_rate(2);
	// std::cout<<"n = "<< n <<std:: endl;
	Initialize(n);



	std::thread sim_thread(&UpdateVisualize);
	sim_thread.detach();
	// std::cout<<"Check point main"<<std::endl;


	while(ros::ok()){
		// std::cout<<"Check point while loop main"<<std::endl;
		/////////////////////////Below part is written by us
		std::cout << "Before subscriber" << std::endl;
		ros::Subscriber x_sub = nh.subscribe("x_topic", 10, xcallback);
		ros::Subscriber y_sub = nh.subscribe("y_topic", 10, ycallback);
		// std::cout << "x_value"<< x <<std::endl;
		std::cout << "After subscriber" << std::endl;
		/////////////////////////////////////////////
		ros::spinOnce();
		loop_rate.sleep();
	}
	geometry_msgs::Vector3 msg_con;
	msg_con.x = 0.0;
	msg_con.y = 0.0;
	msg_con.z = 0.0;
	control_pub.publish(msg_con);
	return 0;
};

void Initialize(ros::NodeHandle& n){

	xinit=0.17;yinit=0.77;thetainit=PI/2;
	xr=0.70;yr=-0.62;thetar=PI/2;
	std::vector<double> obst1 = {0.0,0.0};	
	std::vector<std::vector<double>> neig1(N+1,std::vector<double>{0.5,0.25});//v1
	std::vector<std::vector<double>> neig2(N+1,std::vector<double>{-0.5,0.35});//v2
	//obst.push_back(obst1);
	neig.push_back(neig1);	
	neig.push_back(neig2);	
	bs->set_obst_(obst);
	bs->set_ref_states(xr,yr,thetar);
	bs->set_initial_states(xinit,yinit,thetainit);
	bs->set_neighbors(neig,neig_mtx);

	// std::cout<<"Check point Initialize"<<std::endl;

	vehicle_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    markerArray = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
	neig_pub= n.advertise<mymsg::neighborpos>("neig3pos", 10);
	control_pub = n.advertise<geometry_msgs::Vector3>("control_signal3",10);
        sub1 = n.subscribe("neig1pos", 10, NeighborCallback1);
        sub2 = n.subscribe("neig2pos", 10, NeighborCallback2);
        localization_sub = n.subscribe("tag_detections", 10, LocalizationCallback);



	// x 180
	Twc << 1.0 , 0.0 , 0.0, 0.0, 
	      0.0 , cos(PI) , -sin(PI),0.0,
	      0.0 , sin(PI) , cos(PI),1.27,
	      0.0,0.0,0.0,1.0;

};
void UpdateVisualize(){

	ros::Rate loop_rate_sim(Hz);
	// std::cout<<"Check point visualize"<<std::endl;
	ros::spinOnce();
	loop_rate_sim.sleep();

	while(ros::ok()){
		ObstRviz(obst,safety_dist,markerArray);
		TrajRviz(pre_states, safety_dist,markerArray);
		if(1){	
			std::lock_guard<std::mutex> lk(init_mtx);
			VehicleRviz(xinit,yinit,thetainit,safety_dist,vehicle_pub);
			HeadingRviz(xinit,yinit,thetainit,safety_dist,vehicle_pub);
			bs->set_initial_states(xinit,yinit,thetainit);
			// std::cout<<"Check point while if(1) visualize"<<std::endl;
		}

			
		bs->set_neighbors(neig,neig_mtx);
		bs->Solve(pre_states,pre_inputs,solve_success);

		//while(first_solution_v1 == false){ std::cout<<"ddddd"<<std::endl;};

		if(solve_success){
			update_shift = 0;

			std::vector<double> pre_x;
			std::vector<double> pre_y;
			for(int i = 0 ; i < pre_states.size(); i++){
				pre_x.push_back(pre_states[i][0]);
				pre_y.push_back(pre_states[i][1]);
			}
   			mymsg::neighborpos msg;
   			msg.xpos = pre_x;
   			msg.ypos = pre_y;
    			msg.time_stamp = ros::Time::now().toSec();
			neig_pub.publish(msg);

			geometry_msgs::Vector3 msg_con;
			msg_con.x = pre_inputs[0][0];
			msg_con.y = pre_inputs[0][1];
			msg_con.z = 0.0;
			control_pub.publish(msg_con);


		}else{// if fail to solve, publish the shifted pre_states
			std::cout<<" fail to solve!!!!!"<<std::endl;
			exit(0);

		}

		ros::spinOnce();// send the solution ASAP after the solving
		loop_rate_sim.sleep();

	
	}
	std::cout << "Experiment stopped by user. Stopping Robot..." << std::endl;
	geometry_msgs::Vector3 msg_con;	
	msg_con.x = 0;
	msg_con.y = 0;
	msg_con.z = 0.0;
	control_pub.publish(msg_con);
	std::cout << "THE END." << std::endl;

};

void NeighborCallback1(const mymsg::neighborpos& msg)
{
std::cout<<"Data retrieved from robot 1!"<<std::endl;
// std::cout<<"Check point neighborcallback1"<<std::endl;
	std::vector<double> x = msg.xpos;
	std::vector<double> y = msg.ypos;
	std::lock_guard<std::mutex> lk(neig_mtx);
	for(size_t i = 1; i< neig[0].size();i++){
		neig[0][i-1][0] = x[i];
		neig[0][i-1][1] = y[i];
	}
	neig[0][neig[0].size()-1][0] = neig[0][neig[0].size()-2][0];
	neig[0][neig[0].size()-1][1] = neig[0][neig[0].size()-2][1];
	first_solution_v1 = true;
};
void NeighborCallback2(const mymsg::neighborpos& msg)
{
std::cout<<"Data retrieved from robot 2!"<<std::endl;
// std::cout<<"Check point neighborcallback2"<<std::endl;
	std::vector<double> x = msg.xpos;
	std::vector<double> y = msg.ypos;
	std::lock_guard<std::mutex> lk(neig_mtx);
	for(int i = 1; i< neig[0].size();i++){
		neig[1][i-1][0] = x[i];
		neig[1][i-1][1] = y[i];
		

	}
	neig[1][neig[1].size()-1][0] = neig[1][neig[1].size()-2][0];
	neig[1][neig[1].size()-1][1] = neig[1][neig[1].size()-2][1];	
	first_solution_v2 = true;
};
void LocalizationCallback(const apriltag_ros::AprilTagDetectionArray& msg){
	// std::cout<<"Check point localization callback"<<std::endl;

	// ros::Subscriber x_sub = nh.subscribe("x_topic", 10, xcallback);
	// ros::Subscriber y_sub = nh.subscribe("y_topic", 10, ycallback);
	// std::cout << "x_value"<< x <<std::endl;


	// if(xinit <= xr-0.05 & yinit <= yr+0.05){
	// 			xr_1=0.9;yr_1=0.45;thetar1=PI/2;
	// 			bs->set_ref_states(xr_1,yr_1,thetar1);
	// 			// std::cout<<"xr_1 : "<<xr_1<<", yr_1 : "<<yr_1<<", thetar_1 : "<<thetar1*180/PI<<std::endl;
	// 		}
	// if(xinit >= xr_1-0.04 & yinit >= yr_1+0.04){
	// 			xr_2=0.70;yr_2=-0.62;thetar1=PI/2;
	// 			bs->set_ref_states(xr_2,yr_2,thetar2);
	// 			// std::cout<<"xr_2 : "<<xr_2<<", yr_2 : "<<yr_2<<", thetar_2 : "<<thetar2*180/PI<<std::endl;
	// 		}

	

	if(msg.detections.size() == 0) return;
        for(size_t i = 0; i < msg.detections.size() ; i++){
		if(msg.detections[i].id[0] == 6){
                        //std::cout<<"minions"<<std::endl;
			Eigen::Quaterniond q(msg.detections[i].pose.pose.pose.orientation.w,msg.detections[i].pose.pose.pose.orientation.x,\
			msg.detections[i].pose.pose.pose.orientation.y,msg.detections[i].pose.pose.pose.orientation.z) ;
			Eigen::Matrix4d Tct = Eigen::Matrix4d::Zero();
			Tct.block<3,3>(0,0) =  q.toRotationMatrix();
			Tct(0,3) = msg.detections[i].pose.pose.pose.position.x;
			Tct(1,3) = msg.detections[i].pose.pose.pose.position.y;
			Tct(2,3) = msg.detections[i].pose.pose.pose.position.z;	
			Tct(3,3) = 1.0;

			Eigen::Matrix4d Twt = Twc * Tct;
			double xtag = Twt(0,3);
			double ytag = Twt(1,3);

			// std::cout<<"Check point for loop localization callback"<<std::endl;
			std::lock_guard<std::mutex> lk(init_mtx);	
			thetainit = atan2(Twt(1,0),Twt(0,0));
			xinit = xtag - 0.07 * cos(thetainit);
			yinit = ytag - 0.07 * sin(thetainit);
			// std::cout<<"x : "<<xinit<<", y : "<<yinit<<", theta : "<<thetainit*180/PI<<std::endl;
			// std::cout<<"x_tag : "<<xtag<<", y_tag : "<<ytag<<std::endl;
			

                }

			// if(xinit <= xr-0.05 & yinit <= yr+0.05){
			// 	xr_1=0.9;yr_1=0.45;thetar1=PI/2;
			// 	bs->set_ref_states(xr_1,yr_1,thetar1);
			// 	std::cout<<"xr_1 : "<<xr_1<<", yr_1 : "<<yr_1<<", thetar_1 : "<<thetar1*180/PI<<std::endl;
			// }

			// 	if(xinit <= xr_1-0.04 & yinit <= yr_1+0.04){
			// 	xr_2=0.70;yr_2=-0.62;thetar1=PI/2;
			// 	bs->set_ref_states(xr_2,yr_2,thetar2);
			// 	std::cout<<"xr_2 : "<<xr_2<<", yr_2 : "<<yr_2<<", thetar_2 : "<<thetar2*180/PI<<std::endl;
			// }
			// if(xinit == xr_1-0.04 & yinit == yr_1+0.04){
			// 	xr_2=0.70;yr_2=-0.62;thetar1=PI/2;
			// 	bs->set_ref_states(xr_2,yr_2,thetar2);
			// 	std::cout<<"xr_2 : "<<xr_2<<", yr_2 : "<<yr_2<<", thetar_2 : "<<thetar2*180/PI<<std::endl;
			// }


        }
	// xinit=0.17 ; yinit=0.77 %%% xr=0.70 ; yr=-0.62

	if(xinit <= xr-0.05 & yinit <= yr+0.05){
				xr_1=x;yr_1=y;thetar1=PI/2;
				bs->set_ref_states(xr_1,yr_1,thetar1);
				std::cout<<"xr_1 : "<<xr_1<<", yr_1 : "<<yr_1<<", thetar_1 : "<<thetar1*180/PI<<std::endl;
				std::cout<<"first if"<<std::endl;
			}
// && xr_1+0.04 && yr_1-0.04
	// if(xinit <= xr_1-0.04 && xinit >= xr_1+0.04 & yinit >= yr_1+0.04 && yinit <= yr_1-0.04){
	if(xr_1<x-0.04 & xr_1>x+0.04 & yr_1<y-0.04 & yr_2>y+0.04){
				xr_2=-0.60;yr_2=-0.7;thetar2=PI/2;
				bs->set_ref_states(xr_2,yr_2,thetar2);
				std::cout<<"xr_2 : "<<xr_2<<", yr_2 : "<<yr_2<<", thetar_2 : "<<thetar2*180/PI<<std::endl;
				std::cout<<"second if"<<std::endl;
			}

	// if(xinit <= xr_2-0.04 & yinit <= yr_2+0.04){
	// 			xr=0.70;yr=-0.62;thetar=PI/2;
	// 			bs->set_ref_states(xr,yr,thetar);
	// 			// std::cout<<"xr_2 : "<<xr_2<<", yr_2 : "<<yr_2<<", thetar_2 : "<<thetar2*180/PI<<std::endl;
	// 		}
};



