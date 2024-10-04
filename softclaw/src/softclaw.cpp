#include <stdio.h>
#include <stdlib.h>

#include <time.h>

#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include "definitions.h"	
#include "qbmove_communications.h"	

using namespace std;

// Macro definitions

const float max_allowed_mm_ = 110;
const float min_allowed_mm_ = 0;
const float max_allowed_tick_ = 4500;
const float min_allowed_tick_ = -2200;

double 	ref_claw(0);
double 	stiff_claw(0);
bool		ref_claw_changed = false;

/*---------------------------------------------------------------------*
*                                                *
*                                                                      *
*----------------------------------------------------------------------*/
void ref_claw__Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
	ref_claw = msg->data[0];
	stiff_claw = msg->data[1];
	ref_claw_changed = true;
}

int16_t convertMillimiters2Tick(const int16_t &mm){
  return (int16_t)(min_allowed_tick_ + (max_allowed_tick_ - min_allowed_tick_)*(mm - max_allowed_mm_)/(min_allowed_mm_ - max_allowed_mm_));
}

int16_t convertTick2Millimiters(const int16_t &tick){
  return (int16_t)(max_allowed_mm_ - (max_allowed_mm_/(max_allowed_tick_ - min_allowed_tick_)*(tick - min_allowed_tick_)));
}

int main(int argc, char **argv)
{   
  comm_settings comm_settings_cl;
  short int 	inputs[2];
	short int 	measurements[3];

	string 			port;
	string 			ref_claw_topic;
	string 			meas_claw_topic;

	double 			run_freq(100);			// Desired rate for claw position reading and setting
	int					IDclaw;

  ros::init(argc,argv,"softclaw");  // Initiate new ROS node

	// NodeHandle is the main access point to communications with the ROS system. The first NodeHandle constructed will fully initialize this node, and the last NodeHandle destructed will close down the node.
	ros::NodeHandle n;  

	n.getParam("port", port);
	n.getParam("ID", IDclaw);
	n.getParam("run_freq", run_freq);
	n.getParam("ref_claw_topic", ref_claw_topic);
	n.getParam("meas_claw_topic", meas_claw_topic);

	ros::Publisher 	pub_meas_claw	= n.advertise<std_msgs::Float64MultiArray>(meas_claw_topic, 1);
	ros::Subscriber sub_ref_claw	= n.subscribe(ref_claw_topic, 1, ref_claw__Callback);

	ros::Rate loop_rate(run_freq);

	// Device preparation
	cout << "\nROS NODE FOR SOFTCLAW WITH ID " << IDclaw << " \n";
	// Opening the port the device is connected to
	cout << "Opening COM port to communicate with the SoftClaw.\n";

	openRS485(&comm_settings_cl, port.c_str());

	// Activate softclaw motors
	cout << "Activating SoftClaw motor.\n";
	ros::Duration(1).sleep();
	commActivate(&comm_settings_cl, IDclaw, 1);
	ros::Duration(0.05).sleep();

	cout << "\nYou can now communicate with SoftClaw using \'" << 
		ref_claw_topic << "\' topic to give closure reference in mm (range " <<
			 min_allowed_mm_ << "-" << max_allowed_mm_ << " mm) and stiffness reference (0-100 %) and \'" << 
		meas_claw_topic << "\' topic to retrieve its actual position in mm.\n";
	std_msgs::Float64MultiArray 	msg_meas_claw;

	while ( ros::ok() )
	{

		/**************** SoftClaw position reading and setting section ***********************/

		msg_meas_claw.data.clear();

		// Small break to allow multiple consecutive COM readings
		ros::Duration(0.00025).sleep();		//250 us sleep
	
		if(commGetMeasurements(&comm_settings_cl, IDclaw, measurements)>0){
     //std::cout << "Shaft position in [mm] is --> " << convertTick2Millimiters(measurements[2]);   
			msg_meas_claw.data.push_back( convertTick2Millimiters(measurements[2]) );		// Consider shaft position
		}

		// Publish softclaw position
		pub_meas_claw.publish(msg_meas_claw);


		if (ref_claw_changed){

			if (ref_claw > max_allowed_mm_) {
		    ref_claw = max_allowed_mm_;
		  } else if(ref_claw < min_allowed_mm_) {
		    ref_claw = min_allowed_mm_;
		  }
			ref_claw = convertMillimiters2Tick(ref_claw);  

		  if (stiff_claw > 100) {
		    stiff_claw = 100;
		  } else if(stiff_claw < 0) {
		    stiff_claw = 0;
		  }
		  stiff_claw *= 320;	//from percentage to tick

			inputs[0] = ref_claw;
	  	inputs[1] = stiff_claw;
	  	
	  	// Small break to allow multiple consecutive COM readings
			ros::Duration(0.00025).sleep();		//250 us sleep
	  		
	  	commSetPosStiff(&comm_settings_cl, IDclaw, inputs);

			ref_claw_changed = false;
		}

		ros::spinOnce();     // Need to call this function often to allow ROS to process incoming messages 
		loop_rate.sleep();   // Sleep for the rest of the cycle, to enforce the loop rate
		
	}

	commActivate(&comm_settings_cl, IDclaw, 0);
	ros::Duration(0.05).sleep();
	closeRS485(&comm_settings_cl);
	return 0;

}

