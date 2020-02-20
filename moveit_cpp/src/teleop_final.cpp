#include "/opt/ros/indigo/include/ros/ros.h"
#include "/opt/ros/indigo/include/std_msgs/String.h"

#include </opt/ros/indigo/include/moveit/move_group_interface/move_group.h>
#include </opt/ros/indigo/include/moveit/planning_scene_interface/planning_scene_interface.h>
#include </opt/ros/indigo/include/moveit_msgs/DisplayRobotState.h>
#include </opt/ros/indigo/include/moveit_msgs/DisplayTrajectory.h>
#include </opt/ros/indigo/include/moveit_msgs/AttachedCollisionObject.h>
#include </opt/ros/indigo/include/moveit_msgs/CollisionObject.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <sstream>
#include <cmath>
#include <stdio.h>
#include <signal.h>
#include <termios.h>
#include <unistd.h>


using namespace std;


int kfd = 0;
struct termios cooked, raw;

int getch(){

	int ch;
	struct termios buf;
	struct termios save;

	tcgetattr(0, &save);
	buf = save;
	buf.c_lflag &= ~(ICANON | ECHO);
	buf.c_cc[VMIN] = 1;
	buf.c_cc[VTIME] = 0;

	tcsetattr(0, TCSAFLUSH, &buf);
	ch = getchar();
	tcsetattr(0, TCSAFLUSH, &save);
	return ch;

}


void toEulerDegreeAngle(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw)
{
	// roll (x-axis rotation)
	double sinr = 2.0 * (q.w * q.x + q.y * q.z);
	double cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
	roll = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = 2.0 * (q.w * q.y - q.z * q.x);
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny = 2.0 * (q.w * q.z + q.x * q.y);
	double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
	yaw = atan2(siny, cosy);

	//convert from radian to degree
	roll = roll / M_PI * 180;
	pitch = pitch / M_PI * 180;
	yaw = yaw / M_PI * 180;
}

void toRadQuaternion(geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw){

	//convert to radian
	roll = roll * M_PI / 180;
	pitch = pitch * M_PI / 180;
	yaw = yaw * M_PI / 180;

	// Abbreviations for the various angular functions
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

	q.w = cy * cr * cp + sy * sr * sp;
	q.x = cy * sr * cp - sy * cr * sp;
	q.y = cy * cr * sp + sy * sr * cp;
	q.z = sy * cr * cp - cy * sr * sp;

	q.w = q.w / (q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
	q.x = q.x / (q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
	q.y = q.y / (q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
	q.z = q.z / (q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
}




inline float SIGN(float x) {
	return (x >= 0.0f) ? +1.0f : -1.0f;
}

inline float NORM(float a, float b, float c, float d) {
	return sqrt(a * a + b * b + c * c + d * d);
}

// quaternion = [w, x, y, z]'
void mRot2Quat(const tf::Matrix3x3& m, geometry_msgs::Quaternion & quat) {
	float r11 = m[0][0];
	float r12 = m[0][1];
	float r13 = m[0][2];
	float r21 = m[1][0];
	float r22 = m[1][1];
	float r23 = m[1][2];
	float r31 = m[2][0];
	float r32 = m[2][1];
	float r33 = m[2][2];
	float q0 = (r11 + r22 + r33 + 1.0f) / 4.0f;
	float q1 = (r11 - r22 - r33 + 1.0f) / 4.0f;
	float q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
	float q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;
	if (q0 < 0.0f) {
		q0 = 0.0f;
	}
	if (q1 < 0.0f) {
		q1 = 0.0f;
	}
	if (q2 < 0.0f) {
		q2 = 0.0f;
	}
	if (q3 < 0.0f) {
		q3 = 0.0f;
	}
	q0 = sqrt(q0);
	q1 = sqrt(q1);
	q2 = sqrt(q2);
	q3 = sqrt(q3);
	if (q0 >= q1 && q0 >= q2 && q0 >= q3) {
		q0 *= +1.0f;
		q1 *= SIGN(r32 - r23);
		q2 *= SIGN(r13 - r31);
		q3 *= SIGN(r21 - r12);
	}
	else if (q1 >= q0 && q1 >= q2 && q1 >= q3) {
		q0 *= SIGN(r32 - r23);
		q1 *= +1.0f;
		q2 *= SIGN(r21 + r12);
		q3 *= SIGN(r13 + r31);
	}
	else if (q2 >= q0 && q2 >= q1 && q2 >= q3) {
		q0 *= SIGN(r13 - r31);
		q1 *= SIGN(r21 + r12);
		q2 *= +1.0f;
		q3 *= SIGN(r32 + r23);
	}
	else if (q3 >= q0 && q3 >= q1 && q3 >= q2) {
		q0 *= SIGN(r21 - r12);
		q1 *= SIGN(r31 + r13);
		q2 *= SIGN(r32 + r23);
		q3 *= +1.0f;
	}
	else {
		printf("coding error\n");
	}
	float r = NORM(q0, q1, q2, q3);
	q0 /= r;
	q1 /= r;
	q2 /= r;
	q3 /= r;

	quat.x = q0;
	quat.y = q1;
	quat.z = q2;
	quat.w = q3;

}


void quit(int sig)
{
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
	exit(0);
}



int main(int argc, char **argv){

	ros::init(argc, argv, "moveit_teleop_node");
	ros::NodeHandle n;
	ros::AsyncSpinner spinner(1);  // start a background "spinner", so our node can process ROS messages
	spinner.start();
	sleep(5.0);

	ROS_INFO("Delay ended\n");

	moveit::planning_interface::MoveGroup group("arm");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


	ros::Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;


	//ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
	//group.setPoseReferenceFrame("wrist_roll_link");
	//ROS_INFO("planning frame after setting pose reference frame: %s", group.getPlanningFrame().c_str());
	//ROS_INFO("Reference frame after setting pose reference frame: %s", group.getPoseReferenceFrame().c_str());
	//ROS_INFO("End effector frame: %s", group.getEndEffectorLink().c_str());


	tf::TransformListener listener;
	tf::Quaternion base_grip_tf_quat;
	tf::Quaternion tf_quat_temp;
	tf::Matrix3x3 base_grip_matrix;
	tf::Vector3 diff_vec;
	tf::Vector3 diff_base;
	tf::Matrix3x3 rot_matrix;

	geometry_msgs::Pose target_pose1;
	geometry_msgs::Quaternion get_quat;


	target_pose1 = group.getCurrentPose("wrist_roll_link").pose; 

	group.setPoseTarget(target_pose1);

	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.plan(my_plan);


	display_trajectory.trajectory_start = my_plan.start_state_;
	display_trajectory.trajectory.push_back(my_plan.trajectory_);
	display_publisher.publish(display_trajectory);
	/* Sleep to give Rviz time to visualize the plan. */

	sleep(2.0);
	group.move();


	double pos_diff = 0.05;
	double angle_diff = 15;

	while (ros::ok()){
		tf::StampedTransform transform;

		try{
			// listener.lookupTransform("/gripper_link", "/base_link", ros::Time(0), transform);
			listener.lookupTransform("/base_link", "/gripper_link", ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}

		base_grip_tf_quat = transform.getRotation();
		base_grip_tf_quat.normalize();

		base_grip_matrix = transform.getBasis();


		int ch;
		ch = getch();

		target_pose1 = group.getCurrentPose("wrist_roll_link").pose;


		double e_x, e_y, e_z;
		float p_x, p_y, p_z;

		switch (ch){

		case 117:
			ROS_INFO("ee frame x position +");
			diff_vec[0] = pos_diff;
			diff_vec[1] = 0;
			diff_vec[2] = 0;

			diff_base = diff_vec * (base_grip_matrix.inverse());

			target_pose1.position.x += diff_base[0];
			target_pose1.position.y += diff_base[1];
			target_pose1.position.z += diff_base[2];

			group.setPoseTarget(target_pose1);
			success = group.plan(my_plan);

			group.move();
			//ROS_INFO("Visualizing plan x up (position goal) %s", success ? "" : "FAILED");
			break;



		case 106:
			ROS_INFO("ee frame x position -");
			diff_vec[0] = (-1)*pos_diff;
			diff_vec[1] = 0;
			diff_vec[2] = 0;

			diff_base = diff_vec * base_grip_matrix.inverse();

			target_pose1.position.x += diff_base[0];
			target_pose1.position.y += diff_base[1];
			target_pose1.position.z += diff_base[2];    //translation

			group.setPoseTarget(target_pose1);
			success = group.plan(my_plan);

			group.move();
			//ROS_INFO("Visualizing plan x down (position goal) %s", success ? "" : "FAILED");
			break;


		case 105:
			ROS_INFO("ee frame y position +");
			diff_vec[0] = 0;
			diff_vec[1] = pos_diff;
			diff_vec[2] = 0;

			diff_base = diff_vec * base_grip_matrix.inverse();

			target_pose1.position.x += diff_base[0];
			target_pose1.position.y += diff_base[1];
			target_pose1.position.z += diff_base[2];

			group.setPoseTarget(target_pose1);
			success = group.plan(my_plan);

			group.move();
			//ROS_INFO("Visualizing plan y up (position goal) %s", success ? "" : "FAILED");
			break;


		case 107:
			ROS_INFO("ee frame y position -");
			diff_vec[0] = 0;
			diff_vec[1] = (-1)*pos_diff;
			diff_vec[2] = 0;

			diff_base = diff_vec * base_grip_matrix.inverse();

			target_pose1.position.x += diff_base[0];
			target_pose1.position.y += diff_base[1];
			target_pose1.position.z += diff_base[2];

			group.setPoseTarget(target_pose1);
			success = group.plan(my_plan);

			group.move();
			//ROS_INFO("Visualizing plan y down (position goal) %s", success ? "" : "FAILED");
			break;


		case 111:
			ROS_INFO("z position +");
			diff_vec[0] = 0;
			diff_vec[1] = 0;
			diff_vec[2] = pos_diff;

			diff_base = diff_vec * base_grip_matrix.inverse();

			target_pose1.position.x += diff_base[0];
			target_pose1.position.y += diff_base[1];
			target_pose1.position.z += diff_base[2];

			group.setPoseTarget(target_pose1);
			success = group.plan(my_plan);

			group.move();
			//ROS_INFO("Visualizing plan z up (position goal) %s", success ? "" : "FAILED");
			break;


		case 108:
			ROS_INFO("z position -");
			diff_vec[0] = 0;
			diff_vec[1] = 0;
			diff_vec[2] = (-1)*pos_diff;

			diff_base = diff_vec * base_grip_matrix.inverse();

			target_pose1.position.x += diff_base[0];
			target_pose1.position.y += diff_base[1];
			target_pose1.position.z += diff_base[2];

			group.setPoseTarget(target_pose1);
			success = group.plan(my_plan);

			group.move();
			//ROS_INFO("Visualizing plan z down (position goal) %s", success ? "" : "FAILED");
			break;


		case 101:
			//e
			ROS_INFO("x angle +");

			e_x = angle_diff; e_y = 0; e_z = 0;
			toRadQuaternion(get_quat, e_x, e_y, e_z);

			tf_quat_temp.setX(get_quat.x);
			tf_quat_temp.setY(get_quat.y);
			tf_quat_temp.setZ(get_quat.z);
			tf_quat_temp.setW(get_quat.w);    //to quaternion

			tf_quat_temp = base_grip_tf_quat * tf_quat_temp;    //w.r.t. base frame
			get_quat.x = tf_quat_temp.x();
			get_quat.y = tf_quat_temp.y();
			get_quat.z = tf_quat_temp.z();
			get_quat.w = tf_quat_temp.w();


			target_pose1.orientation = get_quat;

			group.setPoseTarget(target_pose1);
			success = group.plan(my_plan);

			group.move();
			break;
		case 100:
			ROS_INFO("x angle -");

			e_x = -angle_diff; e_y = 0; e_z = 0;
			toRadQuaternion(get_quat, e_x, e_y, e_z);

			tf_quat_temp.setX(get_quat.x);
			tf_quat_temp.setY(get_quat.y);
			tf_quat_temp.setZ(get_quat.z);
			tf_quat_temp.setW(get_quat.w);    //to quaternion

			tf_quat_temp = base_grip_tf_quat * tf_quat_temp;    //w.r.t. base frame
			get_quat.x = tf_quat_temp.x();
			get_quat.y = tf_quat_temp.y();
			get_quat.z = tf_quat_temp.z();
			get_quat.w = tf_quat_temp.w();


			target_pose1.orientation = get_quat;

			group.setPoseTarget(target_pose1);
			success = group.plan(my_plan);

			group.move();
			break;
		case 114:
			ROS_INFO("y angle +");

			e_x = 0; e_y = angle_diff; e_z = 0;
			toRadQuaternion(get_quat, e_x, e_y, e_z);

			tf_quat_temp.setX(get_quat.x);
			tf_quat_temp.setY(get_quat.y);
			tf_quat_temp.setZ(get_quat.z);
			tf_quat_temp.setW(get_quat.w);    //to quaternion

			tf_quat_temp = base_grip_tf_quat * tf_quat_temp;    //w.r.t. base frame
			get_quat.x = tf_quat_temp.x();
			get_quat.y = tf_quat_temp.y();
			get_quat.z = tf_quat_temp.z();
			get_quat.w = tf_quat_temp.w();

			target_pose1.orientation = get_quat;

			group.setPoseTarget(target_pose1);
			success = group.plan(my_plan);

			group.move();
			break;
		case 102:
			ROS_INFO("y angle -");

			e_x = 0; e_y = -angle_diff; e_z = 0;
			toRadQuaternion(get_quat, e_x, e_y, e_z);

			tf_quat_temp.setX(get_quat.x);
			tf_quat_temp.setY(get_quat.y);
			tf_quat_temp.setZ(get_quat.z);
			tf_quat_temp.setW(get_quat.w);    //to quaternion

			tf_quat_temp = base_grip_tf_quat * tf_quat_temp;    //w.r.t. base frame
			get_quat.x = tf_quat_temp.x();
			get_quat.y = tf_quat_temp.y();
			get_quat.z = tf_quat_temp.z();
			get_quat.w = tf_quat_temp.w();


			target_pose1.orientation = get_quat;

			group.setPoseTarget(target_pose1);
			success = group.plan(my_plan);

			group.move();
			break;
		case 116:
			ROS_INFO("z angle +");
			e_x = 0; e_y = 0; e_z = angle_diff;
			toRadQuaternion(get_quat, e_x, e_y, e_z);

			tf_quat_temp.setX(get_quat.x);
			tf_quat_temp.setY(get_quat.y);
			tf_quat_temp.setZ(get_quat.z);
			tf_quat_temp.setW(get_quat.w);    //to quaternion

			tf_quat_temp = base_grip_tf_quat * tf_quat_temp;    //w.r.t. base frame
			get_quat.x = tf_quat_temp.x();
			get_quat.y = tf_quat_temp.y();
			get_quat.z = tf_quat_temp.z();
			get_quat.w = tf_quat_temp.w();


			target_pose1.orientation = get_quat;

			group.setPoseTarget(target_pose1);
			success = group.plan(my_plan);

			group.move();
			break;
		case 103:
			ROS_INFO("z angle -");
			e_x = 0; e_y = 0; e_z = -angle_diff;
			toRadQuaternion(get_quat, e_x, e_y, e_z);

			tf_quat_temp.setX(get_quat.x);
			tf_quat_temp.setY(get_quat.y);
			tf_quat_temp.setZ(get_quat.z);
			tf_quat_temp.setW(get_quat.w);    //to quaternion

			tf_quat_temp = base_grip_tf_quat * tf_quat_temp;    //w.r.t. base frame
			get_quat.x = tf_quat_temp.x();
			get_quat.y = tf_quat_temp.y();
			get_quat.z = tf_quat_temp.z();
			get_quat.w = tf_quat_temp.w();


			target_pose1.orientation = get_quat;

			group.setPoseTarget(target_pose1);
			success = group.plan(my_plan);

			group.move();
			break;

		case 118:
			//v, position increase
			ROS_INFO("position diff increase");
			pos_diff += 0.01;
			if (pos_diff >= 0.1)
				pos_diff = 0.1;
			ROS_INFO("%f", pos_diff);
			break;
		case 98:
			//b, position decrease
			ROS_INFO("position diff decrease");
			pos_diff -= 0.01;
			if (pos_diff <= 0.01)
				pos_diff = 0.01;
			ROS_INFO("%f", pos_diff);
			break;
		case 110:
			//n, angle increase
			ROS_INFO("angle diff increase");
			angle_diff += 5;
			if (angle_diff >= 20)
				angle_diff = 20;
			ROS_INFO("%f", angle_diff);
			break;
		case 109:
			//m, angle decrease
			ROS_INFO("angle diff decrease");
			angle_diff -= 5;
			if (angle_diff <= 5)
				angle_diff = 5;
			ROS_INFO("%f", angle_diff);
			break;

		}

		signal(SIGINT, quit);

	}


	ros::shutdown();
	return 0;
}