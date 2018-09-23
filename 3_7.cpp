#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc, char** argv)
{
	Eigen::Quaterniond q1 = Eigen::Quaterniond(0.35,0.2,0.3,0.1); // for tuple number for robot 1
	Eigen::Vector3d p_c1(0.5,0.0,0.2); // coordinate of point in the view of robot1 
	Eigen::Vector3d t1(0.3,0.1,0.1); // shift vector from global coordinate to robot 1

	// R1*p_w+t1 = p_c1 we need the p_w
	q1.normalize(); // q1: w,x,y,z

	Eigen::Matrix3d rotation_1 = q1.toRotationMatrix();// get rotation matrix(R1) from the four tuple number
	Eigen::Vector3d pw = rotation_1.inverse()*(p_c1-t1); //get global coordinate
	
	
	Eigen::Quaterniond q2 = Eigen::Quaterniond(-0.5,0.4,-0.1,0.2);// for tuple number for robot2
	q2.normalize();

	Eigen::Matrix3d rotation_2 = q2.toRotationMatrix();// R2 the rotation matrix of robot2
	Eigen::Vector3d t2(-0.1,0.5,0.3);// shift vector of robot2 

	Eigen::Vector3d p_c2 = rotation_2*pw+t2;
	cout<<p_c2<<endl;

	Eigen::Vector3d p_c2_straight = rotation_2*rotation_1.inverse()*(p_c1-t1)+t2;
	cout<<p_c2_straight<<endl;
	
	return 0;
}
