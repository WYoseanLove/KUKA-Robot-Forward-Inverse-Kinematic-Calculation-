// Kuka_Robot_Kinematic_Calculation.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include<Eigen\Dense>

#define M_PI 3.1415926

double rx = 0;
double ry = 0;
double rz = 0;

double Tx = 0;
double Ty = 0;
double Tz = 0;  // define the TCP frame coordinate, the value for TCP is according to the base frame;

double J1 = 0;
double J2 = 0;
double J3 = 0;
double J4 = 0;
double J5 = 0;
double J6 = 0; // define the Robot 6 joint value to calculate the TCP frame;



bool isRotationMatrix(Eigen::Matrix3d R)
{
	double err = 1e-6;

	Eigen::Matrix3d shouldIdenity;
	shouldIdenity = R * R.transpose();

	Eigen::Matrix3d I = Eigen::Matrix3d::Identity();


	bool result = (shouldIdenity - I).norm() < err;
	return result;


}

Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R)
{
	assert(isRotationMatrix(R));
	double sy = sqrt(R(0, 0)*R(0, 0) + R(1, 0)*R(1, 0));
	bool singlular = sy < 1e-6;
	double x, y, z;
	if (!singlular)
	{
		x = atan2(R(2, 1), R(2, 2));
		y = atan2(-R(2, 0), sy);
		z = atan2(R(1, 0), R(0, 0));


	}
	else
	{
		x = atan2(-R(1, 2), R(1, 1));
		y = atan2(-R(2, 0), sy);
		z = 0;

	}

	return { x,y,z };


}


int IK_KUKA(double* a1, double* d1, double* a2, double* a3, double* d4, double* d5, double* TCPX, double* TCPY, double* TCPZ, double* TCPRX, double* TCPRY, double* TCPRZ)
{
	// print J1-J6 as the calculation result for Robot inverse kinematic;
	J1 = 0;
	J2 = 0;
	J3 = 0;
	J4 = 0;
	J5 = 0;
	J6 = 0;

	// End effector TCP to End effector flange frame;
	// The translation matrix is defined by the end effector, need calculate the flange frame from the TCP;
	// flange frame will be same the Robot J6 frame in flange;
	// TCP need to translate to flange frame after the direction is common between TCP/ FLANGE Frame;
	// the example showing need to move X=-502, Y=0, Z=905.61, all the data is according to the original TCP aix;
	// Rotation will be happened after translation happened;
	// TCP rotate Z aix 90 deg;
	// then Rotate Y aix 0 deg;
	// finally rotate X aix 90 deg;
	// the TCP frame will be same direction with Tool flange frame;
	
	Eigen::Vector3d rpy_raw;

	rpy_raw << rx, ry, rz;

	rpy_raw = rpy_raw * M_PI / 180;

	Eigen::MatrixXd RX, RY, RZ, R, RTCP;
	RX = Eigen::MatrixXd(3, 3);
	RY = Eigen::MatrixXd(3, 3);
	RZ = Eigen::MatrixXd(3, 3);
	R = Eigen::MatrixXd(3, 3);
	RTCP = Eigen::MatrixXd(4, 4);

	RX << 1, 0, 0,
		0, cos(rpy_raw[0]), -sin(rpy_raw[0]),
		0, sin(rpy_raw[0]), cos(rpy_raw[0]);


	RY << cos(rpy_raw[1]), 0, sin(rpy_raw[1]),
		0, 1, 0,
		-sin(rpy_raw[1]), 0, cos(rpy_raw[1]);

	RZ << cos(rpy_raw[2]), -sin(rpy_raw[2]), 0,
		sin(rpy_raw[2]), cos(rpy_raw[2]), 0,
		0, 0, 1;

	R = RZ * RY*RX;

	RTCP << R(0, 0), R(0, 1), R(0, 2), Tx,
		R(1, 0), R(1, 1), R(1, 2), Ty,
		R(2, 0), R(2, 1), R(2, 2), Tz,
		0, 0, 0, 1; 
	//define the TCP frame coordinate according to the base frame；

	// from the end effector frame to inverse calculate the J6 Flange frame value;
	rpy_raw << *TCPRX, *TCPRY, *TCPRZ;
	rpy_raw = rpy_raw * M_PI / 180;
	Eigen::Isometry3d R_Tool = Eigen::Isometry3d::Identity();
	R_Tool = (Eigen::AngleAxisd(rpy_raw[2], Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(rpy_raw[1], Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(rpy_raw[0], Eigen::Vector3d::UnitX()));
	R_Tool.pretranslate(Eigen::Vector3d(*TCPX, *TCPY, *TCPZ));//在这里平移需要放到轴的坐标上去,相对于旋转之前的坐标系方向

	Eigen::MatrixXd RJ_6;
	RJ_6 = Eigen::MatrixXd(4, 4);
	RJ_6 = RTCP * R_Tool.matrix();

	// a11, a12, a13, x0,
	// a21, a22, a23, y0,
	// a31, a32, a33, z0,
	//   0,   0,   0,  1;


	double a11 = RJ_6(0, 0);
	double a12 = RJ_6(0, 1);
	double a13 = RJ_6(0, 2);
	double x0 = RJ_6(0, 3);

	double a21 = RJ_6(1, 0);
	double a22 = RJ_6(1, 1);
	double a23 = RJ_6(1, 2);
	double y0 = RJ_6(1, 3);

	double a31 = RJ_6(2, 0);
	double a32 = RJ_6(2, 1);
	double a33 = RJ_6(2, 2);
	double z0 = RJ_6(2, 3);

	//A01_inverse* RJ_6*A56_inverse =A15  

	// A01_inverse* RJ_6*A56_inverse=
	// a31*c6+a32*s6,                            -s6*a31+a32*c6,                         a33,            z0-1045,
	// c6*c1*a11-c6*s1*a21+s6*a12*c1-s1*s6*a22,  -s6*(c1*a11-a21*s1)+c6*(a12*c1-s1*a22), c1*a13-s1*a23,  c1*x0-s1*y0-500,
	// (s1*a11+c1*a21)*c6+s6*(a12*s1+a22*c1),   -s6*(s1*a11+c1*a21)+c6*(s1*a12+c1*a22), s1*a13+c1*a23,  s1*x0+c1*y0,
	//                                    0,                                        0,             0,            1;

	//A15 << -c23 * c4*c5 + s23 * s5,   -c23 * s4,    c23*c4*s5 + s23 * c5,    290 * s5*c23*c4 + 290 * c5*s23 - 55 * c23 + 1275 * s23 + 1300 * c2,
	//      s23*c4*c5 + c23 * s5,       s23*s4,      -s23 * c4*s5 + c23 * c5, -290 * s5*s23*c4 +290 * c5*c23 + 55 * s23 + 1275 * c23 - 1300 * s2,
	//       -s4 * c5,                    c4,         s4*s5,                    290 * s5*s4,
	//       0,                           0,          0,                        1;

	// A15(2,3)/A15(2,2)=290;
	// (s1*x0+c1*y0)/(s1*a13+c1*a23)=290;
	// tanJ1=(290*a23-y0)/(x0-290*a13);

	J1 = atan2(((*d5) * a23 - y0), (x0 - (*d5) * a13));

	// calculate the J5_Location to support J2/J3 Calculation;
	Eigen::Vector3d RJ_6_Location;
	RJ_6_Location << x0, y0, z0;

	Eigen::Vector3d w(0, 0, 1);
	Eigen::MatrixXd RJ_6_Oritation;
	RJ_6_Oritation = Eigen::MatrixXd(3, 3);

	RJ_6_Oritation << a11, a12, a13,
		a21, a22, a23,
		a31, a32, a33;

	Eigen::Vector3d J5_Location;

	//J5_Location = TCP - 290 * TCP_Oritation*w;
	J5_Location = RJ_6_Location - (*d5) * RJ_6_Oritation*w;

	// RJ_5 Frame matrix as below :
	//-c1 * s23*c4 + s1 * s4, c1*c23, c1*s23*s4 + s1 * c4, 55 * c1*s23 + 1275 * c1*c23 + 500 * c1 - 1300 * c1*s2,
	//s1*s23*c4 + c1 * s4, -s1 * c23, c1*c4 - s1 * s23*s4, -55 * s1*s23 - 1275 * s1*c23 - 500 * s1 + 1300 * s1*s2,
	//c23*c4, s23, -c23 * s4, -55 * c23 + 1275 * s23 + 1045 + 1300 * c2,
	//0, 0, 0, 1;
	//J5_Location(0,0)=55 * c1*s23 + 1275 * c1*c23 + 500 * c1 - 1300 * c1*s2;
	//J5_Location(1,0)=-55 * s1*s23 - 1275 * s1*c23 - 500 * s1 + 1300 * s1*s2;
	//J5_Location(2,0)= -55 * c23 + 1275 * s23 + 1045 + 1300 * c2;
	//J1 = (J1)*M_PI / 180;
	// 55*s23+1275*c23-1300*s2=J5_Location(0,0)/(cos(J1))-500;
	// 55*c23-1275*s23-1300*c2=-J5_Location(2,0)+1045;

	//double a_temp= J5_Location(0, 0) / (cos(J1)) - 500;
	//double b_temp = -J5_Location(2, 0) + 1045;
	//double c_temp = (55 * 55 + 1275 * 1275 - a_temp * a_temp - b_temp * b_temp - 1300 * 1300) / (2 * 1300);


	double a_temp = J5_Location(0, 0) / (cos(J1)) - (*a1);
	double b_temp = -J5_Location(2, 0) + *d1;
	double c_temp = ((*a3) * (*a3) + (*d4) * (*d4) - a_temp * a_temp - b_temp * b_temp - (*a2) *(*a2)) / (2 * (*a2));

	// J2 calculation have two different solutions, sqrt can be +/-;
	// for the 1st and 2nd solution will be keep sqrt result is  + ;
	if (a_temp > 0)
		J2 = asin(c_temp / (sqrt(a_temp*a_temp + b_temp * b_temp))) - atan2(b_temp, a_temp);

	J3 = asin(((*a3)*(a_temp + (*a2)*sin(J2)) - (*d4)*(b_temp + (*a2)*cos(J2))) / ((*a3) *(*a3) + (*d4) * (*d4))) - J2;

	// below are calculate J4/J5/J6
	
	double c1 = cos(J1);
	double s1 = sin(J1);
	double c2 = cos(J2);
	double s2 = sin(J2);
	double c3 = cos(J3);
	double s3 = sin(J3);


	Eigen::MatrixXd A01;
	A01 = Eigen::MatrixXd(4, 4);

	A01 << 0, c1, s1, (*a1) * c1,
		0, -s1, c1, -(*a1) * s1,
		1, 0, 0, (*d1),
		0, 0, 0, 1;

	Eigen::MatrixXd A12;
	A12 = Eigen::MatrixXd(4, 4);

	A12 << c2, s2, 0, (*a2) * c2,
		-s2, c2, 0, -(*a2) * s2,
		0, 0, 1, 0,
		0, 0, 0, 1;

	Eigen::MatrixXd A23;
	A23 = Eigen::MatrixXd(4, 4);

	A23 << c3, 0, s3, -(*a3) * c3 + (*d4) * s3,
		-s3, 0, c3, (*a3) * s3 + (*d4) * c3,
		0, -1, 0, 0,
		0, 0, 0, 1;


	Eigen::MatrixXd A03;
	A03 = Eigen::MatrixXd(4, 4);

	A03 = A01 * A12*A23;
	Eigen::MatrixXd A36;
	A36 = Eigen::MatrixXd(4, 4);

	A36 = (A03.inverse())*RJ_6;

	//A36=A34*A45*A56;
	//A36<< -c4*c5*c6+s4*s6,   -c4*c5*s6-s4*c6,  c4*s5,  (*d5)*c4*s5, 
	//       s4*c5*c6+c4*s6,   s4*c5*s6-c4*c6,   -s4*s5, -(*d5)*s4*s5,
	//       s5*c6         ,   s5*s6,            c5,     (*d5)*c5,
	//           0,                 0,            0,      1;



	J4 = atan2(-A36(1, 2), A36(0, 2));
	J5 = atan2(sqrt(A36(0, 2)*A36(0, 2) + A36(1, 2)*A36(1, 2)), A36(2, 2));
	J6 = atan2(A36(2, 1), A36(2, 0));


	std::cout << "1st solution :" << std::endl;
	std::cout << J1 * 180 / M_PI << std::endl;
	std::cout << (-(J2) * 180 / M_PI) - 90 << std::endl;
	std::cout << (-(J3) * 180 / M_PI) + 90 << std::endl;
	std::cout << J4 * 180 / M_PI << std::endl;
	std::cout << (-J5 * 180 / M_PI) << std::endl;
	std::cout << (J6 * 180 / M_PI) << std::endl;
	std::cout << std::endl;
	std::cout << "2nd solution :" << std::endl;

	J4 = atan2(-A36(1, 2), -A36(0, 2));
	J5 = atan2(-sqrt(A36(0, 2)*A36(0, 2) + A36(1, 2)*A36(1, 2)), A36(2, 2));
	J6 = atan2(-A36(2, 1), -A36(2, 0));


	std::cout << J1 * 180 / M_PI << std::endl;
	std::cout << (-(J2) * 180 / M_PI) - 90 << std::endl;
	std::cout << (-(J3) * 180 / M_PI) + 90 << std::endl;
	std::cout << (2 * M_PI - J4) * 180 / M_PI << std::endl;
	std::cout << (-J5 * 180 / M_PI) << std::endl;
	std::cout << (J6 * 180 / M_PI) << std::endl;

	std::cout << std::endl;

	//  calculate the 3rd and 4th solustions

	if (a_temp > 0)
		J2 = asin(c_temp / (-sqrt(a_temp*a_temp + b_temp * b_temp))) - atan2(b_temp, a_temp);
	J3 = asin(((*a3)*(a_temp - (*a2)*sin(J2)) - (*d4)*(b_temp - (*a2)*cos(J2))) / ((*a3) *(*a3) + (*d4) * (*d4))) - J2;

	c1 = cos(J1);
	s1 = sin(J1);
	c2 = cos(J2);
	s2 = sin(J2);
	c3 = cos(J3);
	s3 = sin(J3);


	A01 << 0, c1, s1, (*a1) * c1,
		0, -s1, c1, -(*a1) * s1,
		1, 0, 0, (*d1),
		0, 0, 0, 1;



	A12 << c2, s2, 0, (*a2) * c2,
		-s2, c2, 0, -(*a2) * s2,
		0, 0, 1, 0,
		0, 0, 0, 1;



	A23 << c3, 0, s3, -(*a3) * c3 + (*d4) * s3,
		-s3, 0, c3, (*a3) * s3 + (*d4) * c3,
		0, -1, 0, 0,
		0, 0, 0, 1;




	A03 = A01 * A12*A23;


	A36 = (A03.inverse())*RJ_6;

	//A36=A34*A45*A56;
	//A36<< -c4*c5*c6+s4*s6,   -c4*c5*s6-s4*c6,  c4*s5,  290*c4*s5, 
	//       s4*c5*c6+c4*s6,   s4*c5*s6-c4*c6,   -s4*s5, -290*s4*s5,
	//       s5*c6         ,   s5*s6,            c5,     290*c5,
	//           0,                 0,            0,      1;



	J4 = atan2(-A36(1, 2), A36(0, 2));
	J5 = atan2(sqrt(A36(0, 2)*A36(0, 2) + A36(1, 2)*A36(1, 2)), A36(2, 2));
	J6 = atan2(A36(2, 1), A36(2, 0));

	std::cout << "3rd solution :" << std::endl;
	std::cout << J1 * 180 / M_PI << std::endl;
	std::cout << (-(J2) * 180 / M_PI) + 90 << std::endl;
	std::cout << (-(J3) * 180 / M_PI) - 90 << std::endl;
	std::cout << J4 * 180 / M_PI << std::endl;
	std::cout << (-J5 * 180 / M_PI) << std::endl;
	std::cout << (J6 * 180 / M_PI) << std::endl;

	std::cout << std::endl;

	std::cout << "4th solution :" << std::endl;

	J4 = atan2(-A36(1, 2), -A36(0, 2));
	J5 = atan2(-sqrt(A36(0, 2)*A36(0, 2) + A36(1, 2)*A36(1, 2)), A36(2, 2));
	J6 = atan2(-A36(2, 1), -A36(2, 0));


	std::cout << J1 * 180 / M_PI << std::endl;
	std::cout << (-(J2) * 180 / M_PI) + 90 << std::endl;
	std::cout << (-(J3) * 180 / M_PI) - 90 << std::endl;
	std::cout << (2 * M_PI - J4) * 180 / M_PI << std::endl;
	std::cout << (-J5 * 180 / M_PI) << std::endl;
	std::cout << (J6 * 180 / M_PI) << std::endl;

	std::cout << std::endl;


	return 0;


}

int KW_KUKA(double* a1, double* d1, double* a2, double* a3, double* d4, double* d5, double* TCPX, double* TCPY, double* TCPZ, double* TCPRX, double* TCPRY, double* TCPRZ)
{
	// a1,  d1,  a2,  a3,  d4,  d5 is the Robot DH parameters;
	//TCPX, TCPY, TCPZ, TCPRX, TCPRY,  TCPRZ is Tool end effector flange frame to TCP frame, TCPRX, TCPRY, TCPRZ shoud be deg not radian

	Eigen::MatrixXd RT;
	RT = Eigen::MatrixXd(4, 4);

	// from the Base frame to calculate the J1 Joint location and rotation frame 
	// from the base frame to J1 frame will be through below steps:
	// 1. base Frame should be move *D1 along with Z aix direction;
	// 2. base frame should be move *a1 along with X aix direction;
	// 3. base frame should be rotate -90 deg  with Z aix;
	// 4. base frame should be rotate -90 deg with Y aix;
	// after 4 steps, the base frame will be at J1 location and same direction with J1;
	Eigen::Vector3d rpy_raw, ypr;
	rpy_raw << 0, -90, -90;
	rpy_raw = rpy_raw * M_PI / 180;
	Eigen::Isometry3d R_Joint1 = Eigen::Isometry3d::Identity();
	R_Joint1 = (Eigen::AngleAxisd(rpy_raw[2], Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(rpy_raw[1], Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(rpy_raw[0], Eigen::Vector3d::UnitX()));
	R_Joint1.pretranslate(Eigen::Vector3d((*a1), 0, (*d1)));//

	// If J1 rotate deg, what is the final J1 martix, can be got from below calculation：

	Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
	T1.rotate((Eigen::AngleAxisd((J1 - 0)*M_PI / 180, Eigen::Vector3d::UnitZ())).toRotationMatrix());//机械手各个轴的旋转角度，在这里是旋转了45度
	T1.pretranslate(Eigen::Vector3d(0, 0, 0));

	RT = (T1.matrix()).inverse()*(R_Joint1.matrix());

	//same with Base frame to J1 frame, below calculation is J1 to J2 frame；

	rpy_raw << 0, 0, 0;
	rpy_raw = rpy_raw * M_PI / 180;
	Eigen::Isometry3d R_Joint2 = Eigen::Isometry3d::Identity();
	R_Joint2 = (Eigen::AngleAxisd(rpy_raw[2], Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(rpy_raw[1], Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(rpy_raw[0], Eigen::Vector3d::UnitX()));
	
	R_Joint2.pretranslate(Eigen::Vector3d(1300, 0, 0));

	Eigen::Isometry3d T2 = Eigen::Isometry3d::Identity();
	T2.rotate((Eigen::AngleAxisd(-(J2 + 90)*M_PI / 180, Eigen::Vector3d::UnitZ())).toRotationMatrix());

	T2.pretranslate(Eigen::Vector3d(0, 0, 0));

	RT = RT * (T2.matrix()).inverse()*(R_Joint2.matrix());
	

	//same with Base frame to J1 frame, below calculation is J2 to J3 frame；
	rpy_raw << -90, 0, 0;
	rpy_raw = rpy_raw * M_PI / 180;
	Eigen::Isometry3d R_Joint3 = Eigen::Isometry3d::Identity();
	R_Joint3 = (Eigen::AngleAxisd(rpy_raw[2], Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(rpy_raw[1], Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(rpy_raw[0], Eigen::Vector3d::UnitX()));
	R_Joint3.pretranslate(Eigen::Vector3d(-55, 1275, 0));


	Eigen::Isometry3d T3 = Eigen::Isometry3d::Identity();
	T3.rotate((Eigen::AngleAxisd(-(J3 - 90)*M_PI / 180, Eigen::Vector3d::UnitZ())).toRotationMatrix());//机械手各个轴的旋转角度，在这里是旋转了45度
	T3.pretranslate(Eigen::Vector3d(0, 0, 0));

	RT = RT * (T3.matrix()).inverse()*(R_Joint3.matrix());

	//same with Base frame to J1 frame, below calculation is J3 to J4 frame；

	rpy_raw << 90, 0, 0;
	rpy_raw = rpy_raw * M_PI / 180;
	Eigen::Isometry3d R_Joint4 = Eigen::Isometry3d::Identity();
	R_Joint4 = (Eigen::AngleAxisd(rpy_raw[2], Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(rpy_raw[1], Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(rpy_raw[0], Eigen::Vector3d::UnitX()));
	R_Joint4.pretranslate(Eigen::Vector3d(0, 0, 0));


	Eigen::Isometry3d T4 = Eigen::Isometry3d::Identity();
	T4.rotate((Eigen::AngleAxisd(J4* M_PI / 180, Eigen::Vector3d::UnitZ())).toRotationMatrix());
	T4.pretranslate(Eigen::Vector3d(0, 0, 0));

	RT = RT * (T4.matrix()).inverse()*(R_Joint4.matrix());

	//same with Base frame to J1 frame, below calculation is J4 to J5 frame；

	rpy_raw << -90, 180, 0;
	rpy_raw = rpy_raw * M_PI / 180;
	Eigen::Isometry3d R_Joint5 = Eigen::Isometry3d::Identity();
	R_Joint5 = (Eigen::AngleAxisd(rpy_raw[2], Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(rpy_raw[1], Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(rpy_raw[0], Eigen::Vector3d::UnitX()));
	R_Joint5.pretranslate(Eigen::Vector3d(0, 290, 0));


	Eigen::Isometry3d T5 = Eigen::Isometry3d::Identity();
	T5.rotate((Eigen::AngleAxisd(-J5 * M_PI / 180, Eigen::Vector3d::UnitZ())).toRotationMatrix());//机械手各个轴的旋转角度，在这里是旋转了45度
	T5.pretranslate(Eigen::Vector3d(0, 0, 0));

	RT = RT * (T5.matrix()).inverse()*(R_Joint5.matrix());

	//same with Base frame to J1 frame, below calculation is TCP to J5 frame；

	rpy_raw << *TCPRX, *TCPRY, *TCPRZ;
	rpy_raw = rpy_raw * M_PI / 180;
	Eigen::Isometry3d R_TCP = Eigen::Isometry3d::Identity();
	R_TCP = (Eigen::AngleAxisd(rpy_raw[2], Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(rpy_raw[1], Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(rpy_raw[0], Eigen::Vector3d::UnitX()));
	R_TCP.pretranslate(Eigen::Vector3d(*TCPX, *TCPY, *TCPZ));


	Eigen::Isometry3d T6 = Eigen::Isometry3d::Identity();
	T6.rotate((Eigen::AngleAxisd(J6 * M_PI / 180, Eigen::Vector3d::UnitZ())).toRotationMatrix());
	T6.pretranslate(Eigen::Vector3d(0, 0, 0));

	RT = RT * (T6.matrix()).inverse()*(R_TCP.matrix());

	std::cout << RT << std::endl;


	Eigen::Matrix3d R;
	R = Eigen::Matrix3d(3, 3);

	R << RT(0, 0), RT(0, 1), RT(0, 2),
		RT(1, 0), RT(1, 1), RT(1, 2),
		RT(2, 0), RT(2, 1), RT(2, 2);
	Eigen::Vector3d Rotation = rotationMatrixToEulerAngles(R);


	double rx = (Rotation[0]) * 180 / M_PI;
	double ry = (Rotation[1]) * 180 / M_PI;
	double rz = (Rotation[2]) * 180 / M_PI;

	double Tx = RT(0, 3);
	double Ty = RT(1, 3);
	double Tz = RT(2, 3);

	std::cout << Tx << "  " << Ty << "  " << Tz << "  " << rx << "  " << ry << "  " << rz << "  " << std::endl;



	return 0;




}







int main()
{
    std::cout << "Hello World!\n";

	//KW Example :

	double a1 = 500;
	double d1 = 1045;
	double a2 = 1300;
	double a3 = 55;
	double d4 = 1275;
	double d5 = 290;

	double TCPX = 0;
	double TCPY = -905.61;
	double TCPZ = 502;
	double TCPRX = 0;
	double TCPRY = -90;
	double TCPRZ = -90;

	J1 = 38.25;
	J2 = -25.55;
	J3 = -43.64;
	J4 =279.97;
	J5 = 51.11;
	J6 = -56.98;

	KW_KUKA(&a1, & d1, & a2, & a3, & d4, & d5, & TCPX, & TCPY, & TCPZ, & TCPRX,& TCPRY, & TCPRZ);

	TCPX = -502;
	TCPY = 0;
	TCPZ = 905.61;
	TCPRX =90;
	TCPRY = 0;
	TCPRZ = 90; // FROM TCP To Flange frame;


	rx = -8.47;
	ry = -32.61;
	rz = 27.27;

	Tx = 2793.17;
	Ty = -939.64;
	Tz = 2450.18;

	IK_KUKA(&a1, &d1, &a2, &a3, &d4, &d5, &TCPX, &TCPY, &TCPZ, &TCPRX, &TCPRY, &TCPRZ);



}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
