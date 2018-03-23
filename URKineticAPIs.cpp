#include "stdafx.h"
#include <stdio.h>
#include <math.h>
#include<cmath>
#include <iostream>
#include"URKineticAPI.h"

using namespace std;

enum Cartesian
{
	x, y, z, rot, pitch, yaw, rotx, roty, rotz, Euler, RPY
};
enum Solution { OK, Singular, OutOfRange, HighSpeed, Unkown, JointLimit }IkineResult;
enum Transfrom { Increase, Decrease, Clock, AntiClock, World, Tool, Custom };





//辅助逆解计算的方程
#pragma region Funtions 
//由4维齐次变换阵计算RPY角
void   Matrix2RPY(const double T[][4], double* pose)
{


	pose[0] = T[0][3];
	pose[1] = T[1][3];
	pose[2] = T[2][3];

	pose[3] = atan2(T[1][0], T[0][0]);
	pose[4] = atan2(-T[2][0], cos(pose[3]) * T[0][0] + sin(pose[3]) * T[1][0]);
	pose[5] = atan2(sin(pose[3]) * T[0][2] - cos(pose[3]) * T[1][2], -sin(pose[3]) * T[0][1] + cos(pose[3]) * T[1][1]);

	pose[3] = pose[3] / pi * 180;
	pose[4] = pose[4] / pi * 180;
	pose[5] = pose[5] / pi * 180;

}
//由RPY角计算齐次变换矩阵
void  RPY2Matrix(const double*pose, double T[][4])
{
	double r = pose[3] * pi / 180;double p = pose[4] * pi / 180;double y = pose[5] * pi / 180;
	double crt = cos(r);
	double cpt = cos(p);
	double cyt = cos(y);
	double srt = sin(r);
	double spt = sin(p);
	double syt = sin(y);

	T[0][0] = crt*cpt;;
	T[0][1] = crt*spt*syt - srt*cyt;
	T[0][2] = crt*spt*cyt + srt*syt;
	T[0][3] = pose[0];

	T[1][0] = srt*cpt;
	T[1][1] = srt*spt*syt + crt*cyt;
	T[1][2] = srt*spt*cyt - crt*syt;
	T[1][3] = pose[1];

	T[2][0] = -spt;
	T[2][1] = cpt*syt;
	T[2][2] = cpt*cyt;
	T[2][3] = pose[2];

	T[3][0] = 0;
	T[3][1] = 0;
	T[3][2] = 0;
	T[3][3] = 1;
}
//打印8组解
void disp(const double a[][6])
{
	for (int i = 0;i < 8;i++)
	{
		for (int j = 0;j < 6;j++)
		{
			cout << a[i][j] * 180 / pi << '\t';

		}
		cout << endl;
	}

}   
//由旋转轴和旋转角获取4维齐次旋转矩阵
void TransMatrix(const Cartesian trans, const double delta, double(*RotateMatrix)[4])
{
	switch (trans)
	{
	case rotx:
		RotateMatrix[0][0] = 1;
		RotateMatrix[0][1] = 0;
		RotateMatrix[0][2] = 0;
		RotateMatrix[0][3] = 0;
		RotateMatrix[1][0] = 0;
		RotateMatrix[1][1] = cos(delta);
		RotateMatrix[1][2] = -sin(delta);
		RotateMatrix[1][3] = 0;
		RotateMatrix[2][0] = 0;
		RotateMatrix[2][1] = sin(delta);
		RotateMatrix[2][2] = cos(delta);
		RotateMatrix[2][3] = 0;
		RotateMatrix[3][0] = 0;
		RotateMatrix[3][1] = 0;
		RotateMatrix[3][2] = 0;
		RotateMatrix[3][3] = 1;

		break;
	case roty:
		RotateMatrix[0][0] = cos(delta);
		RotateMatrix[0][1] = 0;
		RotateMatrix[0][2] = sin(delta);
		RotateMatrix[0][3] = 0;
		RotateMatrix[1][0] = 0;
		RotateMatrix[1][1] = 1;
		RotateMatrix[1][2] = 0;
		RotateMatrix[1][3] = 0;
		RotateMatrix[2][0] = -sin(delta);
		RotateMatrix[2][1] = 0;
		RotateMatrix[2][2] = cos(delta);
		RotateMatrix[2][3] = 0;
		RotateMatrix[3][0] = 0;
		RotateMatrix[3][1] = 0;
		RotateMatrix[3][2] = 0;
		RotateMatrix[3][3] = 1;
		break;
	case rotz:
		RotateMatrix[0][0] = cos(delta);
		RotateMatrix[0][1] = -sin(delta);
		RotateMatrix[0][2] = 0;
		RotateMatrix[0][3] = 0;
		RotateMatrix[1][0] = sin(delta);
		RotateMatrix[1][1] = cos(delta);
		RotateMatrix[1][2] = 0;
		RotateMatrix[1][3] = 0;
		RotateMatrix[2][0] = 0;
		RotateMatrix[2][1] = 0;
		RotateMatrix[2][2] = 1;
		RotateMatrix[2][3] = 0;
		RotateMatrix[3][0] = 0;
		RotateMatrix[3][1] = 0;
		RotateMatrix[3][2] = 0;
		RotateMatrix[3][3] = 1;
		break;
	default: break;
	}

}

//由旋转轴和旋转角获取3维齐次旋转矩阵
void TransMatrix3(const Cartesian trans, const double delta, double RotateMatrix[3][3])
{
	switch (trans)
	{
	case rotx:
		RotateMatrix[0][0] = 1;
		RotateMatrix[0][1] = 0;
		RotateMatrix[0][2] = 0;

		RotateMatrix[1][0] = 0;
		RotateMatrix[1][1] = cos(delta);
		RotateMatrix[1][2] = -sin(delta);

		RotateMatrix[2][0] = 0;
		RotateMatrix[2][1] = sin(delta);
		RotateMatrix[2][2] = cos(delta);
		break;
	case roty:
		RotateMatrix[0][0] = cos(delta);
		RotateMatrix[0][1] = 0;
		RotateMatrix[0][2] = sin(delta);

		RotateMatrix[1][0] = 0;
		RotateMatrix[1][1] = 1;
		RotateMatrix[1][2] = 0;

		RotateMatrix[2][0] = -sin(delta);
		RotateMatrix[2][1] = 0;
		RotateMatrix[2][2] = cos(delta);
		break;
	case rotz:
		RotateMatrix[0][0] = cos(delta);
		RotateMatrix[0][1] = -sin(delta);
		RotateMatrix[0][2] = 0;

		RotateMatrix[1][0] = sin(delta);
		RotateMatrix[1][1] = cos(delta);
		RotateMatrix[1][2] = 0;

		RotateMatrix[2][0] = 0;
		RotateMatrix[2][1] = 0;
		RotateMatrix[2][2] = 1;

		break;
	default: break;
	}

}
//3维矩阵乘积
void matrixMultipy3(const double a[][3], const double b[][3], double result[3][3])
{

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			result[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
		}
	}
}
//由3维齐次变换阵计算RPY角
void Matrix2RPY3(const double T[][3], double* pose)
{
	pose[3] = atan2(T[1][0], T[0][0]);
	double sp = sin(pose[3]);
	double cp = cos(pose[3]);
	pose[4] = atan2(-T[2][0], cp * T[0][0] + sp * T[1][0]);
	pose[5] = atan2(sp * T[0][2] - cp * T[1][2], -sp * T[0][1] + cp * T[1][1]);

	pose[3] = pose[3] / pi * 180;
	pose[4] = pose[4] / pi * 180;
	pose[5] = pose[5] / pi * 180;
}

//4维矩阵乘积
void matrixMultipy(double a[][4], const double b[][4], double(*result)[4])
{

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			result[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j] + a[i][3] * b[3][j];

		}
	}
}
//计算8组解中与当前关节角最近的最优解的编号
int ComputeDistanceWithJoint(const double solution[8][6], const double*joint, const double Threshold, double*test)
{
	int number = 0;
	double distance = 99999999;
	double temp;
	for (int i = 0; i < 8; i++)
	{
		temp = 0;

		for (int j = 0; j < 6; j++)
		{
			//使用六个关节角之差的平方来判定最近的点
			temp += (solution[i][j] - joint[j]) * (solution[i][j] - joint[j]);
		}
		if (temp < distance)
		{
			distance = temp;
			number = i;
		}
	}
	test[0] = distance;

	if (distance > Threshold)
	{
		number = 9;
	}
	test[1] = number;
	return number;//返回最近那组解的编号
}
//判断最优解是否合理
int Check(double solution[8][6], double*joint, double Threshold, double*test, Solution &result)
{
	int Num = -1;
	double distance;
	//disp(solution);
	for (int i = 0;i < 8;i++)
	{
		result = OK;
		for (int j = 0;j < 6;j++)
		{

			if (isnan(solution[i][j]) || isinf(solution[i][j]))
			{
				result = OutOfRange;

				break;
			}

			distance = solution[i][j] - joint[j];

			if (abs(distance) > Threshold)
			{


				if (abs(distance + 360) > Threshold&&abs(distance - 360) > Threshold)//跳变
				{


					if (isnan(solution[i][1]))
					{
						result = OutOfRange;
					}
					else
					{
						result = HighSpeed;
					}
					break;
				}
				else//超关节范围
				{
					result = JointLimit;
				}
			}

		}

		if (result == OK)
		{
			Num = i;
			return Num;
		}
		else if (result == JointLimit)
		{
			Num = i;
			return Num;
		}

	}


	return	Num;


}
//以下几个都是逆解计算过程中用到的公式，可参照论文查看
inline double GetTheta234(const double t1, const double t5)
{
	return atan2(az / -sin(t5), (ax*cos(t1) + ay*sin(t1)) / -sin(t5));
}//获得a2+a3+a4
inline double GetTheta5(const double t1, const bool isplus)
{
	if (isplus)
	{

		return acos(ax*sin(t1) - ay*cos(t1));
	}
	else
	{
		return -acos(ax*sin(t1) - ay*cos(t1));
	}
}
inline double GetTheta6( const double t1,const double t5)
{

	return atan2((-ox*sin(t1) + oy*cos(t1)) / sin(t5), -(-nx*sin(t1) + ny*cos(t1)) / sin(t5));

}
inline void GetTheta2_3_4(const double t1,const double t5,const bool isplus, double&t2, double&t3, double&t4)
{
	double s234 = -az / sin(t5);
	double	c234 = (ax*cos(t1) + ay*sin(t1)) / -sin(t5);
	double	s1 = sin(t1);
	double	c1 = cos(t1);
	double M = c1*px + py*s1 - d6*(ax*c1 + ay*s1) - d5*s234;
	double N = pz - d1 - az*d6 + c234*d5;
	double C = M*M + N*N + a2*a2 - a3*a3;
	double A = -2 * a2*N;
	double B = 2 * a2*M;

	if (isplus)
	{

		t2 = atan2(B, A) - atan2(C, sqrt(A*A + B*B - C*C));//atan(x,y)

		double s2 = sin(t2);
		double c2 = cos(t2);
		t3 = atan2((N - a2*s2) / a3, (M - a2*c2) / a3) - t2;
		t4 = GetTheta234(t1, t5) - t2 - t3;
	}

	else
	{
		t2 = atan2(B, A) - atan2(C, -sqrt(A*A + B*B - C*C));
		double s2 = sin(t2);
		double c2 = cos(t2);
		t3 = atan2((N - a2*s2) / a3, (M - a2*c2) / a3) - t2;
		t4 = GetTheta234(t1, t5) - t2 - t3;
	}
	if (t3 > 2 * pi)
	{
		t3 -= 2 * pi;
	}
	else if (t3 < -2 * pi)
	{
		t3 += 2 * pi;
	}
}
//将8组解映射限制在[-360,+360]的范围内
void Clamp(double theta[][6], const double*joint, const double threshold)
{


	for (int i = 0;i < 8;i++)
	{
		for (int j = 0;j < 6;j++)
		{
			theta[i][j] = theta[i][j] / pi * 180;


			if (theta[i][j] < 0)
			{
				theta[i][j] += 360;
				/*	if (abs(theta[i][j] - 360) < 1e-5)
							{
								theta[i][j] = 360;
							}*/

			}

			if (joint[j] <= threshold)
			{
				if (abs(joint[j] - theta[i][j]) > threshold)
				{
					theta[i][j] = theta[i][j] - 360;
				}
			}



		}
	}
}
//获取[-360,+360]关节角范围内的另一个可能值	
inline double AnotherOne(const double val)
{
	if (val >= 0)
	{
		return val - 360;
	}
	else
	{
		return val + 360;
	}

}

//将8组解映射到[-360,+360]的范围内的合理值
void Adjust(double theta[][6], const double* joint)
{

	for (int i = 0;i < 8;i++)
	{
		for (int j = 0;j < 6;j++)
		{
			theta[i][j] = theta[i][j] / pi * 180;

			if (abs(theta[i][j] - joint[j]) >= abs(AnotherOne(theta[i][j]) - joint[j]))
			{
				theta[i][j] = AnotherOne(theta[i][j]);
			}
		}
	}
}
//使用小关节优先的最短行程法判定最优解（用于PTP运动，而非连续运动）
int Select(const double solution[8][6],const double*joint, double*test, Solution &result)
{

	int Num = -1;
	double distance;
	double min = 9999999;

	for (int i = 0;i < 8;i++)
	{
		result = OK;
		for (int j = 0;j < 6;j++)
		{

			if (isnan(solution[i][j]) || isinf(solution[i][j]))
			{
				result = OutOfRange;
				break;
			}

		}
		if (result == OutOfRange)
		{
			continue;
		}

		double denominator = abs(solution[i][0] - joint[0]) + abs(solution[i][1] - joint[1]) + abs(solution[i][2] - joint[2]);

		distance = (abs(solution[i][3] - joint[3]) + abs(solution[i][4] - joint[4]) + abs(solution[i][5] - joint[5])) + 5 * denominator;

		cout << distance << endl;

		if (distance < min)
		{
			min = distance;
			Num = i;
		}
	}

	if (Num != -1)
	{
		result = OK;
	}

	return	Num;


}
#pragma endregion

/***************************快速逆解算法专用的计算函数，基本功能与之前相同*********************/
#pragma region 快速逆解函数
inline void f1()
{
	t_1 = atan2(paray, parax) - atan2(-d4, sqrt(parax*parax + paray*paray - d4*d4));
	t_5 = acos(ax*sin(t_1) - ay*cos(t_1));
	t_6 = atan2((-ox*sin(t_1) + oy*cos(t_1)) / sin(t_5), -(-nx*sin(t_1) + ny*cos(t_1)) / sin(t_5));
	GetTheta2_3_4(t_1, t_5, true, t_2, t_3, t_4);
}

inline void f2()
{
	t_1 = atan2(paray, parax) - atan2(-d4, sqrt(parax*parax + paray*paray - d4*d4));
	t_5 = -acos(ax*sin(t_1) - ay*cos(t_1));
	t_6 = atan2((-ox*sin(t_1) + oy*cos(t_1)) / sin(t_5), -(-nx*sin(t_1) + ny*cos(t_1)) / sin(t_5));
	GetTheta2_3_4(t_1, t_5, true, t_2, t_3, t_4);
}
inline void f3()
{
	t_1 = atan2(paray, parax) - atan2(-d4, -sqrt(parax*parax + paray*paray - d4*d4));
	t_5 = acos(ax*sin(t_1) - ay*cos(t_1));
	t_6 = atan2((-ox*sin(t_1) + oy*cos(t_1)) / sin(t_5), -(-nx*sin(t_1) + ny*cos(t_1)) / sin(t_5));
	GetTheta2_3_4(t_1, t_5, true, t_2, t_3, t_4);

}
inline void f4()
{
	t_1 = atan2(paray, parax) - atan2(-d4, -sqrt(parax*parax + paray*paray - d4*d4));
	t_5 = -acos(ax*sin(t_1) - ay*cos(t_1));
	t_6 = atan2((-ox*sin(t_1) + oy*cos(t_1)) / sin(t_5), -(-nx*sin(t_1) + ny*cos(t_1)) / sin(t_5));
	GetTheta2_3_4(t_1, t_5, true, t_2, t_3, t_4);

}

inline void f5()
{
	t_1 = atan2(paray, parax) - atan2(-d4, sqrt(parax*parax + paray*paray - d4*d4));
	t_5 = acos(ax*sin(t_1) - ay*cos(t_1));
	t_6 = atan2((-ox*sin(t_1) + oy*cos(t_1)) / sin(t_5), -(-nx*sin(t_1) + ny*cos(t_1)) / sin(t_5));
	GetTheta2_3_4(t_1, t_5, false, t_2, t_3, t_4);
}
inline void f6()
{
	t_1 = atan2(paray, parax) - atan2(-d4, sqrt(parax*parax + paray*paray - d4*d4));
	t_5 = -acos(ax*sin(t_1) - ay*cos(t_1));
	t_6 = atan2((-ox*sin(t_1) + oy*cos(t_1)) / sin(t_5), -(-nx*sin(t_1) + ny*cos(t_1)) / sin(t_5));
	GetTheta2_3_4(t_1, t_5, false, t_2, t_3, t_4);
}
inline void f7()
{
	t_1 = atan2(paray, parax) - atan2(-d4, -sqrt(parax*parax + paray*paray - d4*d4));
	t_5 = acos(ax*sin(t_1) - ay*cos(t_1));
	t_6 = atan2((-ox*sin(t_1) + oy*cos(t_1)) / sin(t_5), -(-nx*sin(t_1) + ny*cos(t_1)) / sin(t_5));
	GetTheta2_3_4(t_1, t_5, false, t_2, t_3, t_4);

}
inline void f8()
{
	t_1 = atan2(paray, parax) - atan2(-d4, -sqrt(parax*parax + paray*paray - d4*d4));
	t_5 = -acos(ax*sin(t_1) - ay*cos(t_1));
	t_6 = atan2((-ox*sin(t_1) + oy*cos(t_1)) / sin(t_5), -(-nx*sin(t_1) + ny*cos(t_1)) / sin(t_5));
	GetTheta2_3_4(t_1, t_5, false, t_2, t_3, t_4);

}

void FastClamp(double* theta, const double* joint, const double threshold)
{
	for (int j = 0;j < 6;j++)
	{
		theta[j] = theta[j] / pi * 180;


		if (theta[j] < 0)
		{
			theta[j] += 360;
			/*	if (abs(theta[j] - 360) < 1e-5)
						{
							theta[j] = 360;
						}*/
		}

		if (joint[j] <= threshold)
		{
			if (abs(joint[j] - theta[j]) > threshold)
			{
				theta[j] = theta[j] - 360;
			}
		}
	}


}

inline Solution FastCheck(const double* theta, const double*joint, const double threshold)
{
	if (isnan(theta[3]))
	{
		return OutOfRange;

	}
	for (int i = 0;i < 6;++i)
	{
		if (abs(theta[i] - joint[i]) > threshold)
			return HighSpeed;
	}
	return OK;

}


#pragma endregion

//导出dll部分
extern "C"
{
	/*****************************************普通逆解算法API********************************************************/

	__declspec(dllexport) void fkine(const double *joint,const double *d, double*pose)

	{
		double theta[6];
		for (int i = 0;i < 6;i++)
		{
			theta[i] = joint[i] * pi / 180;
		}
		s1 = sin(theta[0]);
		s2 = sin(theta[1]);
		s3 = sin(theta[2]);
		s4 = sin(theta[3]);
		s5 = sin(theta[4]);
		s6 = sin(theta[5]);
		c1 = cos(theta[0]);
		c2 = cos(theta[1]);
		c3 = cos(theta[2]);
		c4 = cos(theta[3]);
		c5 = cos(theta[4]);
		c6 = cos(theta[5]);
		c234 = cos(theta[1] + theta[2] + theta[3]);
		s234 = sin(theta[1] + theta[2] + theta[3]);
		c23 = cos(theta[1] + theta[2]);
		s23 = sin(theta[1] + theta[2]);
		nx = c6*(s1*s5 + c1*c5*c234) - c1*s6*s234;
		ny = -c6*(c1*s5 - c5*c234*s1) - s1*s6*s234;
		nz = c234*s6 + c5*c6*s234;

		ox = -s6*(s1*s5 + c1*c5*c234) - c1*c6*s234;
		oy = s6*(c1*s5 - c5*c234*s1) - c6*s1*s234;
		oz = c6*c234 - c5*s6*s234;

		ax = c5*s1 - c1*c234*s5;
		ay = -c1*c5 - c234*s1*s5;
		az = -s5*s234;

		px = d4*s1 + d6*(c5*s1 - c1*c234*s5) + a2*c1*c2 + a3*c1*c23 + c1*d5*s234;
		py = a2*c2*s1 - d6*(c1*c5 + c234*s1*s5) - c1*d4 + a3*c23*s1 + d5*s1*s234;
		pz = d1 + a2*s2 + a3*s23 - d5*(c4*c23 - s4*s23) - d6*s5*(c4*s23 + c23*s4);

		/*	for (int i = 0;i < 4;i++)
			{
				for (int j = 0;j < 4;j++)
				{
					cout << Tfkine[i][j] << '\t';
				}
				cout << endl;
			}*/
		double ToolRPY[3];
		//工具坐标系求解
		ToolRPY[0] = d[3] * pi / 180;
		ToolRPY[1] = d[4] * pi / 180;
		ToolRPY[2] = d[5] * pi / 180;

		crt = cos(ToolRPY[0]);
		cpt = cos(ToolRPY[1]);
		cyt = cos(ToolRPY[2]);

		srt = sin(ToolRPY[0]);
		spt = sin(ToolRPY[1]);
		syt = sin(ToolRPY[2]);

		pxt = d[0];
		pyt = d[1];
		pzt = d[2];

		nxt = crt*cpt;
		oxt = crt*spt*syt - srt*cyt;
		axt = crt*spt*cyt + srt*syt;

		nyt = srt*cpt;
		oyt = srt*spt*syt + crt*cyt;
		ayt = srt*spt*cyt - crt*syt;

		nzt = -spt;
		ozt = cpt*syt;
		azt = cpt*cyt;

		snx = nx*nxt + ox*nyt + ax*nzt;
		sny = ny*nxt + oy*nyt + ay*nzt;
		snz = nz*nxt + oz*nyt + az*nzt;
		sox = nx*oxt + ox*oyt + ax*ozt;
		soy = ny*oxt + oy*oyt + ay*ozt;
		soz = nz*oxt + oz*oyt + az*ozt;
		sax = nx*axt + ox*ayt + ax*azt;
		say = ny*axt + oy*ayt + ay*azt;
		saz = nz*axt + oz*ayt + az*azt;
		spx = nx*pxt + ox*pyt + ax*pzt + px;
		spy = ny*pxt + oy*pyt + ay*pzt + py;
		spz = nz*pxt + oz*pyt + az*pzt + pz;

		double r1, p1, y1;

		r1 = atan2(sny, snx);
		p1 = atan2(-snz, cos(r1)*snx + sin(r1)*sny);
		y1 = atan2(sin(r1)*sax - cos(r1)*say, -sin(r1)*sox + cos(r1)*soy);

		pose[0] = spx;
		pose[1] = spy;
		pose[2] = spz;
		pose[3] = r1 * 180 / pi;
		pose[4] = p1 * 180 / pi;
		pose[5] = y1 * 180 / pi;

	}
	__declspec(dllexport) Solution ikine(const double *p, const double *d, double *joint,const  double Threshold,const bool GetSolutions, double *sol, double*test)
	{
		crt = cos(d[3] * pi / 180);
		cpt = cos(d[4] * pi / 180);
		cyt = cos(d[5] * pi / 180);
		srt = sin(d[3] * pi / 180);
		spt = sin(d[4] * pi / 180);
		syt = sin(d[5] * pi / 180);

		pxt = d[0];
		pyt = d[1];
		pzt = d[2];

		nxt = crt*cpt;
		oxt = crt*spt*syt - srt*cyt;
		axt = crt*spt*cyt + srt*syt;

		nyt = srt*cpt;
		oyt = srt*spt*syt + crt*cyt;
		ayt = srt*spt*cyt - crt*syt;
		azt = cpt*cyt;
		nzt = -spt;
		ozt = cpt*syt;

		double cr = cos(p[3] * pi / 180);
		double cp = cos(p[4] * pi / 180);
		double cy = cos(p[5] * pi / 180);
		double sr = sin(p[3] * pi / 180);
		double sp = sin(p[4] * pi / 180);
		double sy = sin(p[5] * pi / 180);


		//六轴坐标系绝对位姿求解 


		Tnx = cr*cp;
		Tox = cr*sp*sy - sr*cy;
		Tax = cr*sp*cy + sr*sy;
		Tpx = p[0];
		Tny = sr*cp;
		Toy = sr*sp*sy + cr*cy;
		Tay = sr*sp*cy - cr*sy;
		Tpy = p[1];
		Tpz = p[2];
		Tnz = -sp;
		Toz = cp*sy;
		Taz = cp*cy;


		double pxti = -(nxt*pxt + nyt*pyt + nzt*pzt);
		double pyti = -(oxt*pxt + oyt*pyt + ozt*pzt);
		double pzti = -(axt*pxt + ayt*pyt + azt*pzt);

		//六轴坐标系绝对位姿求解 

		nx = Tnx*nxt + Tox*oxt + Tax*axt;
		ny = Tny*nxt + Toy*oxt + Tay*axt;
		nz = Tnz*nxt + Toz*oxt + Taz*axt;

		ox = Tnx*nyt + Tox*oyt + Tax*ayt;
		oy = Tny*nyt + Toy*oyt + Tay*ayt;
		oz = Tnz*nyt + Toz*oyt + Taz*ayt;

		ax = Tnx*nzt + Tox*ozt + Tax*azt;
		ay = Tny*nzt + Toy*ozt + Tay*azt;
		az = Tnz*nzt + Toz*ozt + Taz*azt;

		px = Tnx*pxti + Tox*pyti + Tax*pzti + Tpx;
		py = Tny*pxti + Toy*pyti + Tay*pzti + Tpy;
		pz = Tnz*pxti + Toz*pyti + Taz*pzti + Tpz;

		double parax = px - d6*ax;
		double paray = py - d6*ay;

		theta1[0] = atan2(paray, parax) - atan2(-d4, sqrt(parax*parax + paray*paray - d4*d4));
		theta1[1] = atan2(paray, parax) - atan2(-d4, -sqrt(parax*parax + paray*paray - d4*d4));

		theta5[0] = GetTheta5(theta1[0], true);
		theta5[1] = GetTheta5(theta1[0], false);
		theta5[2] = GetTheta5(theta1[1], true);
		theta5[3] = GetTheta5(theta1[1], false);


		theta6[0] = GetTheta6(theta1[0], theta5[0]);
		theta6[1] = GetTheta6(theta1[0], theta5[1]);
		theta6[2] = GetTheta6(theta1[1], theta5[2]);
		theta6[3] = GetTheta6(theta1[1], theta5[3]);


		GetTheta2_3_4(theta1[0], theta5[0], true, theta2[0], theta3[0], theta4[0]);
		GetTheta2_3_4(theta1[0], theta5[1], true, theta2[1], theta3[1], theta4[1]);
		GetTheta2_3_4(theta1[1], theta5[2], true, theta2[2], theta3[2], theta4[2]);
		GetTheta2_3_4(theta1[1], theta5[3], true, theta2[3], theta3[3], theta4[3]);
		GetTheta2_3_4(theta1[0], theta5[0], false, theta2[4], theta3[4], theta4[4]);
		GetTheta2_3_4(theta1[0], theta5[1], false, theta2[5], theta3[5], theta4[5]);
		GetTheta2_3_4(theta1[1], theta5[2], false, theta2[6], theta3[6], theta4[6]);
		GetTheta2_3_4(theta1[1], theta5[3], false, theta2[7], theta3[7], theta4[7]);



		double theta[8][6] =
		{
				{theta1[0],theta2[0],theta3[0],theta4[0],theta5[0],theta6[0]},
				{theta1[0],theta2[1],theta3[1],theta4[1],theta5[1],theta6[1]},
				{theta1[1],theta2[2],theta3[2],theta4[2],theta5[2],theta6[2]},
				{theta1[1],theta2[3],theta3[3],theta4[3],theta5[3],theta6[3]},
				{theta1[0],theta2[4],theta3[4],theta4[4],theta5[0],theta6[0]},
				{theta1[0],theta2[5],theta3[5],theta4[5],theta5[1],theta6[1]},
				{theta1[1],theta2[6],theta3[6],theta4[6],theta5[2],theta6[2]},
				{theta1[1],theta2[7],theta3[7],theta4[7],theta5[3],theta6[3]},
		};

		Clamp(theta, joint, Threshold);
		Solution result;
		int Num = Check(theta, joint, Threshold, test, result);

		if (GetSolutions)
		{


			for (int i = 0;i < 8;i++)
			{
				for (int j = 0;j < 6;j++)
				{

					sol[i * 6 + j] = theta[i][j];
				}
			}
			test[1] = Num;
		}

		if (result == OK)
		{
			for (int i = 0;i < 6;i++)
			{
				joint[i] = theta[Num][i];

				if (abs(joint[i]) < 1e-5)
				{
					joint[i] = 0;
				}
			}
		}

		return result;

	}
	__declspec(dllexport) Solution ToolIkine( double *p,const  double* d, const Cartesian xyz, const double delta, double* joint,const double Threshold, bool GetSolutions, double*sol, double*test)
	{

		double cr = cos(p[3] * pi / 180);
		double cp = cos(p[4] * pi / 180);
		double cy = cos(p[5] * pi / 180);
		double sr = sin(p[3] * pi / 180);
		double sp = sin(p[4] * pi / 180);
		double sy = sin(p[5] * pi / 180);

		crt = cos(d[3] * pi / 180);
		cpt = cos(d[4] * pi / 180);
		cyt = cos(d[5] * pi / 180);
		srt = sin(d[3] * pi / 180);
		spt = sin(d[4] * pi / 180);
		syt = sin(d[5] * pi / 180);

		pxt = d[0];
		pyt = d[1];
		pzt = d[2];

		//偏置矩阵
		nxt = crt*cpt;
		oxt = crt*spt*syt - srt*cyt;
		axt = crt*spt*cyt + srt*syt;

		nyt = srt*cpt;
		oyt = srt*spt*syt + crt*cyt;
		ayt = srt*spt*cyt - crt*syt;
		azt = cpt*cyt;
		nzt = -spt;
		ozt = cpt*syt;


		double Matrix[3][3];
		double ResultM[3][3];
		double T[3][3];

		Tpx = p[0];
		Tpy = p[1];
		Tpz = p[2];
		T[0][0] = cr*cp;
		T[0][1] = cr*sp*sy - sr*cy;
		T[0][2] = cr*sp*cy + sr*sy;

		T[1][0] = sr*cp;
		T[1][1] = sr*sp*sy + cr*cy;
		T[1][2] = sr*sp*cy - cr*sy;

		T[2][0] = -sp;
		T[2][1] = cp*sy;
		T[2][2] = cp*cy;

		bool isRot = false;
		double pxti = -(nxt*pxt + nyt*pyt + nzt*pzt);
		double pyti = -(oxt*pxt + oyt*pyt + ozt*pzt);
		double pzti = -(axt*pxt + ayt*pyt + azt*pzt);
		switch (xyz)
		{
		case x:Tpx += delta*T[0][0];Tpy += delta*T[1][0];Tpz += delta*T[2][0];
			break;
		case y:Tpx += delta*T[0][1];Tpy += delta*T[1][1];Tpz += delta*T[2][1];
			break;
		case z:Tpx += delta*T[0][2];Tpy += delta*T[1][2];Tpz += delta*T[2][2];
			break;
		case rotx:
		case roty:
		case rotz:
			TransMatrix3(xyz, delta*pi / 180, Matrix);
			matrixMultipy3(T, Matrix, ResultM);
			Matrix2RPY3(ResultM, p);
			nx = ResultM[0][0] * nxt + ResultM[0][1] * oxt + ResultM[0][2] * axt;
			ny = ResultM[1][0] * nxt + ResultM[1][1] * oxt + ResultM[1][2] * axt;
			nz = ResultM[2][0] * nxt + ResultM[2][1] * oxt + ResultM[2][2] * axt;

			ox = ResultM[0][0] * nyt + ResultM[0][1] * oyt + ResultM[0][2] * ayt;
			oy = ResultM[1][0] * nyt + ResultM[1][1] * oyt + ResultM[1][2] * ayt;
			oz = ResultM[2][0] * nyt + ResultM[2][1] * oyt + ResultM[2][2] * ayt;

			ax = ResultM[0][0] * nzt + ResultM[0][1] * ozt + ResultM[0][2] * azt;
			ay = ResultM[1][0] * nzt + ResultM[1][1] * ozt + ResultM[1][2] * azt;
			az = ResultM[2][0] * nzt + ResultM[2][1] * ozt + ResultM[2][2] * azt;

			px = ResultM[0][0] * pxti + ResultM[0][1] * pyti + ResultM[0][2] * pzti + Tpx;
			py = ResultM[1][0] * pxti + ResultM[1][1] * pyti + ResultM[1][2] * pzti + Tpy;
			pz = ResultM[2][0] * pxti + ResultM[2][1] * pyti + ResultM[2][2] * pzti + Tpz;
			isRot = true;
			break;

		}

		if (!isRot)
		{
			p[0] = Tpx;
			p[1] = Tpy;
			p[2] = Tpz;

			//六轴坐标系相对于工具坐标系位置 
			nx = T[0][0] * nxt + T[0][1] * oxt + T[0][2] * axt;
			ny = T[1][0] * nxt + T[1][1] * oxt + T[1][2] * axt;
			nz = T[2][0] * nxt + T[2][1] * oxt + T[2][2] * axt;

			ox = T[0][0] * nyt + T[0][1] * oyt + T[0][2] * ayt;
			oy = T[1][0] * nyt + T[1][1] * oyt + T[1][2] * ayt;
			oz = T[2][0] * nyt + T[2][1] * oyt + T[2][2] * ayt;

			ax = T[0][0] * nzt + T[0][1] * ozt + T[0][2] * azt;
			ay = T[1][0] * nzt + T[1][1] * ozt + T[1][2] * azt;
			az = T[2][0] * nzt + T[2][1] * ozt + T[2][2] * azt;

			px = T[0][0] * pxti + T[0][1] * pyti + T[0][2] * pzti + Tpx;
			py = T[1][0] * pxti + T[1][1] * pyti + T[1][2] * pzti + Tpy;
			pz = T[2][0] * pxti + T[2][1] * pyti + T[2][2] * pzti + Tpz;
		}
		//六轴坐标系绝对位姿求解 

		double parax = px - d6*ax;
		double paray = py - d6*ay;

		theta1[0] = atan2(paray, parax) - atan2(-d4, sqrt(parax*parax + paray*paray - d4*d4));
		theta1[1] = atan2(paray, parax) - atan2(-d4, -sqrt(parax*parax + paray*paray - d4*d4));


		theta5[0] = GetTheta5(theta1[0], true);
		theta5[1] = GetTheta5(theta1[0], false);
		theta5[2] = GetTheta5(theta1[1], true);
		theta5[3] = GetTheta5(theta1[1], false);


		theta6[0] = GetTheta6(theta1[0], theta5[0]);
		theta6[1] = GetTheta6(theta1[0], theta5[1]);
		theta6[2] = GetTheta6(theta1[1], theta5[2]);
		theta6[3] = GetTheta6(theta1[1], theta5[3]);

		GetTheta2_3_4(theta1[0], theta5[0], true, theta2[0], theta3[0], theta4[0]);
		GetTheta2_3_4(theta1[0], theta5[1], true, theta2[1], theta3[1], theta4[1]);
		GetTheta2_3_4(theta1[1], theta5[2], true, theta2[2], theta3[2], theta4[2]);
		GetTheta2_3_4(theta1[1], theta5[3], true, theta2[3], theta3[3], theta4[3]);
		GetTheta2_3_4(theta1[0], theta5[0], false, theta2[4], theta3[4], theta4[4]);
		GetTheta2_3_4(theta1[0], theta5[1], false, theta2[5], theta3[5], theta4[5]);
		GetTheta2_3_4(theta1[1], theta5[2], false, theta2[6], theta3[6], theta4[6]);
		GetTheta2_3_4(theta1[1], theta5[3], false, theta2[7], theta3[7], theta4[7]);

		double theta[8][6] =
		{
				{theta1[0],theta2[0],theta3[0],theta4[0],theta5[0],theta6[0]},
				{theta1[0],theta2[1],theta3[1],theta4[1],theta5[1],theta6[1]},
				{theta1[1],theta2[2],theta3[2],theta4[2],theta5[2],theta6[2]},
				{theta1[1],theta2[3],theta3[3],theta4[3],theta5[3],theta6[3]},
				{theta1[0],theta2[4],theta3[4],theta4[4],theta5[0],theta6[0]},
				{theta1[0],theta2[5],theta3[5],theta4[5],theta5[1],theta6[1]},
				{theta1[1],theta2[6],theta3[6],theta4[6],theta5[2],theta6[2]},
				{theta1[1],theta2[7],theta3[7],theta4[7],theta5[3],theta6[3]}

		};


		Clamp(theta, joint, Threshold);
		Solution result;
		int Num = Check(theta, joint, Threshold, test, result);

		if (GetSolutions)
		{
			for (int i = 0;i < 8;i++)
			{
				for (int j = 0;j < 6;j++)
				{

					sol[i * 6 + j] = theta[i][j];
				}
			}
			test[1] = Num;
		}

		if (result == OK)
		{
			for (int i = 0;i < 6;i++)
			{
				joint[i] = theta[Num][i];

				if (abs(joint[i]) < 1e-5)
				{
					joint[i] = 0;
				}
			}
		}

		return result;
	}
	__declspec(dllexport) void RotXYZ(const Cartesian xyz, double* TempPose, double delta)
	{
		double theta, fan, fi;
		theta = TempPose[4] * pi / 180;
		fi = TempPose[5] * pi / 180;
		fan = TempPose[3] * pi / 180;
		double position[4][4];
		position[0][3] = TempPose[0];
		position[1][3] = TempPose[1];
		position[2][3] = TempPose[2];
		position[3][3] = 1;
		position[3][0] = 0;
		position[3][1] = 0;
		position[3][2] = 0;
		position[0][0] = cos(fan) * cos(theta);
		position[1][0] = sin(fan) * cos(theta);
		position[2][0] = -sin(theta);
		position[0][1] = cos(fan) * sin(theta) * sin(fi) - sin(fan) * cos(fi);
		position[1][1] = sin(fan) * sin(theta) * sin(fi) + cos(fan) * cos(fi);
		position[2][1] = cos(theta) * sin(fi);
		position[0][2] = cos(fan) * sin(theta) * cos(fi) + sin(fan) * sin(fi);
		position[1][2] = sin(fan) * sin(theta) * cos(fi) - cos(fan) * sin(fi);
		position[2][2] = cos(theta) * cos(fi);


		double T[4][4];
		delta = delta / 180 * pi;

		TransMatrix(xyz, delta, matrix);

		matrixMultipy(matrix, position, T);

		Matrix2RPY(T, TempPose);

	}
	__declspec(dllexport) void SelfDefKine(const double* TransM,const double *joint,const double *d, double*pose)

	{

		double theta[6], trans[6];
		for (int i = 0;i < 3;i++)
		{
			trans[i] = TransM[i];
		}
		for (int i = 3;i < 6;i++) {
			trans[i] = TransM[i] * pi / 180;

		}
		for (int i = 0;i < 6;i++)
		{
			theta[i] = joint[i] * pi / 180;
		}

		double cr = cos(trans[3]);
		double cp = cos(trans[4]);
		double cy = cos(trans[5]);
		double sr = sin(trans[3]);
		double sp = sin(trans[4]);
		double sy = sin(trans[5]);
		double s_nx = cr*cp;
		double s_ox = sr*cp;
		double s_ax = -sp;

		double s_ny = cr*sp*sy - sr*cy;
		double s_oy = sr*sp*sy + cr*cy;
		double s_ay = cp*sy;


		double s_nz = cr*sp*cy + sr*sy;
		double s_oz = sr*sp*cy - cr*sy;

		double s_az = cp*cy;

		double s_px = -(trans[0] * s_nx + trans[1] * s_ox + trans[2] * s_ax);
		double s_py = -(trans[0] * s_ny + trans[1] * s_oy + trans[2] * s_ay);
		double s_pz = -(trans[0] * s_nz + trans[1] * s_oz + trans[2] * s_az);


		s1 = sin(theta[0]);
		s2 = sin(theta[1]);
		s3 = sin(theta[2]);
		s4 = sin(theta[3]);
		s5 = sin(theta[4]);
		s6 = sin(theta[5]);
		c1 = cos(theta[0]);
		c2 = cos(theta[1]);
		c3 = cos(theta[2]);
		c4 = cos(theta[3]);
		c5 = cos(theta[4]);
		c6 = cos(theta[5]);
		c234 = cos(theta[1] + theta[2] + theta[3]);
		s234 = sin(theta[1] + theta[2] + theta[3]);
		c23 = cos(theta[1] + theta[2]);
		s23 = sin(theta[1] + theta[2]);

		Tnx = c6*(s1*s5 + c1*c5*c234) - c1*s6*s234;
		Tny = -c6*(c1*s5 - c5*c234*s1) - s1*s6*s234;
		Tnz = c234*s6 + c5*c6*s234;

		Tox = -s6*(s1*s5 + c1*c5*c234) - c1*c6*s234;
		Toy = s6*(c1*s5 - c5*c234*s1) - c6*s1*s234;
		Toz = c6*c234 - c5*s6*s234;

		Tax = c5*s1 - c1*c234*s5;
		Tay = -c1*c5 - c234*s1*s5;
		Taz = -s5*s234;

		Tpx = d4*s1 + d6*(c5*s1 - c1*c234*s5) + a2*c1*c2 + a3*c1*c23 + c1*d5*s234;
		Tpy = a2*c2*s1 - d6*(c1*c5 + c234*s1*s5) - c1*d4 + a3*c23*s1 + d5*s1*s234;
		Tpz = d1 + a2*s2 + a3*s23 - d5*(c4*c23 - s4*s23) - d6*s5*(c4*s23 + c23*s4);

		crt = cos(d[3] / 180 * pi);
		cpt = cos(d[4] / 180 * pi);
		cyt = cos(d[5] / 180 * pi);
		srt = sin(d[3] / 180 * pi);
		spt = sin(d[4] / 180 * pi);
		syt = sin(d[5] / 180 * pi);

		pxt = d[0];
		pyt = d[1];
		pzt = d[2];

		nxt = crt*cpt;
		oxt = crt*spt*syt - srt*cyt;
		axt = crt*spt*cyt + srt*syt;

		nyt = srt*cpt;
		oyt = srt*spt*syt + crt*cyt;
		ayt = srt*spt*cyt - crt*syt;

		nzt = -spt;
		ozt = cpt*syt;
		azt = cpt*cyt;

		snx = Tnx*nxt + Tox*nyt + Tax*nzt;
		sny = Tny*nxt + Toy*nyt + Tay*nzt;
		snz = Tnz*nxt + Toz*nyt + Taz*nzt;
		sox = Tnx*oxt + Tox*oyt + Tax*ozt;
		soy = Tny*oxt + Toy*oyt + Tay*ozt;
		soz = Tnz*oxt + Toz*oyt + Taz*ozt;
		sax = Tnx*axt + Tox*ayt + Tax*azt;
		say = Tny*axt + Toy*ayt + Tay*azt;
		saz = Tnz*axt + Toz*ayt + Taz*azt;
		spx = Tnx*pxt + Tox*pyt + Tax*pzt + Tpx;
		spy = Tny*pxt + Toy*pyt + Tay*pzt + Tpy;
		spz = Tnz*pxt + Toz*pyt + Taz*pzt + Tpz;


		nx = s_nx*snx + s_ox*sny + s_ax*snz;
		ny = s_ny*snx + s_oy*sny + s_ay*snz;
		nz = s_nz*snx + s_oz*sny + s_az*snz;
		ox = s_nx*sox + s_ox*soy + s_ax*soz;
		oy = s_ny*sox + s_oy*soy + s_ay*soz;
		oz = s_nz*sox + s_oz*soy + s_az*soz;
		ax = s_nx*sax + s_ox*say + s_ax*saz;
		ay = s_ny*sax + s_oy*say + s_ay*saz;
		az = s_nz*sax + s_oz*say + s_az*saz;
		px = s_nx*spx + s_ox*spy + s_ax*spz + s_px;
		py = s_ny*spx + s_oy*spy + s_ay*spz + s_py;
		pz = s_nz*spx + s_oz*spy + s_az*spz + s_pz;



		double r1, p1, y1;

		r1 = atan2(ny, nx);
		p1 = atan2(-nz, cos(r1)*nx + sin(r1)*ny);
		y1 = atan2(sin(r1)*ax - cos(r1)*ay, -sin(r1)*ox + cos(r1)*oy);

		pose[0] = px;
		pose[1] = py;
		pose[2] = pz;
		pose[3] = r1 * 180 / pi;
		pose[4] = p1 * 180 / pi;
		pose[5] = y1 * 180 / pi;

	}
	__declspec(dllexport) Solution SelfDefIkine(const double*TransM,  double *p, const double* d,const Cartesian xyz,const double delta, double* joint,const double Threshold, bool GetSolutions, double*sol, double*test)
	{


		crt = cos(d[3] * pi / 180);
		cpt = cos(d[4] * pi / 180);
		cyt = cos(d[5] * pi / 180);
		srt = sin(d[3] * pi / 180);
		spt = sin(d[4] * pi / 180);
		syt = sin(d[5] * pi / 180);

		pxt = d[0];
		pyt = d[1];
		pzt = d[2];

		//偏置矩阵
		nxt = crt*cpt;
		oxt = crt*spt*syt - srt*cyt;
		axt = crt*spt*cyt + srt*syt;

		nyt = srt*cpt;
		oyt = srt*spt*syt + crt*cyt;
		ayt = srt*spt*cyt - crt*syt;
		azt = cpt*cyt;
		nzt = -spt;
		ozt = cpt*syt;

		//自定义坐标系的齐次矩阵旋转变换部分





		double pxti = -(nxt*pxt + nyt*pyt + nzt*pzt);
		double pyti = -(oxt*pxt + oyt*pyt + ozt*pzt);
		double pzti = -(axt*pxt + ayt*pyt + azt*pzt);


		RPY2Matrix(TransM, SelfDefCoor);

		RPY2Matrix(p, RefKine);


		switch (xyz)
		{
		case x:RefKine[0][3] += delta;p[0] += delta;
			matrixMultipy(SelfDefCoor, RefKine, ToolPose);
			break;
		case y:RefKine[1][3] += delta;matrixMultipy(SelfDefCoor, RefKine, ToolPose);p[1] += delta;
			break;
		case z:RefKine[2][3] += delta;matrixMultipy(SelfDefCoor, RefKine, ToolPose);p[2] += delta;
			break;
		case rotx:
		case roty:
		case rotz:

			TransMatrix(xyz, delta*pi / 180, matrix);
			matrixMultipy(matrix, RefKine, ResultM4);
			matrixMultipy(SelfDefCoor, ResultM4, ToolPose);
			Matrix2RPY(ResultM4, p);
			break;
		}




		nx = Tnx*nxt + Tox*oxt + Tax*axt;
		ny = Tny*nxt + Toy*oxt + Tay*axt;
		nz = Tnz*nxt + Toz*oxt + Taz*axt;

		ox = Tnx*nyt + Tox*oyt + Tax*ayt;
		oy = Tny*nyt + Toy*oyt + Tay*ayt;
		oz = Tnz*nyt + Toz*oyt + Taz*ayt;

		ax = Tnx*nzt + Tox*ozt + Tax*azt;
		ay = Tny*nzt + Toy*ozt + Tay*azt;
		az = Tnz*nzt + Toz*ozt + Taz*azt;

		px = Tnx*pxti + Tox*pyti + Tax*pzti + Tpx;
		py = Tny*pxti + Toy*pyti + Tay*pzti + Tpy;
		pz = Tnz*pxti + Toz*pyti + Taz*pzti + Tpz;






		//六轴坐标系绝对位姿求解 

		double parax = px - d6*ax;
		double paray = py - d6*ay;

		theta1[0] = atan2(paray, parax) - atan2(-d4, sqrt(parax*parax + paray*paray - d4*d4));
		theta1[1] = atan2(paray, parax) - atan2(-d4, -sqrt(parax*parax + paray*paray - d4*d4));


		theta5[0] = GetTheta5(theta1[0], true);
		theta5[1] = GetTheta5(theta1[0], false);
		theta5[2] = GetTheta5(theta1[1], true);
		theta5[3] = GetTheta5(theta1[1], false);


		theta6[0] = GetTheta6(theta1[0], theta5[0]);
		theta6[1] = GetTheta6(theta1[0], theta5[1]);
		theta6[2] = GetTheta6(theta1[1], theta5[2]);
		theta6[3] = GetTheta6(theta1[1], theta5[3]);

		GetTheta2_3_4(theta1[0], theta5[0], true, theta2[0], theta3[0], theta4[0]);
		GetTheta2_3_4(theta1[0], theta5[1], true, theta2[1], theta3[1], theta4[1]);
		GetTheta2_3_4(theta1[1], theta5[2], true, theta2[2], theta3[2], theta4[2]);
		GetTheta2_3_4(theta1[1], theta5[3], true, theta2[3], theta3[3], theta4[3]);
		GetTheta2_3_4(theta1[0], theta5[0], false, theta2[4], theta3[4], theta4[4]);
		GetTheta2_3_4(theta1[0], theta5[1], false, theta2[5], theta3[5], theta4[5]);
		GetTheta2_3_4(theta1[1], theta5[2], false, theta2[6], theta3[6], theta4[6]);
		GetTheta2_3_4(theta1[1], theta5[3], false, theta2[7], theta3[7], theta4[7]);

		double theta[8][6] =
		{
				{theta1[0],theta2[0],theta3[0],theta4[0],theta5[0],theta6[0]},
				{theta1[0],theta2[1],theta3[1],theta4[1],theta5[1],theta6[1]},
				{theta1[1],theta2[2],theta3[2],theta4[2],theta5[2],theta6[2]},
				{theta1[1],theta2[3],theta3[3],theta4[3],theta5[3],theta6[3]},
				{theta1[0],theta2[4],theta3[4],theta4[4],theta5[0],theta6[0]},
				{theta1[0],theta2[5],theta3[5],theta4[5],theta5[1],theta6[1]},
				{theta1[1],theta2[6],theta3[6],theta4[6],theta5[2],theta6[2]},
				{theta1[1],theta2[7],theta3[7],theta4[7],theta5[3],theta6[3]}

		};

		Clamp(theta, joint, Threshold);
		Solution result;
		int Num = Check(theta, joint, Threshold, test, result);

		if (GetSolutions)
		{
			for (int i = 0;i < 8;i++)
			{
				for (int j = 0;j < 6;j++)
				{

					sol[i * 6 + j] = theta[i][j];
				}
			}
		}

		if (result == OK)
		{
			for (int i = 0;i < 6;i++)
			{
				joint[i] = theta[Num][i];

				if (abs(joint[i]) < 1e-5)
				{
					joint[i] = 0;
				}
			}
			test[1] = Num;
		}

		return result;
	}
	__declspec(dllexport) Solution SelfDefPose(const double*TransM, double *p,const double* d,const Transfrom Type, double* joint, bool GetSolutions, double*sol, double*test)
	{


		crt = cos(d[3] * pi / 180);
		cpt = cos(d[4] * pi / 180);
		cyt = cos(d[5] * pi / 180);
		srt = sin(d[3] * pi / 180);
		spt = sin(d[4] * pi / 180);
		syt = sin(d[5] * pi / 180);

		pxt = d[0];
		pyt = d[1];
		pzt = d[2];

		//偏置矩阵
		nxt = crt*cpt;
		oxt = crt*spt*syt - srt*cyt;
		axt = crt*spt*cyt + srt*syt;

		nyt = srt*cpt;
		oyt = srt*spt*syt + crt*cyt;
		ayt = srt*spt*cyt - crt*syt;
		azt = cpt*cyt;
		nzt = -spt;
		ozt = cpt*syt;

		//自定义坐标系的齐次矩阵旋转变换部分




		double pxti = -(nxt*pxt + nyt*pyt + nzt*pzt);
		double pyti = -(oxt*pxt + oyt*pyt + ozt*pzt);
		double pzti = -(axt*pxt + ayt*pyt + azt*pzt);

		if (Type == Custom)
		{
			RPY2Matrix(TransM, SelfDefCoor);

			RPY2Matrix(p, RefKine);

			matrixMultipy(SelfDefCoor, RefKine, ToolPose);
			Matrix2RPY(RefKine, p);
		}
		else
		{
			double cr = cos(p[3] * pi / 180);
			double cp = cos(p[4] * pi / 180);
			double cy = cos(p[5] * pi / 180);
			double sr = sin(p[3] * pi / 180);
			double sp = sin(p[4] * pi / 180);
			double sy = sin(p[5] * pi / 180);


			Tnx = cr*cp;
			Tox = cr*sp*sy - sr*cy;
			Tax = cr*sp*cy + sr*sy;
			Tpx = p[0];
			Tny = sr*cp;
			Toy = sr*sp*sy + cr*cy;
			Tay = sr*sp*cy - cr*sy;
			Tpy = p[1];
			Tpz = p[2];
			Tnz = -sp;
			Toz = cp*sy;
			Taz = cp*cy;



		}
		nx = Tnx*nxt + Tox*oxt + Tax*axt;
		ny = Tny*nxt + Toy*oxt + Tay*axt;
		nz = Tnz*nxt + Toz*oxt + Taz*axt;

		ox = Tnx*nyt + Tox*oyt + Tax*ayt;
		oy = Tny*nyt + Toy*oyt + Tay*ayt;
		oz = Tnz*nyt + Toz*oyt + Taz*ayt;

		ax = Tnx*nzt + Tox*ozt + Tax*azt;
		ay = Tny*nzt + Toy*ozt + Tay*azt;
		az = Tnz*nzt + Toz*ozt + Taz*azt;

		px = Tnx*pxti + Tox*pyti + Tax*pzti + Tpx;
		py = Tny*pxti + Toy*pyti + Tay*pzti + Tpy;
		pz = Tnz*pxti + Toz*pyti + Taz*pzti + Tpz;



		//六轴坐标系绝对位姿求解 

		double parax = px - d6*ax;
		double paray = py - d6*ay;

		theta1[0] = atan2(paray, parax) - atan2(-d4, sqrt(parax*parax + paray*paray - d4*d4));
		theta1[1] = atan2(paray, parax) - atan2(-d4, -sqrt(parax*parax + paray*paray - d4*d4));


		theta5[0] = GetTheta5(theta1[0], true);
		theta5[1] = GetTheta5(theta1[0], false);
		theta5[2] = GetTheta5(theta1[1], true);
		theta5[3] = GetTheta5(theta1[1], false);


		theta6[0] = GetTheta6(theta1[0], theta5[0]);
		theta6[1] = GetTheta6(theta1[0], theta5[1]);
		theta6[2] = GetTheta6(theta1[1], theta5[2]);
		theta6[3] = GetTheta6(theta1[1], theta5[3]);

		GetTheta2_3_4(theta1[0], theta5[0], true, theta2[0], theta3[0], theta4[0]);
		GetTheta2_3_4(theta1[0], theta5[1], true, theta2[1], theta3[1], theta4[1]);
		GetTheta2_3_4(theta1[1], theta5[2], true, theta2[2], theta3[2], theta4[2]);
		GetTheta2_3_4(theta1[1], theta5[3], true, theta2[3], theta3[3], theta4[3]);
		GetTheta2_3_4(theta1[0], theta5[0], false, theta2[4], theta3[4], theta4[4]);
		GetTheta2_3_4(theta1[0], theta5[1], false, theta2[5], theta3[5], theta4[5]);
		GetTheta2_3_4(theta1[1], theta5[2], false, theta2[6], theta3[6], theta4[6]);
		GetTheta2_3_4(theta1[1], theta5[3], false, theta2[7], theta3[7], theta4[7]);


		double theta[8][6] =
		{
				{theta1[0],theta2[0],theta3[0],theta4[0],theta5[0],theta6[0]},
				{theta1[0],theta2[1],theta3[1],theta4[1],theta5[1],theta6[1]},
				{theta1[1],theta2[2],theta3[2],theta4[2],theta5[2],theta6[2]},
				{theta1[1],theta2[3],theta3[3],theta4[3],theta5[3],theta6[3]},
				{theta1[0],theta2[4],theta3[4],theta4[4],theta5[0],theta6[0]},
				{theta1[0],theta2[5],theta3[5],theta4[5],theta5[1],theta6[1]},
				{theta1[1],theta2[6],theta3[6],theta4[6],theta5[2],theta6[2]},
				{theta1[1],theta2[7],theta3[7],theta4[7],theta5[3],theta6[3]}

		};

		//disp(theta);
		Adjust(theta, joint);
		Solution result;
		int Num = Select(theta, joint, test, result);

		if (GetSolutions)
		{
			for (int i = 0;i < 8;i++)
			{
				for (int j = 0;j < 6;j++)
				{

					sol[i * 6 + j] = theta[i][j];
				}
			}
			test[1] = Num;
		}

		if (result == OK)
		{

			for (int i = 0;i < 6;i++)
			{
				joint[i] = theta[Num][i];

				cout << theta[Num][i] << " ";
				if (abs(joint[i]) < 1e-5)
				{
					joint[i] = 0;
				}
			}
			cout << endl;
		}

		return result;
	}

/***************************************快速逆解算法专用API（详见论文）***************************************/
	//快速逆解算法
	__declspec(dllexport) Solution Fast_Ikine(double *p, double *d, double *joint, double Threshold, double*test)
	{
		crt = cos(d[3] * pi / 180);
		cpt = cos(d[4] * pi / 180);
		cyt = cos(d[5] * pi / 180);
		srt = sin(d[3] * pi / 180);
		spt = sin(d[4] * pi / 180);
		syt = sin(d[5] * pi / 180);

		pxt = d[0];
		pyt = d[1];
		pzt = d[2];

		nxt = crt*cpt;
		oxt = crt*spt*syt - srt*cyt;
		axt = crt*spt*cyt + srt*syt;

		nyt = srt*cpt;
		oyt = srt*spt*syt + crt*cyt;
		ayt = srt*spt*cyt - crt*syt;
		azt = cpt*cyt;
		nzt = -spt;
		ozt = cpt*syt;

		double cr = cos(p[3] * pi / 180);
		double cp = cos(p[4] * pi / 180);
		double cy = cos(p[5] * pi / 180);
		double sr = sin(p[3] * pi / 180);
		double sp = sin(p[4] * pi / 180);
		double sy = sin(p[5] * pi / 180);


		//六轴坐标系绝对位姿求解 


		Tnx = cr*cp;
		Tox = cr*sp*sy - sr*cy;
		Tax = cr*sp*cy + sr*sy;
		Tpx = p[0];
		Tny = sr*cp;
		Toy = sr*sp*sy + cr*cy;
		Tay = sr*sp*cy - cr*sy;
		Tpy = p[1];
		Tpz = p[2];
		Tnz = -sp;
		Toz = cp*sy;
		Taz = cp*cy;


		double pxti = -(nxt*pxt + nyt*pyt + nzt*pzt);
		double pyti = -(oxt*pxt + oyt*pyt + ozt*pzt);
		double pzti = -(axt*pxt + ayt*pyt + azt*pzt);

		//六轴坐标系绝对位姿求解 

		nx = Tnx*nxt + Tox*oxt + Tax*axt;
		ny = Tny*nxt + Toy*oxt + Tay*axt;
		nz = Tnz*nxt + Toz*oxt + Taz*axt;

		ox = Tnx*nyt + Tox*oyt + Tax*ayt;
		oy = Tny*nyt + Toy*oyt + Tay*ayt;
		oz = Tnz*nyt + Toz*oyt + Taz*ayt;

		ax = Tnx*nzt + Tox*ozt + Tax*azt;
		ay = Tny*nzt + Toy*ozt + Tay*azt;
		az = Tnz*nzt + Toz*ozt + Taz*azt;

		px = Tnx*pxti + Tox*pyti + Tax*pzti + Tpx;
		py = Tny*pxti + Toy*pyti + Tay*pzti + Tpy;
		pz = Tnz*pxti + Toz*pyti + Taz*pzti + Tpz;

		parax = px - d6*ax;
		paray = py - d6*ay;

		//快速逆解选取
		Solution result;
		int i = -1;
		do {
			++i;
			(Functions[(FuncIndex + i) % 8])();
			FastClamp(theta_fast, joint, Threshold);
			result = FastCheck(theta_fast, joint, Threshold);


		} while (result != OK&&i < 7);


		if (result == OK)
		{
			FuncIndex = (FuncIndex + i) % 8;
			for (int i = 0;i < 6;i++)
			{
				joint[i] = theta_fast[i];

				if (abs(joint[i]) < 1e-5)
				{
					joint[i] = 0;
				}
			}
		}
		else
		{
			(Functions[FuncIndex])();
			if (!isnan(theta_fast[3]))//说明不是无解
			{
				result = HighSpeed;
				for (int i = 0;i < 6;++i)
				{
					if (abs(theta_fast[i] - joint[i] + 360) < Threshold || abs(theta_fast[i] - joint[i] - 360) < Threshold)
					{
						result = JointLimit;
					}
				}
			}

		}
		test[1] = FuncIndex;
		return result;

	}
	__declspec(dllexport) Solution Fast_ToolIkine(double *p, double* d, const Cartesian xyz, double delta, double* joint, double Threshold, double*test)
	{

		double cr = cos(p[3] * pi / 180);
		double cp = cos(p[4] * pi / 180);
		double cy = cos(p[5] * pi / 180);
		double sr = sin(p[3] * pi / 180);
		double sp = sin(p[4] * pi / 180);
		double sy = sin(p[5] * pi / 180);

		crt = cos(d[3] * pi / 180);
		cpt = cos(d[4] * pi / 180);
		cyt = cos(d[5] * pi / 180);
		srt = sin(d[3] * pi / 180);
		spt = sin(d[4] * pi / 180);
		syt = sin(d[5] * pi / 180);

		pxt = d[0];
		pyt = d[1];
		pzt = d[2];

		//偏置矩阵
		nxt = crt*cpt;
		oxt = crt*spt*syt - srt*cyt;
		axt = crt*spt*cyt + srt*syt;

		nyt = srt*cpt;
		oyt = srt*spt*syt + crt*cyt;
		ayt = srt*spt*cyt - crt*syt;
		azt = cpt*cyt;
		nzt = -spt;
		ozt = cpt*syt;


		double Matrix[3][3];
		double ResultM[3][3];
		double T[3][3];

		Tpx = p[0];
		Tpy = p[1];
		Tpz = p[2];
		T[0][0] = cr*cp;
		T[0][1] = cr*sp*sy - sr*cy;
		T[0][2] = cr*sp*cy + sr*sy;

		T[1][0] = sr*cp;
		T[1][1] = sr*sp*sy + cr*cy;
		T[1][2] = sr*sp*cy - cr*sy;

		T[2][0] = -sp;
		T[2][1] = cp*sy;
		T[2][2] = cp*cy;

		bool isRot = false;
		double pxti = -(nxt*pxt + nyt*pyt + nzt*pzt);
		double pyti = -(oxt*pxt + oyt*pyt + ozt*pzt);
		double pzti = -(axt*pxt + ayt*pyt + azt*pzt);
		switch (xyz)
		{
		case x:Tpx += delta*T[0][0];Tpy += delta*T[1][0];Tpz += delta*T[2][0];
			break;
		case y:Tpx += delta*T[0][1];Tpy += delta*T[1][1];Tpz += delta*T[2][1];
			break;
		case z:Tpx += delta*T[0][2];Tpy += delta*T[1][2];Tpz += delta*T[2][2];
			break;
		case rotx:
		case roty:
		case rotz:
			TransMatrix3(xyz, delta*pi / 180, Matrix);
			matrixMultipy3(T, Matrix, ResultM);
			Matrix2RPY3(ResultM, p);
			nx = ResultM[0][0] * nxt + ResultM[0][1] * oxt + ResultM[0][2] * axt;
			ny = ResultM[1][0] * nxt + ResultM[1][1] * oxt + ResultM[1][2] * axt;
			nz = ResultM[2][0] * nxt + ResultM[2][1] * oxt + ResultM[2][2] * axt;

			ox = ResultM[0][0] * nyt + ResultM[0][1] * oyt + ResultM[0][2] * ayt;
			oy = ResultM[1][0] * nyt + ResultM[1][1] * oyt + ResultM[1][2] * ayt;
			oz = ResultM[2][0] * nyt + ResultM[2][1] * oyt + ResultM[2][2] * ayt;

			ax = ResultM[0][0] * nzt + ResultM[0][1] * ozt + ResultM[0][2] * azt;
			ay = ResultM[1][0] * nzt + ResultM[1][1] * ozt + ResultM[1][2] * azt;
			az = ResultM[2][0] * nzt + ResultM[2][1] * ozt + ResultM[2][2] * azt;

			px = ResultM[0][0] * pxti + ResultM[0][1] * pyti + ResultM[0][2] * pzti + Tpx;
			py = ResultM[1][0] * pxti + ResultM[1][1] * pyti + ResultM[1][2] * pzti + Tpy;
			pz = ResultM[2][0] * pxti + ResultM[2][1] * pyti + ResultM[2][2] * pzti + Tpz;
			isRot = true;
			break;

		}

		if (!isRot)
		{
			p[0] = Tpx;
			p[1] = Tpy;
			p[2] = Tpz;

			//六轴坐标系相对于工具坐标系位置 
			nx = T[0][0] * nxt + T[0][1] * oxt + T[0][2] * axt;
			ny = T[1][0] * nxt + T[1][1] * oxt + T[1][2] * axt;
			nz = T[2][0] * nxt + T[2][1] * oxt + T[2][2] * axt;

			ox = T[0][0] * nyt + T[0][1] * oyt + T[0][2] * ayt;
			oy = T[1][0] * nyt + T[1][1] * oyt + T[1][2] * ayt;
			oz = T[2][0] * nyt + T[2][1] * oyt + T[2][2] * ayt;

			ax = T[0][0] * nzt + T[0][1] * ozt + T[0][2] * azt;
			ay = T[1][0] * nzt + T[1][1] * ozt + T[1][2] * azt;
			az = T[2][0] * nzt + T[2][1] * ozt + T[2][2] * azt;

			px = T[0][0] * pxti + T[0][1] * pyti + T[0][2] * pzti + Tpx;
			py = T[1][0] * pxti + T[1][1] * pyti + T[1][2] * pzti + Tpy;
			pz = T[2][0] * pxti + T[2][1] * pyti + T[2][2] * pzti + Tpz;
		}
		//六轴坐标系绝对位姿求解 

		parax = px - d6*ax;
		paray = py - d6*ay;


		Solution result;
		int i = -1;
		do {
			++i;
			(Functions[(FuncIndex + i) % 8])();
			FastClamp(theta_fast, joint, Threshold);
			result = FastCheck(theta_fast, joint, Threshold);


		} while (result != OK&&i < 7);


		if (result == OK)
		{
			FuncIndex = (FuncIndex + i) % 8;
			for (int i = 0;i < 6;i++)
			{
				joint[i] = theta_fast[i];

				if (abs(joint[i]) < 1e-5)
				{
					joint[i] = 0;
				}
			}
		}
		else
		{
			(Functions[FuncIndex])();
			if (!isnan(theta_fast[3]))//说明不是无解
			{
				result = HighSpeed;
				for (int i = 0;i < 6;++i)
				{
					if (abs(theta_fast[i] - joint[i] + 360) < Threshold || abs(theta_fast[i] - joint[i] - 360) < Threshold)
					{
						result = JointLimit;
					}
				}
			}


		}
		test[1] = FuncIndex;
		return result;
	}
	__declspec(dllexport) Solution Fast_SelfDefIkine(const double*TransM, double *p, double* d,const  Cartesian xyz, double delta, double* joint, double Threshold, double*test)
	{


		crt = cos(d[3] * pi / 180);
		cpt = cos(d[4] * pi / 180);
		cyt = cos(d[5] * pi / 180);
		srt = sin(d[3] * pi / 180);
		spt = sin(d[4] * pi / 180);
		syt = sin(d[5] * pi / 180);

		pxt = d[0];
		pyt = d[1];
		pzt = d[2];

		//偏置矩阵
		nxt = crt*cpt;
		oxt = crt*spt*syt - srt*cyt;
		axt = crt*spt*cyt + srt*syt;

		nyt = srt*cpt;
		oyt = srt*spt*syt + crt*cyt;
		ayt = srt*spt*cyt - crt*syt;
		azt = cpt*cyt;
		nzt = -spt;
		ozt = cpt*syt;

		//自定义坐标系的齐次矩阵旋转变换部分





		double pxti = -(nxt*pxt + nyt*pyt + nzt*pzt);
		double pyti = -(oxt*pxt + oyt*pyt + ozt*pzt);
		double pzti = -(axt*pxt + ayt*pyt + azt*pzt);


		RPY2Matrix(TransM, SelfDefCoor);

		RPY2Matrix(p, RefKine);


		switch (xyz)
		{
		case x:RefKine[0][3] += delta;p[0] += delta;
			matrixMultipy(SelfDefCoor, RefKine, ToolPose);
			break;
		case y:RefKine[1][3] += delta;matrixMultipy(SelfDefCoor, RefKine, ToolPose);p[1] += delta;
			break;
		case z:RefKine[2][3] += delta;matrixMultipy(SelfDefCoor, RefKine, ToolPose);p[2] += delta;
			break;
		case rotx:
		case roty:
		case rotz:

			TransMatrix(xyz, delta*pi / 180, matrix);
			matrixMultipy(matrix, RefKine, ResultM4);
			matrixMultipy(SelfDefCoor, ResultM4, ToolPose);
			Matrix2RPY(ResultM4, p);
			break;
		}




		nx = Tnx*nxt + Tox*oxt + Tax*axt;
		ny = Tny*nxt + Toy*oxt + Tay*axt;
		nz = Tnz*nxt + Toz*oxt + Taz*axt;

		ox = Tnx*nyt + Tox*oyt + Tax*ayt;
		oy = Tny*nyt + Toy*oyt + Tay*ayt;
		oz = Tnz*nyt + Toz*oyt + Taz*ayt;

		ax = Tnx*nzt + Tox*ozt + Tax*azt;
		ay = Tny*nzt + Toy*ozt + Tay*azt;
		az = Tnz*nzt + Toz*ozt + Taz*azt;

		px = Tnx*pxti + Tox*pyti + Tax*pzti + Tpx;
		py = Tny*pxti + Toy*pyti + Tay*pzti + Tpy;
		pz = Tnz*pxti + Toz*pyti + Taz*pzti + Tpz;






		//六轴坐标系绝对位姿求解 

		parax = px - d6*ax;
		paray = py - d6*ay;


		Solution result;
		int i = -1;
		do {
			++i;
			(Functions[(FuncIndex + i) % 8])();
			FastClamp(theta_fast, joint, Threshold);
			result = FastCheck(theta_fast, joint, Threshold);


		} while (result != OK&&i < 7);


		if (result == OK)
		{
			FuncIndex = (FuncIndex + i) % 8;
			for (int i = 0;i < 6;i++)
			{
				joint[i] = theta_fast[i];

				if (abs(joint[i]) < 1e-5)
				{
					joint[i] = 0;
				}
			}
		}
		else
		{
			(Functions[FuncIndex])();
			if (!isnan(theta_fast[3]))//说明不是无解
			{
				result = HighSpeed;
				for (int i = 0;i < 6;++i)
				{
					if (abs(theta_fast[i] - joint[i] + 360) < Threshold || abs(theta_fast[i] - joint[i] - 360) < Threshold)
					{
						result = JointLimit;
					}
				}
			};
		}
		test[1] = FuncIndex;
		return result;
	}

}
