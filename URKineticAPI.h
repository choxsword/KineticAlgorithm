#ifndef _URKineticAPIs

#define _URKineticAPIs

//快速逆解算法专用函数指针
typedef void (*Functions_t[])(void);
//dh参数
#define pi 3.141592653
#define d1 94
#define d4 -109
#define d5 95.5
#define d6 85
#define a2 353
#define a3 310


double Tfkine[4][4] = { {},{},{},{0,0,0,1} };//关节6坐标系位姿
#define nx Tfkine[0][0]
#define ny Tfkine[1][0]
#define nz Tfkine[2][0]
#define ox Tfkine[0][1]
#define oy Tfkine[1][1]
#define oz Tfkine[2][1]
#define ax Tfkine[0][2]
#define ay Tfkine[1][2]
#define az Tfkine[2][2]
#define px Tfkine[0][3]
#define py Tfkine[1][3]
#define pz Tfkine[2][3]

double	ToolPose[4][4];//工具系相对基坐标位姿
#define Tnx ToolPose[0][0]
#define Tny ToolPose[1][0]
#define Tnz ToolPose[2][0]
#define Tox ToolPose[0][1]
#define Toy ToolPose[1][1]
#define Toz ToolPose[2][1]
#define Tax ToolPose[0][2]
#define Tay ToolPose[1][2]
#define Taz ToolPose[2][2]
#define Tpx ToolPose[0][3]
#define Tpy ToolPose[1][3]
#define Tpz ToolPose[2][3]


double theta_fast[6];//存放快速逆解算法的关节角
#define t_1 theta_fast[0]
#define t_2 theta_fast[1]
#define t_3 theta_fast[2]
#define t_4 theta_fast[3]
#define t_5 theta_fast[4]
#define t_6 theta_fast[5]


double matrix[4][4];//齐次变换矩阵
double RefKine[4][4] = { {},{},{},{0,0,0,1} };//自定义坐标系下的位姿
double SelfDefCoor[4][4];//自定义坐标系的齐次变换阵
double ResultM4[4][4];//计算的结果矩阵


double k[6] = { 180,180,180,180,180,180 };
double kmin[6] = { -180, -180,-180,-180,-180,-180 };





//各个关节角值，用于逆解中
double theta1[2];
double theta5[4];
double theta6[4];

double theta2[8];
double theta3[8];
double theta4[8];


double s1, s2, s3, s4, s5, s6, c1, c2, c3, c4, c5, c6, c234, s234, c23, s23;
double crt, cpt, cyt, srt, spt, syt, pxt, pyt, pzt;//s和c代表sin和cos;r,p,y代表rpy角,t代表是Tool，即工具坐标系的参数
double nxt, nyt, nzt, oxt, oyt, ozt, axt, ayt, azt;//nx,ny等参考齐次坐标系中的含义,t代表Tool,含义同上
double snx, sny, snz, sox, soy, soz, sax, say, saz, spx, spy, spz;//用于存储一些临时计算出的齐次坐标系参数


double parax, paray;//方便逆解计算


void f1(), f2(), f3(), f4(), f5(), f6(), f7(), f8();//快速逆解算法的8组逆解映射

Functions_t Functions = { f1,f2,f3,f4,f5,f6,f7,f8 };//8组逆解函数数组

int FuncIndex = 0;//快速逆解算法索引

#endif 





