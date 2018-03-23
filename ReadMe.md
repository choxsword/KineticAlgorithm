# URKineticAPIs接口定义说明

- 本文档对API中的一些主要参数做了说明

---
## Cartesian枚举类
- 作用：用于定义笛卡尔坐标系下的运动。
```c++
enum Cartesian
{
	x, y, z, rot, pitch, yaw, rotx, roty, rotz, Euler, RPY
};
```

- 说明：x,y,z为沿坐标轴运动，rot,pitch,yaw为以基坐标为参考的**rpy变换**，rotx,roty,rotz为绕x,y,z轴的**绕轴旋转变换**

## Solution枚举类
- 作用:用于计算逆解时，返回逆解计算的结果。
```c++
enum Solution { OK, Singular, OutOfRange, HighSpeed, Unkown,JointLimit };

```

> ok:计算成功，得到一组合适的关节角 

> OutOfRange:所要求解的位姿超出机器人工作空间

> HighSpeed:解出的该组逆解与当前关节角差距较大，会导致机器人高速运动

>JointLimit:当前位姿不在机器人的关节角范围内（±360)


---
## API函数接口变量名摘要
变量名解释（具体见下方函数）：
1. double* p/pose为笛卡尔坐标系下位姿(x,y,z,r,p,y)数组
2. double *joint为机器人6个关节角数组
3. Cartesian xyz代表在笛卡尔坐标系下运动类型
4. double*TransM为自定义世界坐标系相对世界坐标系的偏移(x,y,z,r,p,y) 数组
5. double*d为工具坐标系相对于默认工具坐标系偏移(x,y,z,r,p,y)数组
6. double Threshold 是自定义量，用来表示关节角间的最大变化量，限制连续运动时关节角发生跳变（一般在奇异点附近）
7. double delta是绕某个轴的旋转角
8. double*sol用于调试时存储8组逆解
9. double*test为调试预留
10. bool GetSolution与调试sol配合使用 


## fkine函数
- 功能：计算当前工具系相对于基坐标系的位姿
```c++
void fkine(const double *joint,const double *d, double*pose)
```
- 形参说明
>**double joint[6]**:当前机器人的6个关节角

>**double d[6]**:当前工具坐标系相对于第六个关节坐标系的位姿，d[6]={x,y,z,r,p,y},其中rpy分别为绕关节6坐标系z轴,y轴,x轴旋转的角度

>**double pose[6]**:当前工具坐标系相对于基坐标系的位姿，即正解计算结果会传入该矩阵中。pose[6]={x,y,z,r,p,y},其中rpy分别为绕基坐标系z轴,y轴,x轴旋转的角度

## ikine函数

- 功能:计算机器人逆解，以世界坐标系为参照

 ```c++
 
 Solution ikine(const double *p, const double *d, double *joint,const  double Threshold,const bool GetSolutions, double *sol, double*test)
 ```
- 形参说明：
> **double p[6]**：同fkine中的pose[6]

> **double d[6]**:同fkine中d[6]

> **double joint[6]**:同fkine中joint[6]

> **double Threshold**:阈值，用于判定逆解是否有效。以关节1为例，如果当前关节为0度，而求解出的为1度，两者之间之差不超过阈值，则认为是有效解。

> **bool GetSolutions**该参数和下一参数double *sol配合使用，如果为true,则函数会向sol中传入求解出的8组关节角，该功能主要用于调试和查看计算结果

>**double sol[48]**:用于保存求解出的8组关节角，和上一参数bool GetSolutions配合使用，如不需要查看求解结果，令上一参数为false即可

>**double test[4]**:该参数由开发者用于调试，普通使用者随便传入一个double[4]类型数组即可。

## ToolIkine函数
- 功能：计算以工具坐标系为参考下的微分运动逆解。其与ikine函数不同之处在于，可以通过给定xyz和delta,能由当前位姿计算出微分运动delta（距离/角度）后的新位姿
```c++
Solution ToolIkine( double *p,const  double* d, const Cartesian xyz, const double delta, double* joint,const double Threshold, bool GetSolutions, double*sol, double*test)
```
- 形参说明
1. delta为微分量，表示直线运动/旋转角的大小，单位为mm/°，具体是哪个由xyz控制
2. xyz用于控制运动类型（以工具坐标系为参考），表示微分运动是旋转还是沿直线运动


## RotXYZ
- 功能：计算当前位姿p0绕轴旋转/直线移动delta量后的新位姿p1
```c++
void RotXYZ(const Cartesian xyz, double* TempPose, double delta)
```
- 形参
1. TemPose同p


## SelfDefKine
- 功能：计算末端工具坐标系相对于自定义坐标系的正运动学位姿
```c++
void SelfDefKine(const double* TransM,const double *joint,const double *d, double*pose)
```
- 形参说明
1. TransM 表示自定义坐标系的6元数组

## SelfDefIkine
- 功能：基于**SelfDefKine**计算出的位姿（以自定义坐标系为参照），计算对应的6个关节角。
```c++
Solution SelfDefIkine(const double*TransM,  double *p, const double* d,const Cartesian xyz,const double delta, double* joint,const double Threshold, bool GetSolutions, double*sol, double*test)
```
- 形参说明：
1. TransM同上
2. p为以自定义坐标系为参照的，工具坐标系的位姿
 ## SelfDefPose
 ```c++
 Solution SelfDefPose(const double*TransM, double *p,const double* d,const Transfrom Type, double* joint,  bool GetSolutions, double*sol, double*test)
 ```
- 功能：用于PTP的逆解运算，即判断从当前点，运动到下一指定位姿时，计算出目标位姿的8组解中，距离当前关节角最“合理”的那组解（本算法中用的是小关节加权优先法）
> 本API与之前的Ikine系列函数不同，不用于连续运动的逆解运算，因此不需要传入Threshold。



## Fast系列
> 注:功能、参数均与普通API相同，只是中间的逆解选取替换成了快速算法。