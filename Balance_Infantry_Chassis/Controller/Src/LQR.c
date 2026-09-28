#include "LQR.h"

//定腿长下增益矩阵,仅用于判断极性
/*		
		{  -6.135326f,  -10.919361f,  -22.120046f,   -5.327986f,   10.002740f,    0.476388f,  -61.270923f,   -2.237103f,   92.457981f,    6.657238f},  // T_r_to_b
    {  -6.135326f,  -10.919361f,   22.120046f,    5.327986f,  -61.270923f,   -2.237103f,   10.002740f,    0.476388f,   92.457981f,    6.657238f},  // T_l_to_b
    {   2.279579f,    4.130120f,   -5.354413f,   -1.208115f,    3.397834f,    0.169001f,   21.873866f,    0.821639f,   19.525712f,    2.097439f},  // T_wr_to_r
    {   2.279579f,    4.130120f,    5.354413f,    1.208115f,   21.873866f,    0.821639f,    3.397834f,    0.169001f,   19.525712f,    2.097439f}   // T_wl_to_l
*/
//变腿长下多项式拟合的全部系数
float P[40][6] = 
{ 		
/*
lqr_Q = diag([200, 50, 2500, 50, 8000, 10, 8000, 10, 40000, 100]);
lqr_R = diag([1.5, 1.5, 18, 18]);
*/
		{     -10.852f,      30.2068f,      1.06851f,      -17.773f,     -46.6823f,       32.916f},  // K[0][0]
    {    -20.1256f,      58.9735f,      6.71183f,     -30.1554f,     -101.119f,      57.3828f},  // K[0][1]
    {    -11.0595f,     -33.2665f,     -47.1196f,      48.7036f,     -31.0959f,      81.8532f},  // K[0][2]
    {     -2.0014f,     -5.81441f,     -16.8173f,       8.1265f,      -13.051f,      26.2604f},  // K[0][3]
    {    -4.14353f,      46.7678f,      43.1557f,     -45.6079f,      32.9583f,     -58.7539f},  // K[0][4]
    {   -0.174235f,      1.84984f,      1.84869f,    -0.578947f,      3.78184f,     -3.31525f},  // K[0][5]
    {    -87.2978f,     -5.51606f,      175.639f,      21.3585f,     -56.3474f,     -149.619f},  // K[0][6]
    {    -2.69002f,    -0.311251f,      4.13586f,      1.20088f,     -3.53975f,     -4.73698f},  // K[0][7]
    {     46.8911f,     -121.751f,      446.317f,      158.219f,      -51.168f,     -549.058f},  // K[0][8]
    {     2.40372f,     -6.80492f,      37.6682f,      11.0909f,     -14.3304f,     -39.1409f},  // K[0][9]
    {     -10.852f,      1.06851f,      30.2068f,       32.916f,     -46.6823f,      -17.773f},  // K[1][0]
    {    -20.1256f,      6.71183f,      58.9735f,      57.3828f,     -101.119f,     -30.1554f},  // K[1][1]
    {     11.0595f,      47.1196f,      33.2665f,     -81.8532f,      31.0959f,     -48.7036f},  // K[1][2]
    {      2.0014f,      16.8173f,      5.81441f,     -26.2604f,       13.051f,      -8.1265f},  // K[1][3]
    {    -87.2978f,      175.639f,     -5.51606f,     -149.619f,     -56.3474f,      21.3585f},  // K[1][4]
    {    -2.69002f,      4.13586f,    -0.311251f,     -4.73698f,     -3.53975f,      1.20088f},  // K[1][5]
    {    -4.14353f,      43.1557f,      46.7678f,     -58.7539f,      32.9583f,     -45.6079f},  // K[1][6]
    {   -0.174235f,      1.84869f,      1.84984f,     -3.31525f,      3.78184f,    -0.578947f},  // K[1][7]
    {     46.8911f,      446.317f,     -121.751f,     -549.058f,      -51.168f,      158.219f},  // K[1][8]
    {     2.40372f,      37.6682f,     -6.80492f,     -39.1409f,     -14.3304f,      11.0909f},  // K[1][9]
    {      1.0491f,       -7.595f,      16.3874f,      4.34245f,      3.54834f,     -19.8524f},  // K[2][0]
    {     2.35901f,     -14.6139f,      26.2179f,      6.49977f,       13.009f,     -35.9851f},  // K[2][1]
    {    -8.57827f,     -11.0444f,      31.5592f,      12.7678f,      10.4302f,     -35.2358f},  // K[2][2]
    {    -1.80943f,     -4.37345f,      7.81674f,      5.77589f,     0.908629f,     -6.54745f},  // K[2][3]
    {     2.11183f,      16.8916f,     -5.22093f,     -18.0417f,       -12.72f,      3.82095f},  // K[2][4]
    {   0.0556796f,     0.958363f,    -0.252513f,    -0.454956f,     -1.39417f,     0.353708f},  // K[2][5]
    {     11.3666f,     -10.1449f,       84.953f,      12.6232f,      -2.2278f,     -95.0577f},  // K[2][6]
    {    0.339269f,    -0.447103f,      3.26066f,     0.682118f,    -0.383653f,     -1.94541f},  // K[2][7]
    {     35.0413f,     -41.9793f,     -59.5095f,      38.5435f,      39.6379f,      23.2207f},  // K[2][8]
    {     3.70312f,     -6.81921f,     -4.15454f,      5.99628f,      6.44891f,    -0.703629f},  // K[2][9]
    {      1.0491f,      16.3874f,       -7.595f,     -19.8524f,      3.54834f,      4.34245f},  // K[3][0]
    {     2.35901f,      26.2179f,     -14.6139f,     -35.9851f,       13.009f,      6.49977f},  // K[3][1]
    {     8.57827f,     -31.5592f,      11.0444f,      35.2358f,     -10.4302f,     -12.7678f},  // K[3][2]
    {     1.80943f,     -7.81674f,      4.37345f,      6.54745f,    -0.908629f,     -5.77589f},  // K[3][3]
    {     11.3666f,       84.953f,     -10.1449f,     -95.0577f,      -2.2278f,      12.6232f},  // K[3][4]
    {    0.339269f,      3.26066f,    -0.447103f,     -1.94541f,    -0.383653f,     0.682118f},  // K[3][5]
    {     2.11183f,     -5.22093f,      16.8916f,      3.82095f,       -12.72f,     -18.0417f},  // K[3][6]
    {   0.0556796f,    -0.252513f,     0.958363f,     0.353708f,     -1.39417f,    -0.454956f},  // K[3][7]
    {     35.0413f,     -59.5095f,     -41.9793f,      23.2207f,      39.6379f,      38.5435f},  // K[3][8]
    {     3.70312f,     -4.15454f,     -6.81921f,    -0.703629f,      6.44891f,      5.99628f}   // K[3][9]
};
float Offset_Fit_Coefficients[4] = {
					-17.178f,				11.939f,			-2.9224f,				0.3429f// theta_eq
};

//定义控制矩阵
float u[10];

//定义拟合增益矩阵
float Fitting_K[4][10];

void Offset_Calc(Flag_Bit_t *flag,Compensation_Amount_t *comp,float (*Fit_Coefficients),float TL){
// float theta_offset = Fit_Coefficients[0]*TL*TL*TL + Fit_Coefficients[1]*TL*TL +	 Fit_Coefficients[2]*TL	+	Fit_Coefficients[3];
 float theta_offset = -0.5492*TL	+	0.2731;
 comp->Gravity_Comp_Theta_l = 	theta_offset;
 comp->Gravity_Comp_Theta_r = 	theta_offset;
 //theta_b定义正方向与开源不同
 if(flag->spinning_flag){
	comp->Spin_Comp_Theta_b = 0;
 }
 else{
	comp->Spin_Comp_Theta_b = 0;
 }
}

void Fitting_K_Calc(float (*fitting_k)[10],float (*p)[6],float L_l,float L_r)
{
	static unsigned short int i = 0;
	static unsigned short int j = 0;
	
	for(i=0;i<=3;i++)
	{
		for(j=0;j<=9;j++)
		{
			fitting_k[i][j] = p[i*10+j][0] + p[i*10+j][1]*L_l + p[i*10+j][2]*L_r + p[i*10+j][3]*(L_l*L_l) + p[i*10+j][4]*L_l*L_r + p[i*10+j][5]*(L_r*L_r);
		}
	}
}

float test_theta = 0.09f;
float test_yaw = 0.0f;
void LQR_Calc(Flag_Bit_t *flag,
							Goal_Setting_t *goal,
							Compensation_Amount_t *comp,
							Body_Current_Situation_t *body,
							Leg_Current_Situation_t *leg[2],
							Joint_Motor_Status_t *joint_m[2])
{
	//bz:|0,T_br|1,T_bl|2,T_wr|3,T_wl|
	static float T[4];
	u[0] = Max_Output(										   	 0  -        body->x,      X_MAX); u[1] = 	goal->d_x_t -       body->d_x;
	u[2] = Max_Output(	Find_Min_RADIAN(body->abs_yaw,goal->yaw_t),    Yaw_MAX); u[3] = goal->d_yaw_t -     body->d_yaw;
	u[4] = Max_Output(comp->Gravity_Comp_Theta_l  -  leg[0]->theta,Theta_L_MAX); u[5] =             0 - leg[0]->d_theta;
	u[6] = Max_Output(comp->Gravity_Comp_Theta_r  -  leg[1]->theta,Theta_R_MAX); u[7] =             0 - leg[1]->d_theta;
	u[8] = Max_Output(	 comp->Spin_Comp_Theta_b  -  	 body->theta,Theta_B_MAX); u[9] =             0 -   body->d_theta;
	
//	u[0] = Max_Output(										   	 0  -        body->x,      X_MAX); u[1] = 	goal->d_x_t -       body->d_x;
//	u[2] = Max_Output(	Find_Min_RADIAN(body->abs_yaw,goal->yaw_t),    Yaw_MAX); u[3] = goal->d_yaw_t -     body->d_yaw;
//	u[4] = Max_Output(								test_theta  -  leg[0]->theta,Theta_L_MAX); u[5] =             0 - leg[0]->d_theta;
//	u[6] = Max_Output(								test_theta  -  leg[1]->theta,Theta_R_MAX); u[7] =             0 - leg[1]->d_theta;
//	u[8] = Max_Output(	 comp->Spin_Comp_Theta_b  -  	 body->theta,Theta_B_MAX); u[9] =             0 -   body->d_theta;
	
	//腿杆异常取消YAW增益提高腿杆THETA增益
	if(flag->theta_flag[0]||flag->theta_flag[1])
	{
		if(flag->theta_flag[0])
		{
			Fitting_K[0][2] = 0; Fitting_K[0][3] = 0; 					
			Fitting_K[2][2] = 0; Fitting_K[2][3] = 0; 	
			
			Fitting_K[0][4] = Fitting_K[0][4]*2.0f; Fitting_K[0][6] = Fitting_K[0][6]*2.0f; 
			Fitting_K[2][4] = Fitting_K[2][4]*2.0f; Fitting_K[2][6] = Fitting_K[2][6]*2.0f;
		}
		if(flag->theta_flag[1])
		{
			Fitting_K[1][2] = 0; Fitting_K[1][3] = 0; 					
			Fitting_K[3][2] = 0; Fitting_K[3][3] = 0; 	
			
			Fitting_K[1][4] = Fitting_K[1][4]*2.0f; Fitting_K[1][6] = Fitting_K[1][6]*2.0f; 
			Fitting_K[3][4] = Fitting_K[3][4]*2.0f; Fitting_K[3][6] = Fitting_K[3][6]*2.0f;
		}
	}

	//先调腿部摆角，再调机体PITCH，再调位移速度，最后调YAW
		if(!flag->off_flag) 
		{
			if(!flag->spinning_flag) 
			{
					//关节电机极性
				T[0] = 
				-u[0]*Fitting_K[0][0]-u[1]*Fitting_K[0][1]
				+u[2]*Fitting_K[0][2]-u[3]*Fitting_K[0][3]
				-u[4]*Fitting_K[0][4]-u[5]*Fitting_K[0][5]
				-u[6]*Fitting_K[0][6]-u[7]*Fitting_K[0][7]
				-u[8]*Fitting_K[0][8]-u[9]*Fitting_K[0][9];

				T[1] = 
				-u[0]*Fitting_K[1][0]-u[1]*Fitting_K[1][1]
				+u[2]*Fitting_K[1][2]-u[3]*Fitting_K[1][3]
				-u[4]*Fitting_K[1][4]-u[5]*Fitting_K[1][5]
				-u[6]*Fitting_K[1][6]-u[7]*Fitting_K[1][7]
				-u[8]*Fitting_K[1][8]-u[9]*Fitting_K[1][9];
				
					//轮电机极性
				T[2] = 
				+u[0]*Fitting_K[2][0]+u[1]*Fitting_K[2][1]
				-u[2]*Fitting_K[2][2]+u[3]*Fitting_K[2][3]
				+u[4]*Fitting_K[2][4]+u[5]*Fitting_K[2][5]
				+u[6]*Fitting_K[2][6]+u[7]*Fitting_K[2][7]
				+u[8]*Fitting_K[2][8]+u[9]*Fitting_K[2][9];
					
				T[3] = 
				+u[0]*Fitting_K[3][0]+u[1]*Fitting_K[3][1]
				-u[2]*Fitting_K[3][2]+u[3]*Fitting_K[3][3]
				+u[4]*Fitting_K[3][4]+u[5]*Fitting_K[3][5]
				+u[6]*Fitting_K[3][6]+u[7]*Fitting_K[3][7]
				+u[8]*Fitting_K[3][8]+u[9]*Fitting_K[3][9];
				
			}
			else //小陀螺取消X位移,YAW位置增益
			{
				
					//关节电机极性
				T[0] = 
				- 0												 - u[1]*Fitting_K[0][1]
				- 0												 - u[3]*Fitting_K[0][3]
				- u[4]*Fitting_K[0][4]*1.5f- u[5]*Fitting_K[0][5]*1.5f
				-	u[6]*Fitting_K[0][6]*1.5f- u[7]*Fitting_K[0][7]*1.5f
				- u[8]*Fitting_K[0][8]*2.0f- u[9]*Fitting_K[0][9]*2.0f;

				T[1] = 
				- 0												 - u[1]*Fitting_K[1][1]
				- 0												 - u[3]*Fitting_K[1][3]
				- u[4]*Fitting_K[1][4]*1.5f- u[5]*Fitting_K[1][5]*1.5f
				-	u[6]*Fitting_K[1][6]*1.5f- u[7]*Fitting_K[1][7]*1.5f
				- u[8]*Fitting_K[1][8]*2.0f- u[9]*Fitting_K[1][9]*2.0f;
				
					//轮电机极性
				T[2] = 
				+	0												 + u[1]*Fitting_K[2][1]
				+ 0												 + u[3]*Fitting_K[2][3]
				+ u[4]*Fitting_K[2][4]*1.5f+ u[5]*Fitting_K[2][5]*1.5f
				+	u[6]*Fitting_K[2][6]*1.5f+ u[7]*Fitting_K[2][7]*1.5f
				+ u[8]*Fitting_K[2][8]*2.0f+ u[9]*Fitting_K[2][9]*2.0f;

				T[3] = 
				+	0												 + u[1]*Fitting_K[3][1]
				+ 0												 + u[3]*Fitting_K[3][3]
				+ u[4]*Fitting_K[3][4]*1.5f+ u[5]*Fitting_K[3][5]*1.5f
				+	u[6]*Fitting_K[3][6]*1.5f+ u[7]*Fitting_K[3][7]*1.5f
				+ u[8]*Fitting_K[3][8]*2.0f+ u[9]*Fitting_K[3][9]*2.0f;
				
			}
		}
		else //离地时只控制腿杆垂直
		{
			
					//关节电机极性
				T[0] = 
				- u[4]*Fitting_K[0][4]- u[5]*Fitting_K[0][5]
				-	u[6]*Fitting_K[0][6]- u[7]*Fitting_K[0][7];

				T[1] = 
				- u[4]*Fitting_K[1][4]- u[5]*Fitting_K[1][5]
				-	u[6]*Fitting_K[1][6]- u[7]*Fitting_K[1][7];

					//轮电机极性
				T[2] = 
				+ u[4]*Fitting_K[2][4]+ u[5]*Fitting_K[2][5]
				+	u[6]*Fitting_K[2][6]+ u[7]*Fitting_K[2][7];

				T[3] = 
				+ u[4]*Fitting_K[3][4]+ u[5]*Fitting_K[3][5]
				+	u[6]*Fitting_K[3][6]+ u[7]*Fitting_K[3][7];
			
		}

	//输出赋值
	joint_m[0]->Tp 							= T[1];joint_m[1]->Tp 							= T[0]; 
	joint_m[0]->T_Wheel     		= T[3];joint_m[1]->T_Wheel      		= T[2];
}








