#ifndef __PARAMETROS_H__
#define __PARAMETROS_H__

    //  PARAMETROS FISICOS:

//---REFERENCIAS---
//#define rP (0.0f)
//#define rR (0.0f)
//#define rY (0.0f)
//#define rZ (0.2f)
//---PITCH---
#define bP (0.8705f)//(16.85f)//(61.42f)//(16.85f)//(61.42f)//(222.84f)//(16.85f)//(222.84f)//(7.340f)//
#define uMIN_P (-0.02155f)//(-12.93e-4f)//(-3.115e-4f)//(-12.66e-4f)//(-14.842e-3f)//(-13.13e-4f)//(-15.0e-4f)//(-16.0e-4f)//(2.35e-3f)//(15.685e-3f)//(1.35e-3f)//(15.685e-3f)//(0.3948f)//(0.2792f)//(0.3948f)//0.1974f
#define uMAX_P (0.02155f)//(12.93e-4f)//(3.115e-4f)//(12.66e-4f)//(14.842e-3f)//(13.13e-4f)//(15.0e-4f)//(16.0e-4f)
#define xiMIN_P (-0.7657f)//(-0.8087f)//(-0.5641f)//(-0.6f)//(-0.5641f)//(0.39f)  // y = 18 grados
#define xiMAX_P (0.7657f)//(0.8087f)//(0.5641f)//(-0.6f)//(0.5641f)
//u=1.313e-3    xi=0.5421   q=0.5398
#define qMIN_P (-0.4406f)//(0.39f)  // y = 18 grados
#define qMAX_P (0.4406f)
//---ROLL---
#define bR (0.8961f)//(17.35f)//(63.23f)//(17.35f)//(229.39f)
#define uMIN_R (-0.02093f)//(-12.56e-4f)//(-2.418e-4f)//(-13.0e-4f)//
#define uMAX_R (0.02093f)//(12.56e-4f)//(2.418e-4f)//(13.0e-4f)//
#define xiMIN_R (-0.7657f)//(-0.8564f)  // y = 18 grados
#define xiMAX_R (0.7657f)//(0.8564f)
//u=1.256e-3    xi=0.5465   q=0.5344
#define qMIN_R (-0.3356f)//(0.39f)  // y = 18 grados
#define qMAX_R (0.3356f)
//---YAW---
#define bY (42.54f)//(0.5752f)//(11.14f)//(147.26f)
#define uMIN_Y (-4.47e-3f)//(-0.03261f)//(-19.56e-4f)//(-2.353e-2f)//(-2.049e-2f)//(-5.0e-4f)//(-20.0e-4f)
#define uMAX_Y (4.47e-3)//(0.03261f)//(19.56e-4f)//(2.353e-2f)//(2.049e-2f)//(5.0e-4f)//(20.0e-4f)//(15.5e-4f)
#define xiMIN_Y (-8.129f)//(-0.7657f)//(-0.8564f)//(-5.723f)//  // y = 180 grados
#define xiMAX_Y (8.129f)//(0.7657f)//(0.8564f)//(5.723f)
//u=2.049e-2    xi=5.723   q=5.596
#define qMIN_Y (-3.493f)//(-0.3356f)//(-0.4406f)//(-0.4f)//(0.39f)  // y = 18 grados
#define qMAX_Y (3.493f)//(0.3356f)//(0.4406f)//(0.4f)
//---Z---
#define bZ (0.1185f)//(0.535f)
#define uMIN_Z (-18.13e-3f)//(-11.61e-3f)//(-0.02f)
#define uMAX_Z (52.35e-3f)//(2.265e-2f)//(0.06f)
#define xiMIN_Z (0.0f)//(0.29f)//(0.0f) //  y = 30 [cm]
#define xiMAX_Z (18.29e-3f)//(0.3162f)//(1.0f)
//r=0.1 umin=-5.365e-3    umax=1.003e-2   ximin= 0   ximax=0.3162    qmin=-0.0825    qmax=0
//r=0.2 umin=-11.61e-3    umax=2.265e-2   ximin= 0   ximax=0.6344    qmin=-0.1656    qmax=0.008502
//r=0.3 umin=-17.86e-3    umax=4.290e-2   ximin= 0   ximax=0.9536    qmin=-0.2488    qmax=0.021640
//r=0.4 umin=-24.10e-3    umax=6.315e-2   ximin= 0   ximax=1.2730    qmin=-0.3319    qmax=0.034790
#define qMIN_Z (-0.2477f)//(-0.0825f)//(-0.25f)//(0.39f)  // y = 18 grados
#define qMAX_Z (0.01978f)//(0.0f)//(0.02f)

#define pi (3.1416f)        //  ctte. pi
#define Ke (2.88e-8f)       //  coef. empuje
#define Cd (7.24e-10f)       //  coef. torque
#define l (0.0281f)         //  l/sqrt(2)
#define a1 (308989665.13f)  //(608989665.13f)//(309989665.13f)  //  1/(2*sqrt(2)*l*Ce)
#define a11 (390989665.13f)  //  1/(2*sqrt(2)*l*Ce)
#define a2 (345399281.57f)  //  1/(4*Cd)
#define a3 (8680555.56f)    //  1/(4*Ce)
#define velMINch (0.0f)
#define velMAXch (6853924.0f) //  2618^2 [(rad/s)^2].....25000 rpm
#define vel_HOVER (17300.0f)//(15000.0f)//(19000.0f)//  velocidad de hover p/un motor (10000.0f)
#define hoverT (0.0f)//(161197.92f)//(173650.0f)//(1950049.0f)
#define hoverR (0.0f)//(676378.38f)//(161197.92f)//
#define Tr (5)//(22)
#define TY (43)//(42)//(22)
#define Tt (87)//(170)

    //  GANANCIAS PITCH:
//Seguidor:
#define k1Ps (0.2702f)//(0.0141f)//(0.0020f)//(0.0022f)//(-0.5210f)//(0.0020f)//(0.0020f) //(0.0022f)//     //  [Nm/rad]
#define k2Ps (0.2163f)//(0.0113f)//(0.0015f)//(0.7826f)//(0.0015f)//(0.0015f)
#define NP (0.0539f)//(0.0028f)//(5.8697e-4f)//(7.7305e-4f)//(5.8697e-4f)
//Robusto:
#define k1Pr (898.8379e-3f)//(46.6257e-3f)//(12.5821e-3f)//(46.6257e-3f)//(5.4374e-3f)//(5.4374e-3f)//
#define k2Pr (682.5343e-3f)//(35.3243e-3f)//(9.6211e-3f)//(35.3243e-3f)//(2.9750e-3f)//(2.9750e-3f)//
#define kiPr (53.8823e-3f)//(2.8467e-3f)//(703.7185e-6f)//(2.8467e-3f)//(586.9714e-6f)//
//PID:
#define kpP_PID (2.65f)
#define kiP_PID (0.03f)
#define kdP_PID (1.0f)
#define kwP_PID (0.173f)

    //  GANANCIAS ROLL:
//Seguidor:
#define k1Rs (0.2625f)//(0.0137f)//(0.0020f)//(-0.5210f)//(0.0020f)//(0.0020f)      //  [Nm/rad]
#define k2Rs (0.2101f)//(0.0110f)//(0.0014f)//(0.7826f)//(0.0014f)
#define NR (0.0523f)//(0.0028f)//(5.7021e-4f)//(5.7021e-4f)
//Robusto:
#define k1Rr (873.1597e-3f)//(45.2820e-3f)//(12.2219e-3f)//(45.2820e-3f)//(5.253e-3f)
#define k2Rr (663.0355e-3f)//(34.3063e-3f)//(9.3457e-3f)//(34.3063e-3f)//(2.853e-3f)
#define kiRr (52.3430e-3f)//(2.7647e-3f)//(683.5741e-6f)//(2.7647e-3f)//(555.956e-6f)
//PID:
#define kpR_PID (2.65f)
#define kiR_PID (0.03f)
#define kdR_PID (1.0f)
#define kwR_PID (0.173f)

    //  GANANCIAS YAW:
//Seguidor:
#define k1Ys (0.0054f)//(0.4089f)//(0.0214f)//(0.0031f)      //  [Nm/rad]
#define k2Ys (0.0044f)//(0.3274f)//(0.0171f)//(0.0015f)//(0.0022f)//(0.0015f)
#define NY (0.0011f)//(0.0815f)//(0.0043f)//(8.8823e-4f)
//Robusto:
#define k1Yr (18.3089e-3f)//(1.3603f)//(70.5244e-3f)//(6.6245e-3f)
#define k2Yr (13.9387e-3f)//(1.0329f)//(53.4304e-3f)//(3.4446e-3f)//(4.4446e-3f)//(3.4446e-3f)//
#define kiYr (1.06e-3f)//(81.5448e-3f)//(4.3058e-3f)//(866.024e-6f)
//PID:
#define kpY_PID (2.65f)
#define kiY_PID (0.03f)
#define kdY_PID (1.0f)
#define kwY_PID (0.173f)

    //  GANANCIAS Z:
//Seguidor:
#define k1Zs (1.5440f)//(0.8108f)      //  [N/m]
#define k2Zs (1.2988f)//(0.6083f)
#define NZ (0.2452f)//(0.2025f)
//Robusto:
#define k1Zr (5.5146f)//(1.8470f)//(-1.0f)//(1.8470f)//
#define k2Zr (4.2157f)//(1.2387f)//(1.6509f)//(1.2387f)//
#define kiZr (245.1698e-3f)//(0.2025082f)
//PID:
#define kpZ_PID (2.65f)
#define kiZ_PID (0.03f)
#define kdZ_PID (1.0f)
#define kwZ_PID (0.173f)
//Observador:
#define L1PRY (0.5458f)//(0.0028f)//(1.5640f)
#define L2PRY (0.7202f)//(0.0042f)//(-0.9525f)
#define LPRY (-0.449f)//(-0.218f)      //
#define LY (-0.458f)//(-0.218f)      //

#define L1Z (1.0940f)//(1.1816f)//(1.5740f)
#define L2Z (-0.7948f)//(1.7605f)//(-0.9546f)
#define LZ (-0.453f)

#define cPRs (0.1f)
#define cPRr (0.18f)
#define cYs (0.0f)
#define cYr (0.38f)
#define cZs (0.0f)
#define cZr (0.0f)

    //  VARIABLES:
float rP,rR,rY,rZ,rqZ;
float x1P,x2P,x1R,x2R,x1Y,x2Y,x1Z,x2Z;
float qP,qR,qY,qZ;
//const float *P, *R, *Y, *Z;
float posP,velP,posR,velR,posY,velY,posZ,velZ;
float posP1,velP1,posR1,velR1,posY1,velY1,posZ1,velZ1;
float xiP,xiR,xiY,xiZ,x1Y,x2Y;
float posZobsv,velZobsv;
float uP,uR,uY,uZ,uP1,uR1,uY1,uZ1;
float e1P,e2P,e1R,e2R,e1Y,e2Y,e1Z,e2Z;
float windup,W,empR,empT,Th,ui,magX,magY;//,sat;

float r,x1,x2,u,e,y,xi;                     //PID
float r_z,y_z,u_z,x_1,x_2;                  //OBSERVADOR
float ur,urr;
//float rs,x1s,x2s,us;                        //SEGUIDOR
//float rss,x1ss,x2ss,uss,G,yss;              //SEGUIDOR 2
//float rr,x1r,x2r,ur,yr,er,xiactr;           //ROBUSTO
//float rrr,x1rr,x2rr,urr,yrr,err,xiactrr;    //ROBUSTO 2
float rot1,rot2,rot3,rot4,tras;
float rot_1,rot_2,rot_3,rot_4,tras_;
float ctrl_1,ctrl_2,ctrl_3,ctrl_4;          //Velocidad^2 [(rad/s)^2]
float VEL1,VEL2,VEL3,VEL4; 
float VEL1_,VEL2_,VEL3_,VEL4_;
float dtt;                 //Velocidad del motor [rad/s]
unsigned long tiempo_antr;
unsigned long tiempo_antY;
unsigned long tiempo_antt;
unsigned long tiempo_act;
/*
float e0;
float e1;
float e2;
float e1P;
float e2P;
float VEL;*/


    //  PROTOTIPO DE FUNCIONES:
void controlChuraRESET(void);
float ctrlChura_SEG(float ref, float pos, float vel, float k1, float k2, float N);
void ctrlChura_SEG2(float ref, float pos, float vel, float k1, float k2, float N, float L, float b, float c, float qMIN, float qMAX, float *qss, float *uss);
void ctrlChura_SEG3(float ref, float vel, float k1, float k2, float N, float L, float b, float qMIN, float qMAX, float *qov, float *uov);
void ctrlChura_SEG4(float ref, float pos, float k1, float k2, float N, float L1, float L2, float b, float *x1o, float *x2o, float *uo);
float ctrlChura_SEG_OBSV(float ref, float k1, float k2, float N, float L1, float L2, float *x_1, float *x_2);
void ctrlChura_ROB(float ref, float pos, float vel, float k1, float k2, float ki, float b, float xiMIN, float xiMAX, float uMIN, float uMAX, float *xi, float *ur);
void ctrlChura_ROB2(float ref, float pos, float vel, float k1, float k2, float ki, float L, float b, float c, float xiMIN, float xiMAX, float uMIN, float uMAX, float qMIN, float qMAX, float *xirr, float *qrr, float *urr);
void ctrlChura_ROB3(float ref, float pos, float k1, float k2, float ki, float L1, float L2, float b, float xiMIN, float xiMAX, float uMIN, float uMAX, float *x1or, float *x2or, float *xior, float *uor);
//float ctrlChura_PID(float ref, float pos, float kp, float ki, float kd, float kw, float uMAX, float *e1, float *e2, float *u);
float restringir(float valor, float valorMIN, float valorMAX);

#endif /* __PARAMETROS_H__ */