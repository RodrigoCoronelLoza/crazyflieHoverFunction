#include "FreeRTOS.h"
#include "task.h"
/////////////////////
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "sensfusion6.h"
#include "position_controller.h"
#include "controller_pid.h"

#include "log.h"
#include "param.h"
#include "math3d.h"

#include "Parametros.h"
#include "math.h"
#include "led.h"
#include "num.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static bool tiltCompensationEnabled = false;

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

////////////////////////////////////
  //  Pitch:
static float referenciaP;
static float velocidadP;
static float salidaP;
static float integralP;
static float entradaP;
  //  Roll:
static float velocidadR;
static float salidaR;
static float integralR;
static float entradaR;
  //  yaw:
static float referenciaY;
static float velocidadY;
static float salidaY;
static float integralY;
static float entradaY;
  //  Thrust:
//static float referenciaZ;
static float velocidadZ;
static float salidaZ;
static float integralZ;
static float entradaZ;
  //  Velocidades:
static float vel_1;
static float vel_2;
static float vel_3;
static float vel_4;
///////////////////////////////////
static float Tss;
bool habilitador;

void controllerPidInit(void)
{
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  positionControllerInit();
}

bool controllerPidTest(void)
{
  bool pass = true;

  pass &= attitudeControllerTest();

  return pass;
}

static float capAngle(float angle) {
  float result = angle;

  while (result > 180.0f) {
    result -= 360.0f;
  }

  while (result < -180.0f) {
    result += 360.0f;
  }

  return result;
}

void controllerPid(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Rate-controled YAW is moving YAW angle setpoint
    if (setpoint->mode.yaw == modeVelocity) {
       attitudeDesired.yaw += setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT;
    } else {
      attitudeDesired.yaw = setpoint->attitude.yaw;
    }

    attitudeDesired.yaw = capAngle(attitudeDesired.yaw);
  }

  if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
    positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Switch between manual and automatic position control
    if (setpoint->mode.z == modeDisable) {
      actuatorThrust = setpoint->thrust;
    }
    if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
      attitudeDesired.roll = setpoint->attitude.roll;
      attitudeDesired.pitch = setpoint->attitude.pitch;
    }

    attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

    // For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
    // value. Also reset the PID to avoid error buildup, which can lead to unstable
    // behavior if level mode is engaged later
    if (setpoint->mode.roll == modeVelocity) {
      rateDesired.roll = setpoint->attitudeRate.roll;
      attitudeControllerResetRollAttitudePID();
    }
    if (setpoint->mode.pitch == modeVelocity) {
      rateDesired.pitch = setpoint->attitudeRate.pitch;
      attitudeControllerResetPitchAttitudePID();
    }

    // TODO: Investigate possibility to subtract gyro drift.
    attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                             rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

    attitudeControllerGetActuatorOutput(&control->roll,
                                        &control->pitch,
                                        &control->yaw);

    control->yaw = -control->yaw;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;
    r_roll = radians(sensors->gyro.x);
    r_pitch = -radians(sensors->gyro.y);
    r_yaw = radians(sensors->gyro.z);
    accelz = sensors->acc.z;
  }

  if (tiltCompensationEnabled)
  {
    control->thrust = actuatorThrust / sensfusion6GetInvThrustCompensationForTilt();
  }
  else
  {
    control->thrust = actuatorThrust;
  }

  if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    attitudeControllerResetAllPID();
    positionControllerResetAllPID();

    // Reset the calculated YAW angle for rate control
    attitudeDesired.yaw = state->attitude.yaw;
  }
}

///////////////////////////////////////////
void controlCHURAInit(void)
{
  controlChuraRESET();
}
bool controlCHURATest(void){
  return true;
}
void controlCHURA(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{

  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }
  if(setpoint->thrust > 0){
 // if(setpoint->attitude.pitch > 0){
    habilitador = 1;
  }
  else{
    habilitador = 0;
  }
  if (habilitador == 0){
    controlChuraRESET();
    ledSet(LED_RED_L,0);
  }
  else{
    tiempo_act = tick;

    /************************
     * periodo de muestreo
  //dtt = tick;
dtt = tiempo_act-tiempo_antr;

tiempo_antr = tiempo_act;
***************************/
  rP = -0.015f;//setpoint->attitude.pitch;
  rR = 0.01f;//0.01f;//setpoint->attitude.roll;
  rY = 0.0f;//-state->attitude.yaw*pi/180;
  rZ = 0.1f;
  posP = state->attitude.pitch*pi/180;
  posR = state->attitude.roll*pi/180;

/*
  //magX = sensors->mag.x*180/pi + 16;//state->attitude.yaw*pi/180;
  magY = sensors->mag.y*180/pi - 240;//sensors->gyro.z*pi/180;
  if(magY>=0){
    posY = (float)atan2(sensors->mag.y,sensors->mag.x) - 1.688f;//1.653f;
  }
  else{
    posY = -((float)atan2(sensors->mag.y,sensors->mag.x) - 1.688f);//1.653f;
  }
  */
  posY = state->attitude.yaw*pi/180;//posY*pi/0.1f;
  posZ = x1Z;//bZ*(x1Z + x2Z);//state->position.z;
  
  velP = -sensors->gyro.y*pi/180;
  velR = sensors->gyro.x*pi/180;
  velY = sensors->gyro.z*pi/180;
  velZ = state->velocity.z;
  //vTaskDelay(10);
  

                    //  SENIAL DE CONTROL:
  /////////////////////////////////////////////////////////////////////////////////////////////////
  //                      Control - Seguidor
  //uP = ctrlChura_SEG(rP, posP, velP, k1Ps, k2Ps, NP);
  //uR = ctrlChura_SEG(rR, posR, velR, k1Rs, k2Rs, NR);
  //uY = ctrlChura_SEG(rY, posY, velY, k1Ys, k2Ys, NY);
  //uZ = ctrlChura_SEG(rZ, posZ, velZ, k1Zs, k2Zs, NZ);
  //                      Control robusto
  //ctrlChura_ROB(rP, posP, velP, k1Pr, k2Pr, kiPr, bP, xiMIN_P, xiMAX_P, uMIN_P, uMAX_P, &xiP, &uP);
  //ctrlChura_ROB(rR, posR, velR, k1Rr, k2Rr, kiRr, bR, xiMIN_R, xiMAX_R, uMIN_R, uMAX_R, &xiR, &uR);
  //ctrlChura_ROB(rY, posY, velY, k1Yr, k2Yr, kiYr, bY, xiMIN_Y, xiMAX_Y, uMIN_Y, uMAX_Y, &xiY, &uY);
  //ctrlChura_ROB(rZ, posZ, velZ, k1Zr, k2Zr, kiZr, bZ, xiMIN_Z, xiMAX_Z, uMIN_Z, uMAX_Z, &xiZ, &uZ);
  ///////////////////////////////////////////////////////////////////////////////////////////////////
  //****************************************ROTACIONAL**************************************************
  if((tiempo_act-tiempo_antr)>=Tr){
    //                    OBSERVADOR REDUCIDO:
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    //                    Control seguidor
    //ctrlChura_SEG2(rP, posP, velP, k1Ps, k2Ps, NP, LPRY, bP, cPRs, qMIN_P, qMAX_P, &qP, &uP); //observador posicion
    //ctrlChura_SEG2(rR, posR, velR, k1Rs, k2Rs, NR, LPRY, bR, cPRs, qMIN_R, qMAX_R, &qR, &uR); //observador posicion
    //ctrlChura_SEG2(rY, posY, velY, k1Ys, k2Ys, NY, LPRY, bY, cPRs, qMIN_Y, qMAX_Y, &qY, &uY); //observador posicion

    //ctrlChura_SEG3(rY, velY, k2Ys, k1Ys, NY, LPRY, bY, qMIN_Y, qMAX_Y, &qY, &uY); //observador velocidad
    //                    Control robusto
    ctrlChura_ROB2(rP, posP, velP, k1Pr, k2Pr, kiPr, LPRY, bP, cPRr, xiMIN_P, xiMAX_P, uMIN_P, uMAX_P, qMIN_P, qMAX_P, &xiP, &qP, &uP);
    ctrlChura_ROB2(rR, posR, velR, k1Rr, k2Rr, kiRr, LPRY, bR, cPRr, xiMIN_R, xiMAX_R, uMIN_R, uMAX_R, qMIN_R, qMAX_R, &xiR, &qR, &uR);
    //ctrlChura_ROB2(rY, posY, velY, k1Yr, k2Yr, kiYr, LPRY, bY, cPRr, xiMIN_Y, xiMAX_Y, uMIN_Y, uMAX_Y, qMIN_Y, qMAX_Y, &xiY, &qY, &uY);
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    //                    OBSERVADOR COMPLETO:
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    //                      Control seguidor
    //ctrlChura_SEG4(rP, posP, k1Ps, k2Ps, NP, L1PRY, L2PRY, bP, &x1P, &x2P, &uP); 
    //ctrlChura_SEG4(rR, posR, k1Rs, k2Rs, NR, L1PRY, L2PRY, bR, &x1R, &x2R, &uR);
    //ctrlChura_SEG4(rY, posY, k1Ys, k2Ys, NY, L1PRY, L2PRY, bY, &x1Y, &x2Y, &uY);
    //                      Control robusto
    //ctrlChura_ROB3(rP, posP, k1Pr, k2Pr, kiPr, L1PRY, L2PRY, bP, xiMIN_P, xiMAX_P, uMIN_P, uMAX_P, &x1P, &x2P, &xiP, &uP);
    //ctrlChura_ROB3(rR, posR, k1Rr, k2Rr, kiRr, L1PRY, L2PRY, bR, xiMIN_R, xiMAX_R, uMIN_R, uMAX_R, &x1R, &x2R, &xiR, &uR);
    //ctrlChura_ROB3(rY, posY, k1Yr, k2Yr, kiYr, L1PRY, L2PRY, bY, xiMIN_Y, xiMAX_Y, uMIN_Y, uMAX_Y, &x1Y, &x2Y, &xiY, &uY);
    tiempo_antr = tiempo_act;
    //dtt = tiempo_act/1000;
  }//*/
  //**************************************************YAW***********************************************************
  if((tiempo_act-tiempo_antY)>=TY){
    //                    OBSERVADOR REDUCIDO:
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    //                    Control seguidor
    //ctrlChura_SEG2(rY, posY, velY, k1Ys, k2Ys, NY, LPRY, bY, cYs, qMIN_Y, qMAX_Y, &qY, &uY); //observador posicion
    //                    Control robusto
    ctrlChura_ROB2(rY, posY, velY, k1Yr, k2Yr, kiYr, LY, bY, cYr, xiMIN_Y, xiMAX_Y, uMIN_Y, uMAX_Y, qMIN_Y, qMAX_Y, &xiY, &qY, &uY);
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    //                    OBSERVADOR COMPLETO:
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    //                      Control seguidor
    //ctrlChura_SEG4(rY, posY, k1Ys, k2Ys, NY, L1PRY, L2PRY, bY, &x1Y, &x2Y, &uY);
    //                      Control robusto
    //ctrlChura_ROB3(rY, posY, k1Yr, k2Yr, kiYr, L1PRY, L2PRY, bY, xiMIN_Y, xiMAX_Y, uMIN_Y, uMAX_Y, &x1Y, &x2Y, &xiY, &uY);
    tiempo_antY = tiempo_act;
    //dtt = tiempo_act/1000;
  }//*/
  //****************************************TRASLACIONAL*************************************************
  if((tiempo_act-tiempo_antt)>=Tt){
    //                    OBSERVADOR REDUCIDO:
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    //                    Control seguidor
    //ctrlChura_SEG2(rZ, posZ, velZ, k1Zs, k2Zs, NZ, LZ, bZ, cZs, qMIN_Z, qMAX_Z, &qZ, &uZ); //observador posicion
    //                    Control robusto
    //ctrlChura_ROB2(rZ, posZ, velZ, k1Zr, k2Zr, kiZr, LZ, bZ, cZr, xiMIN_Z, xiMAX_Z, uMIN_Z, uMAX_Z, qMIN_Z, qMAX_Z, &xiZ, &qZ, &uZ);
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    //                    OBSERVADOR COMPLETO:
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    //                      Control seguidor
    ctrlChura_SEG4(rZ, posZ, k1Zs, k2Zs, NZ, L1Z, L2Z, bZ, &x1Z, &x2Z, &uZ); 
    //                      Control robusto
    //ctrlChura_ROB3(rZ, posZ, k1Zr, k2Zr, kiZr, L1Z, L2Z, bZ, xiMIN_Z, xiMAX_Z, uMIN_Z, uMAX_Z, &x1Z, &x2Z, &xiZ, &uZ);
    tiempo_antt = tiempo_act;
    //dtt = tiempo_act/1000;
  }//*/
  //vTaskDelay(100);
  //return u;
  //xiZ = xiZ;
    // VELOCIDAD DEL MOTOR:
    empR = 50.0f;
    empT = 0.0f;//10.0f;//50.0f;//17.0f;//22.0f;//14;//25.0f;//50.0f;//32.0f;//20.0f;

    uP1 = uP;
    uR1 = uR;//0.0f;
    uY1 = uY;
    uZ1 = uZ;//0.0f;//constrain(uZ,0.0f,1.0f);//0.0f;

    rot1 =  hoverR - (a1*uR1) + (a1*uP1) - (a2*uY1);
    rot2 =  hoverR - (a1*uR1) - (a1*uP1) + (a2*uY1);
    rot3 =  hoverR + (a1*uR1) - (a1*uP1) - (a2*uY1);
    rot4 =  hoverR + (a1*uR1) + (a1*uP1) + (a2*uY1);

    tras = hoverT + (a3*uZ1);
//(float)sqrt
    rot_1 = (float)sqrt(constrain(rot1,velMINch,velMAXch));
    rot_2 = (float)sqrt(constrain(rot2,velMINch,velMAXch));
    rot_3 = (float)sqrt(constrain(rot3,velMINch,velMAXch));
    rot_4 = (float)sqrt(constrain(rot4,velMINch,velMAXch));

    tras_ = (float)sqrt(constrain(tras,velMINch,velMAXch));

    //ctrl_1 = hover + (a3*uZ1) - (a1*uR1) + (a1*uP1) - (a2*uY1);
    //ctrl_2 = hover + (a3*uZ1) - (a1*uR1) - (a1*uP1) + (a2*uY1);
    //ctrl_3 = hover + (a3*uZ1) + (a1*uR1) - (a1*uP1) - (a2*uY1);
    //ctrl_4 = hover + (a3*uZ1) + (a1*uR1) + (a1*uP1) + (a2*uY1);

    VEL1_ = (empT*tras_) + (empR*rot_1);
    VEL2_ = (empT*tras_) + (empR*rot_2);
    VEL3_ = (empT*tras_) + (empR*rot_3);
    VEL4_ = (empT*tras_) + (empR*rot_4);
    
    ///////////////////////////////////////////////////
    //VEL1 = fabs(sqrt(ctrl_1));
    //VEL2 = fabs(sqrt(ctrl_2));
    //VEL3 = fabs(sqrt(ctrl_3));
    //VEL4 = fabs(sqrt(ctrl_4));
    //////////////////////////////////////////////////*/

    VEL1 = (vel_HOVER + VEL1_);//Seg 9.6
    VEL2 = (vel_HOVER + VEL2_);//Rob 2.6
    VEL3 = (vel_HOVER + VEL3_);//Seg 9.6
    VEL4 = (vel_HOVER + VEL4_);//Rob 2.6

    ledSet(LED_RED_L,1);
  }

/////////////// PITCH ///////////////
  referenciaP = rP;
  velocidadP = velP;
  salidaP = posP;
  integralP = xiP;
  entradaP = uP*1000;
/////////////// ROLL ///////////////
  velocidadR = velR;
  salidaR = posR;
  integralR = xiR;
  entradaR = uR;
  /////////////// YAW ///////////////
  referenciaY = rY;
  velocidadY = qY;//qY;
  salidaY = posY;//posY;
  integralY = xiY;
  entradaY = 1000*uY;
  /////////////// Z ///////////////
  velocidadZ = x2Z;
  salidaZ = x1Z;
  integralZ = xiZ;
  entradaZ = 10*uZ;

////////////// VELOCIDADES //////////////
  vel_1 = VEL1;
  vel_2 = VEL2;
  vel_3 = VEL3;
  vel_4 = VEL4;
  Tss = dtt;
/*****************************************************************************/
}
      // Reset de variables
void controlChuraRESET(void)
{
  xiP = 0.0f;//
  xiR = 0.0f;
  xiY = 0.0f;
  xiZ = 0.0f;

  x1P = 0.0f;
  x2P = 0.0f;
  x1R = 0.0f;
  x2R = 0.0f;
  x1Y = 0.0f;//3.1416f;
  x2Y = 0.0f;
  x1Z = 0.01f;
  x2Z = 0.0f;

  qP = 0.0f;
  qR = 0.0f;
  qY = 0.0f;
  qZ = 0.0f;

  posP = 0.0f;
  posR = 0.0f;
  posY = 0.0f;
  posZ = 0.0f;

  velP = 0.0f;
  velR = 0.0f;
  velY = 0.0f;
  velZ = 0.0f;

  uP = 0.0f;
  uR = 0.0f;
  uY = 0.0f;
  uZ = 0.0f;

  //rZ = 0.0f;
dtt = 0.0f;
tiempo_act = 0;
tiempo_antr = 0;
tiempo_antt = 0;

  VEL1 = 0.0f;
  VEL2 = 0.0f;
  VEL3 = 0.0f;
  VEL4 = 0.0f;
/*
  r = 0.0f;
  x1 = 0.0f;
  x2 = 0.0f;
  
 // xi = 0.0f;
  u = 0.0f;
  uS = 0.0f;
  y = 0.0f;
  *q = 0.0f;
// ur = 0.0f;
// urr = 0.0f;
  r_z = 0.0f;
  y_z = 0.0f;
  u_z = 0.0f;
  x_1 = 0.0f;
  x_2 = 0.0f;
  windup = 0.0f;*/
}
      //  control SEGUIDOR
float ctrlChura_SEG(float ref, float pos, float vel, float k1, float k2, float N){
  float rs,x1s,x2s,us;                        //SEGUIDOR
  rs = ref;
  x1s = pos;
  x2s = vel;
  us = -(k1*x1s + k2*x2s) + N*rs;
  return us;
}
void ctrlChura_SEG2(float ref, float pos, float vel, float k1, float k2, float N, float L, float b, float c, float qMIN, float qMAX, float *qss, float *uss){
  float rss,x1ss,x2ss,yss,F,G,H; 
  F = -L;
  G = -L*L - 1.0f - 2.0f*L;
  H = b*(1.0f - L);
             //OBSERVADOR POSICION
  rss = ref;
  x1ss = pos;

  yss = x1ss;
  x2ss = L*yss + (*qss) + c*vel;
  (*uss) = -((k1*x1ss) + (k2*x2ss)) + (N*rss);
  (*qss) = F*(*qss) + G*yss + H*(*uss);//- (0.612f*yss) + (1.218f*b*(*uss));//G*yss + (1-L)*b*(*uss);//
  //(*qss) = constrain((*qss),qMIN,qMAX);
}
void ctrlChura_SEG3(float ref, float vel, float k1, float k2, float N, float L, float b, float qMIN, float qMAX, float *qov, float *uov){
  float rov,x1ov,x2ov,yov; //G 
              //OBSERVADOR VELOCIDAD
  rov = ref;
  x2ov = vel;
  yov = x2ov;
  x1ov = L*yov + (*qov);
  *uov = -(k1*x1ov + k2*x2ov) + N*rov;
  *qov = -L*(*qov) - 0.612f*yov + (1-L)*b*(*uov);
  *qov = constrain(*qov,qMIN,qMAX);
} 
void ctrlChura_SEG4(float ref, float pos, float k1, float k2, float N, float L1, float L2, float b, float *x1o, float *x2o, float *uo){
  float ro,yo,x1ant,x2ant;
  ro = ref;
  x1ant = (*x1o);
  x2ant = (*x2o);
  yo = x1ant;
  //yo = (*x1o);
  //yo = pos;//b*(x1ant + x2ant);
  (*uo) = -(k1*x1ant + k2*x2ant) + N*ro;
  (*x1o) = (2-L1)*x1ant + x2ant + L1*yo + b*(*uo);  //FCO
  (*x2o) = -(1+L2)*x1ant + L2*yo + b*(*uo);
  //(*x1o) = -b*L1*x1ant + (1-b*L1)*x2ant + L1*yo;      //FCC
  //(*x2o) = -(1+b*L2)*x1ant + (2-b*L2)*x2ant + L2*yo + (*uo);
}
float ctrlChura_SEG_OBSV(float ref, float k1, float k2, float N, float L1, float L2, float *x_1, float *x_2){
 // float y_z,u_z;
  //r_z = ref;
  y_z = *x_1;
  /*
  u_z = -(k1*(*x_1) + k2*(*x_2)) + N*r_z;
  *x_1 = 0.0474f*(*x_1) + (*x_2) + L1*y_z + 0.535f*u_z;
  *x_2 = -0.0454f*(*x_1) - L2*y_z + 0.535f*u_z;
  */
  u_z = -(0.8108f*(*x_1) + 0.6083f*(*x_2)) + 0.2025f*0.1f;
  *x_1 = 0.0474f*(*x_1) + (*x_2) + 1.9526f*y_z + 0.535f*u_z;
  *x_2 = -0.0454f*(*x_1) - 0.9546f*y_z + 0.535f*u_z;
  //vTaskDelay(170);
  return u_z;
}
  /*    //  control PID
float ctrlChura_PID(float ref, float pos, float kp, float ki, float kd, float kw, float uMAX, float *e1, float *e2, float *u){
  float y,e,r,x1,x2,xiact,sat,u;
  y = pos;
  r = ref;
  e = r - y;
  if(*u<uMAX && *u>-uMAX){
    *u = *u + e*(kp + ki + kd)/cP - (*e1)*(kp + 2*kd)/cP + *e2*kd/cP;
    windup = 0;
  }
  else{
    if(e<0){
      windup = -uMAX;
    }
    else{
      windup = uMAX;
    }
    *u = *u + e*(kp + ki + kd)/cP - (*e1)*(kp + 2*kd)/cP + *e2*kd/cP + kw*windup;
    *u = *u/(1+kw);
  }
  *e2 = *e1;
  *e1 = e;
  float U1 = *u;
  return U1;
}*/

      //  control ROBUSTO
void ctrlChura_ROB(float ref, float pos, float vel, float k1, float k2, float ki, float b, float xiMIN, float xiMAX, float uMIN, float uMAX, float *xir, float *ur){
  float rr,x1r,x2r,yr,er,xiactr;           //ROBUSTO
  float vr;
  rr = ref;
  x1r = pos;
  x2r = vel;
  yr = x1r;
  er = rr - yr;
  //  ANTI-WINDWUP CLAMPING
  vr = constrain((*ur),uMIN,uMAX);
  if (((*ur)!=vr)){//} && (er*(*ur))>0.0f){
    er = 0.0f;
  }
  /*else{
    er=er;
  }*/
  xiactr = (*xir) + er;
  (*ur) = -(k1*x1r + k2*x2r) + ki*xiactr;
  (*xir) = xiactr;
  //(*xir) = constrain((*xir),xiMIN,xiMAX);
}
      //  control ROBUSTO 2
void ctrlChura_ROB2(float ref, float pos, float vel, float k1, float k2, float ki, float L, float b, float c, float xiMIN, float xiMAX, float uMIN, float uMAX, float qMIN, float qMAX, float *xirr, float *qrr, float *urr){
  float rrr,x1rr,x2rr,yrr,err,xiactrr,G;    //ROBUSTO 2
  float vrr;
  rrr = ref;
  x1rr = pos;
  //vTaskDelay(80);
  yrr = x1rr;
  x2rr = L*yrr + (*qrr) + c*vel;
  err = rrr - yrr;
  //  ANTI-WINDWUP CLAMPING
  vrr = constrain((*urr),uMIN,uMAX);
  if (((*urr) != vrr) && ((err*(*urr))>0.0f)){
    err = 0.0f;
  }/*
  else{
    err=err;
  }*/
  xiactrr = (*xirr) + err;
  (*urr) = -(k1*x1rr + k2*x2rr) + ki*xiactrr;
  G = -(L*L + 1.0f + 2*L);
  (*qrr) = -L*(*qrr) + G*yrr + (1-L)*b*(*urr);
  (*qrr) = constrain((*qrr),qMIN,qMAX);
  (*xirr) = xiactrr;
  (*xirr) = constrain((*xirr),xiMIN,xiMAX);
}
      //  control ROBUSTO 3
void ctrlChura_ROB3(float ref, float pos, float k1, float k2, float ki, float L1, float L2, float b, float xiMIN, float xiMAX, float uMIN, float uMAX, float *x1or, float *x2or, float *xior, float *uor){
  float ror,yor,eor,xiactor,x1antr,x2antr;    //ROBUSTO 2
 // float vor;
  ror = ref;
  x1antr = (*x1or);
  x2antr = (*x2or);
  //vTaskDelay(80);
  yor = x1antr;//pos;//(*x1or);
  eor = ror - yor;
  //  ANTI-WINDWUP CLAMPING
 /* vor = constrain((*uor),uMIN,uMAX);
  if ((*uor)!=vor && eor*(*uor)>0.0f){
    eor = 0.0f;
  }
  else{
    eor=eor;
  }*/
  xiactor = (*xior) + eor;
  (*uor) = -(k1*x1antr + k2*x2antr) + ki*xiactor;
  (*x1or) = (2-L1)*x1antr + x2antr + L1*yor + b*(*uor); //FCO
  (*x2or) = -(1+L2)*x1antr + L2*yor + b*(*uor);
  //(*x1or) = -b*L1*x1antr + (1-b*L1)*x2antr + L1*yor;    //FCC
  //(*x2or) = -(1+b*L2)*x1antr + (2-b*L2)*x2antr + L2*yor + (*uor);
  (*xior) = xiactor;
  //(*xior) = constrain((*xior),xiMIN,xiMAX);
}
      // Limitar variables:
float restringir(float valor, float valorMIN, float valorMAX){
  if(valor<=valorMIN){
    valor = valorMIN;
  }
  else if(valor>=valorMAX){
    valor = valorMAX;
  }
  else{
    valor = valor;
  }
  return valor;
}
//////////////////////////////////////////
  //CONTROL DE PITCH:
LOG_GROUP_START(CHURAP)
/**
 * @brief Ref_P
 */
LOG_ADD(LOG_FLOAT, referenciaP, &referenciaP)
/**
 * @brief Vel_Pitch
 */
LOG_ADD(LOG_FLOAT, velocidadP, &velocidadP)
/**
 * @brief Salida_Pitch
 */
LOG_ADD(LOG_FLOAT, salidaP, &salidaP)
/**
 * @brief Integral_Pitch
 */
LOG_ADD(LOG_FLOAT, integralP, &integralP)
/**
 * @brief Entrada_Pitch
 */
LOG_ADD(LOG_FLOAT, entradaP, &entradaP)
/**
 * @brief tiempo_muestra
 */
LOG_ADD(LOG_FLOAT, Tss, &Tss)
LOG_GROUP_STOP(CHURAP)

  //CONTROL DE ROLL:
LOG_GROUP_START(CHURAR)
/**
 * @brief Vel_Roll
 */
LOG_ADD(LOG_FLOAT, velocidadR, &velocidadR)
/**
 * @brief Salida_Roll
 */
LOG_ADD(LOG_FLOAT, salidaR, &salidaR)
/**
 * @brief Integral_Roll
 */
LOG_ADD(LOG_FLOAT, integralR, &integralR)
/**
 * @brief Entrada_Roll
 */
LOG_ADD(LOG_FLOAT, entradaR, &entradaR)
LOG_GROUP_STOP(CHURAR)

//CONTROL DE YAW:
LOG_GROUP_START(CHURAY)
/**
 * @brief Ref_Y
 */
LOG_ADD(LOG_FLOAT, referenciaY, &referenciaY)
/**
 * @brief Vel_Yaw
 */
LOG_ADD(LOG_FLOAT, velocidadY, &velocidadY)
/**
 * @brief Salida_Yaw
 */
LOG_ADD(LOG_FLOAT, salidaY, &salidaY)
/**
 * @brief Integral_Yaw
 */
LOG_ADD(LOG_FLOAT, integralY, &integralY)
/**
 * @brief Entrada_Yaw
 */
LOG_ADD(LOG_FLOAT, entradaY, &entradaY)
LOG_GROUP_STOP(CHURAY)

  //CONTROL DE Z:
LOG_GROUP_START(CHURAZ)
/**
 * @brief Vel_Z
 */
LOG_ADD(LOG_FLOAT, velocidadZ, &velocidadZ)
/**
 * @brief Salida_Z
 */
LOG_ADD(LOG_FLOAT, salidaZ, &salidaZ)
/**
 * @brief Integral_Z
 */
LOG_ADD(LOG_FLOAT, integralZ, &integralZ)
/**
 * @brief Entrada_Z
 */
LOG_ADD(LOG_FLOAT, entradaZ, &entradaZ)
LOG_GROUP_STOP(CHURAZ)
/**************************************************/
//    VELOCIDAD DE MOTORES:
LOG_GROUP_START(CHURAv)
/**
 * @brief Velocidad1
 */
LOG_ADD(LOG_FLOAT, vel_1, &vel_1)
/**
 * @brief Velocidad2
 */
LOG_ADD(LOG_FLOAT, vel_2, &vel_2)
/**
 * @brief Velocidad3
 */
LOG_ADD(LOG_FLOAT, vel_3, &vel_3)
/**
 * @brief Velocidad4
 */
LOG_ADD(LOG_FLOAT, vel_4, &vel_4)
LOG_GROUP_STOP(CHURAv)

/**
 * Logging variables for the command and reference signals for the
 * altitude PID controller
 */
LOG_GROUP_START(controller)
/**
 * @brief Thrust command
 */
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
/**
 * @brief Roll command
 */
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
/**
 * @brief Pitch command
 */
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
/**
 * @brief yaw command
 */
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
/**
 * @brief Gyro roll measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
/**
 * @brief Gyro pitch measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
/**
 * @brief Yaw  measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
/**
 * @brief Acceleration in the zaxis in G-force
 */
LOG_ADD(LOG_FLOAT, accelz, &accelz)
/**
 * @brief Thrust command without (tilt)compensation
 */
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
/**
 * @brief Desired roll setpoint
 */
LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
/**
 * @brief Desired pitch setpoint
 */
LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
/**
 * @brief Desired yaw setpoint
 */
LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
/**
 * @brief Desired roll rate setpoint
 */
LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll)
/**
 * @brief Desired pitch rate setpoint
 */
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
/**
 * @brief Desired yaw rate setpoint
 */
LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw)
LOG_GROUP_STOP(controller)


/**
 * Controller parameters
 */
PARAM_GROUP_START(controller)
/**
 * @brief Nonzero for tilt compensation enabled (default: 0)
 */
PARAM_ADD(PARAM_UINT8, tiltComp, &tiltCompensationEnabled)
PARAM_GROUP_STOP(controller)
