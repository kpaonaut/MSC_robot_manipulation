/*  File    : CMT_TRQ.c
 *  Abstract:
 *
 *      Main routines for computing torques
 *
 *	Copyright (c) 11/09/2014 by Yu Zhao. All Rights Reserved.
 *  Revised: 06/10/2016, by Wenjie Chen, for TLC implementation of S-Function and with model parameter update options
 */


#ifdef MATLAB_MEX_FILE
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

#include "math.h"
#include "vmathsim.h"
#include "finvsim.h"


// forward defines
static void rot_mat (Link *l, Rot *R, Vect *r, double th, double d);
//*************************

// robot parameters
static Vect gravity={0.0,0.0,9.80665};
static int njoints=6;
static Vect z0={0,0,1};
static Vect zero={0,0,0};
static int nq=1;//calculate torque for only one point

//*******************
// link cog
static Vect rbar1={-0.031409041,   -0.07709071,  -0.003370095};
static Vect rbar2={-0.307386075,   -0.007107989,  0.024780079};
static Vect rbar3={-0.029206867,   0.020732428,  -0.00590663};
static Vect rbar4={-0.000245714,   0.213938872,  -0.002822415};
static Vect rbar5={0.000048044,   0.004438151,  -0.027383968};
// static Vect rbar6={-0.00012,                0,  -0.00947};
static Vect rbar6={0.000013564547335,0.000026039162710,0.073013473215935};

// link inertia
static double I1[9]={0.023317972863,0,0,                   0,0.019377617871,0,                        0,0,0.013914152634};
static double I2[9]={0.032897173905,0,0,                    0,0.207036579327,0,                         0,0,0.188399193896};
static double I3[9]={0.008101947424,0,0,                    0,0.006869708266,0,                         0,0,0.007981851348};
static double I4[9]={0.052910105926,0,0,                  0,0.005704618963,0,                        0,0,0.053150971445};
static double I5[9]={0.00247955645,0,0,                  0,0.002437545307,0,                       0,0,0.001200343197};
// static double I6[9]={0.0000318,0,0,                      0,0.0000315,0,                            0,0,0.0000542};
static double I6[9]={0.004812870550095,-0.000000160668691,-0.000003748192089,                      -0.000000160668691,0.004049903849694,0.000001947046491,                            -0.000003748192089,0.000001947046491,0.001305983560329};

//******************
// coulomb friction (lumped at motor side)
static Vect2 Tc1={0.103,-0.103};
static Vect2 Tc2={0.103,-0.103};
static Vect2 Tc3={0.082,-0.082};
static Vect2 Tc4={0.0877,-0.0877};
static Vect2 Tc5={0.0235,-0.0235};
static Vect2 Tc6={0.0567,-0.0567};

//***************************
//Link:alpha,A,D,theta,offset,*rbar,m,*I,Jm,G,B,*Tc (B=viscous damping lumped at motor side)
// Jm is changed here, m is changed here
static Link l1={-1.570796326794897,     0.05,   0,      0,                  0,                   &rbar1, 2.39847,   I1,    8.942106763972944e-05,           1.145945945945946e+02,   0.000467915532690172,      &Tc1};
static Link l2={3.141592653589793,      0.44,   0,      0,                  -1.570796326794897,  &rbar2, 7.801869,  I2,    6.010000000000000e-05,           121.0,                   0.00056340849854531,       &Tc2};
static Link l3={-1.570796326794897,     0.035,  0,      0,                  0,                   &rbar3, 2.984721,  I3,    5.193730387143901e-05,           1.020689655172414e+02,   0.000229183118052329,      &Tc3};
static Link l4={1.570796326794897,      0,      -0.42,  0,                  0,                   &rbar4, 4.144178,  I4,    7.177676417233561e-05,           73.043478260869563,      0.000303667631419336,      &Tc4};
static Link l5={-1.570796326794897,     0,      0,      0,                  0,                   &rbar5, 1.70042,   I5,    1.441192000000000e-05,           83.333333333333329,      0.000106952121757754,      &Tc5};
// static Link l6={3.141592653589793,      0,      -0.08,  0,                  0,                   &rbar6, 0.17,      I6,    1.734483854166667e-05,           41.379310344827587,      0.000211994384198405,      &Tc6};
static Link l6={3.141592653589793,      0,      -0.08,  0,                  0,                   &rbar6, 1.46782141,I6,    1.734483854166667e-05,           41.379310344827587,      0.000211994384198405,      &Tc6};

// saturation gain
static double SG[6]={9.549296585513721,9.549296585513721,9.549296585513721,9.549296585513721,9.549296585513721,9.549296585513721};
//***************************

static struct _link* M16iB[6]={&l1,&l2,&l3,&l4,&l5,&l6};
//*************************

// useful alias
#define M(j)        (M16iB[j]->m)
#define R_COG(j)    (M16iB[j]->rbar)
#define INERTIA(j)  (M16iB[j]->I)
//*************************

// global variables
static struct matrix ROT[6];
static struct vector PSTAR[6];
static struct vector OMEGA[6];
static struct vector OMEGADOT[6];    
static struct vector ACC[6];
static struct vector ACC_COG[6];
static struct vector f[6];
static struct vector n[6];
//*************************

// to change the static parameter values according to different models
void param_change(int model_no)
{
    switch (model_no) {
        case 1:  /* LR Mate 200iD/7L without gripper */
        {
            // link cog
            Vect rbar1_tmp={-0.031409041,   -0.07709071,  -0.003370095};
            Vect rbar2_tmp={-0.307386075,   -0.007107989,  0.024780079};
            Vect rbar3_tmp={-0.029206867,   0.020732428,  -0.00590663};
            Vect rbar4_tmp={-0.000245714,   0.213938872,  -0.002822415};
            Vect rbar5_tmp={ 0.000048044,   0.004438151,  -0.027383968};
            Vect rbar6_tmp={-0.00012,                0,   -0.00947};
            
            vect_assign(&rbar1, &rbar1_tmp);
            vect_assign(&rbar2, &rbar2_tmp);
            vect_assign(&rbar3, &rbar3_tmp);
            vect_assign(&rbar4, &rbar4_tmp);
            vect_assign(&rbar5, &rbar5_tmp);
            vect_assign(&rbar6, &rbar6_tmp);

            // link inertia
            double I1_tmp[9]={0.023317972863,0,0,                   0,0.019377617871,0,                        0,0,0.013914152634};
            double I2_tmp[9]={0.032897173905,0,0,                    0,0.207036579327,0,                         0,0,0.188399193896};
            double I3_tmp[9]={0.008101947424,0,0,                    0,0.006869708266,0,                         0,0,0.007981851348};
            double I4_tmp[9]={0.052910105926,0,0,                  0,0.005704618963,0,                        0,0,0.053150971445};
            double I5_tmp[9]={0.00247955645,0,0,                  0,0.002437545307,0,                       0,0,0.001200343197};
            double I6_tmp[9]={0.0000318,0,0,                      0,0.0000315,0,                            0,0,0.0000542};
            
            double9_assign(I1, I1_tmp);
            double9_assign(I2, I2_tmp);
            double9_assign(I3, I3_tmp);
            double9_assign(I4, I4_tmp);
            double9_assign(I5, I5_tmp);
            double9_assign(I6, I6_tmp);

            // coulomb friction (lumped at motor side)
            Vect2 Tc1_tmp={0.103,-0.103};
            Vect2 Tc2_tmp={0.103,-0.103};
            Vect2 Tc3_tmp={0.082,-0.082};
            Vect2 Tc4_tmp={0.0877,-0.0877};
            Vect2 Tc5_tmp={0.0235,-0.0235};
            Vect2 Tc6_tmp={0.0567,-0.0567};
            
            vect2_assign(&Tc1, &Tc1_tmp);
            vect2_assign(&Tc2, &Tc2_tmp);
            vect2_assign(&Tc3, &Tc3_tmp);
            vect2_assign(&Tc4, &Tc4_tmp);
            vect2_assign(&Tc5, &Tc5_tmp);
            vect2_assign(&Tc6, &Tc6_tmp);

            //Link:alpha,A,D,theta,offset,*rbar,m,*I,Jm,G,B,*Tc (B=viscous damping lumped at motor side)
            Link l1_tmp={-1.570796326794897,     0.05,   0,      0,                  0,                   &rbar1, 2.39847,   I1,    8.942106763972944e-05,           1.145945945945946e+02,   0.000467915532690172,      &Tc1};
            Link l2_tmp={3.141592653589793,      0.44,   0,      0,                  -1.570796326794897,  &rbar2, 7.801869,  I2,    6.010000000000000e-05,           121.0,                   0.00056340849854531,       &Tc2};
            Link l3_tmp={-1.570796326794897,     0.035,  0,      0,                  0,                   &rbar3, 2.984721,  I3,    5.193730387143901e-05,           1.020689655172414e+02,   0.000229183118052329,      &Tc3};
            Link l4_tmp={1.570796326794897,      0,      -0.42,  0,                  0,                   &rbar4, 4.144178,  I4,    7.177676417233561e-05,           73.043478260869563,      0.000303667631419336,      &Tc4};
            Link l5_tmp={-1.570796326794897,     0,      0,      0,                  0,                   &rbar5, 1.70042,   I5,    1.441192000000000e-05,           83.333333333333329,      0.000106952121757754,      &Tc5};
            Link l6_tmp={3.141592653589793,      0,      -0.08,  0,                  0,                   &rbar6, 0.17,      I6,    1.734483854166667e-05,           41.379310344827587,      0.000211994384198405,      &Tc6};
            
            link_assign(&l1, &l1_tmp);
            link_assign(&l2, &l2_tmp);
            link_assign(&l3, &l3_tmp);
            link_assign(&l4, &l4_tmp);
            link_assign(&l5, &l5_tmp);
            link_assign(&l6, &l6_tmp);
            
            break;
        }
        
        case 2:  /* LR Mate 200iD with gripper */
        {
            // link cog
            Vect rbar1_tmp={-0.032489403,   -0.069929718,   -0.002676405};
            Vect rbar2_tmp={-0.238031347,   -0.006312797,    0.024755017};
            Vect rbar3_tmp={-0.027961724,    0.021143786,   -0.006038888};
            Vect rbar4_tmp={-0.000279767,    0.175753904,   -0.002956255};
            Vect rbar5_tmp={ 0.000048044,    0.004438151,   -0.027383968};
            Vect rbar6_tmp={ 0.000011572,    0.000002843,    0.111399209};
            
            vect_assign(&rbar1, &rbar1_tmp);
            vect_assign(&rbar2, &rbar2_tmp);
            vect_assign(&rbar3, &rbar3_tmp);
            vect_assign(&rbar4, &rbar4_tmp);
            vect_assign(&rbar5, &rbar5_tmp);
            vect_assign(&rbar6, &rbar6_tmp);

            // link inertia
            double I1_tmp[9]={0.0217873,0,0,                      0,0.0182422,0,                            0,0,0.0121832};
            double I2_tmp[9]={0.0337788,0,0,                      0,0.1249452,0,                            0,0,0.1046281};
            double I3_tmp[9]={0.0078802,0,0,                      0,0.0067803,0,                            0,0,0.0078177};
            double I4_tmp[9]={0.0273623,0,0,                      0,0.0050257,0,                            0,0,0.0275217};
            double I5_tmp[9]={0.0024796,0,0,                      0,0.0024375,0,                            0,0,0.0012003};
            double I6_tmp[9]={0.014065, 0,-0.000004,              0,0.009654,-0.000001,                     -0.000004,-0.000001,0.006171};
            
            double9_assign(I1, I1_tmp);
            double9_assign(I2, I2_tmp);
            double9_assign(I3, I3_tmp);
            double9_assign(I4, I4_tmp);
            double9_assign(I5, I5_tmp);
            double9_assign(I6, I6_tmp);

            // coulomb friction (lumped at motor side)
            Vect2 Tc1_tmp={0.112,-0.112};
            Vect2 Tc2_tmp={0.117,-0.117};
            Vect2 Tc3_tmp={0.118,-0.118};
            Vect2 Tc4_tmp={0.15,-0.15};
            Vect2 Tc5_tmp={0.029,-0.029};
            Vect2 Tc6_tmp={0.049,-0.049};
            
            vect2_assign(&Tc1, &Tc1_tmp);
            vect2_assign(&Tc2, &Tc2_tmp);
            vect2_assign(&Tc3, &Tc3_tmp);
            vect2_assign(&Tc4, &Tc4_tmp);
            vect2_assign(&Tc5, &Tc5_tmp);
            vect2_assign(&Tc6, &Tc6_tmp);

            //Link:alpha,A,D,theta,offset,*rbar,m,*I,Jm,G,B,*Tc (B=viscous damping lumped at motor side)
            Link l1_tmp={-1.570796326794897,     0.05,   0,      0,                  0,                   &rbar1, 2.131767,  I1,    8.942106763972944e-05,           91.53846154,             0.00026738,       &Tc1};
            Link l2_tmp={3.141592653589793,      0.33,   0,      0,                  -1.570796326794897,  &rbar2, 7.653326,  I2,    6.010000000000000e-05,           101.0,                   0.000706648,      &Tc2};
            Link l3_tmp={-1.570796326794897,     0.035,  0,      0,                  0,                   &rbar3, 2.907898,  I3,    5.193730387143901e-05,           80.0,                    0.000315127,      &Tc3};
            Link l4_tmp={1.570796326794897,      0,      -0.335,  0,                  0,                  &rbar4, 3.697198,  I4,    7.177676417233561e-05,           73.043478260869563,      0.000315127,      &Tc4};
            Link l5_tmp={-1.570796326794897,     0,      0,      0,                  0,                   &rbar5, 1.70042,   I5,    1.441192000000000e-05,           83.333333333333329,      8.59437e-05,      &Tc5};
            Link l6_tmp={3.141592653589793,      0,      -0.08,  0,                  0,                   &rbar6, 3.138889,  I6,    1.734483854166667e-05,           41.379310344827587,      0.000162338,      &Tc6};
            
            link_assign(&l1, &l1_tmp);
            link_assign(&l2, &l2_tmp);
            link_assign(&l3, &l3_tmp);
            link_assign(&l4, &l4_tmp);
            link_assign(&l5, &l5_tmp);
            link_assign(&l6, &l6_tmp);
            
            break;
        }

        case 3:  /* LR Mate 200iD without gripper */
        {
            // link cog
            Vect rbar1_tmp={-0.032489403,   -0.069929718,   -0.002676405};
            Vect rbar2_tmp={-0.238031347,   -0.006312797,    0.024755017};
            Vect rbar3_tmp={-0.027961724,    0.021143786,   -0.006038888};
            Vect rbar4_tmp={-0.000279767,    0.175753904,   -0.002956255};
            Vect rbar5_tmp={ 0.000048044,    0.004438151,   -0.027383968};
            Vect rbar6_tmp={-0.00012,                  0,   -0.00947};
            
            vect_assign(&rbar1, &rbar1_tmp);
            vect_assign(&rbar2, &rbar2_tmp);
            vect_assign(&rbar3, &rbar3_tmp);
            vect_assign(&rbar4, &rbar4_tmp);
            vect_assign(&rbar5, &rbar5_tmp);
            vect_assign(&rbar6, &rbar6_tmp);

            // link inertia
            double I1_tmp[9]={0.0217873,0,0,                      0,0.0182422,0,                            0,0,0.0121832};
            double I2_tmp[9]={0.0337788,0,0,                      0,0.1249452,0,                            0,0,0.1046281};
            double I3_tmp[9]={0.0078802,0,0,                      0,0.0067803,0,                            0,0,0.0078177};
            double I4_tmp[9]={0.0273623,0,0,                      0,0.0050257,0,                            0,0,0.0275217};
            double I5_tmp[9]={0.0024796,0,0,                      0,0.0024375,0,                            0,0,0.0012003};
            double I6_tmp[9]={0.0000318,0,0,                      0,0.0000315,0,                            0,0,0.0000542};
            
            double9_assign(I1, I1_tmp);
            double9_assign(I2, I2_tmp);
            double9_assign(I3, I3_tmp);
            double9_assign(I4, I4_tmp);
            double9_assign(I5, I5_tmp);
            double9_assign(I6, I6_tmp);

            // coulomb friction (lumped at motor side)
            Vect2 Tc1_tmp={0.112,-0.112};
            Vect2 Tc2_tmp={0.117,-0.117};
            Vect2 Tc3_tmp={0.118,-0.118};
            Vect2 Tc4_tmp={0.15,-0.15};
            Vect2 Tc5_tmp={0.029,-0.029};
            Vect2 Tc6_tmp={0.049,-0.049};
            
            vect2_assign(&Tc1, &Tc1_tmp);
            vect2_assign(&Tc2, &Tc2_tmp);
            vect2_assign(&Tc3, &Tc3_tmp);
            vect2_assign(&Tc4, &Tc4_tmp);
            vect2_assign(&Tc5, &Tc5_tmp);
            vect2_assign(&Tc6, &Tc6_tmp);

            //Link:alpha,A,D,theta,offset,*rbar,m,*I,Jm,G,B,*Tc (B=viscous damping lumped at motor side)
            Link l1_tmp={-1.570796326794897,     0.05,   0,      0,                  0,                   &rbar1, 2.131767,  I1,    8.942106763972944e-05,           91.53846154,             0.00026738,       &Tc1};
            Link l2_tmp={3.141592653589793,      0.33,   0,      0,                  -1.570796326794897,  &rbar2, 7.653326,  I2,    6.010000000000000e-05,           101.0,                   0.000706648,      &Tc2};
            Link l3_tmp={-1.570796326794897,     0.035,  0,      0,                  0,                   &rbar3, 2.907898,  I3,    5.193730387143901e-05,           80.0,                    0.000315127,      &Tc3};
            Link l4_tmp={1.570796326794897,      0,      -0.335,  0,                  0,                  &rbar4, 3.697198,  I4,    7.177676417233561e-05,           73.043478260869563,      0.000315127,      &Tc4};
            Link l5_tmp={-1.570796326794897,     0,      0,      0,                  0,                   &rbar5, 1.70042,   I5,    1.441192000000000e-05,           83.333333333333329,      8.59437e-05,      &Tc5};
            Link l6_tmp={3.141592653589793,      0,      -0.08,  0,                  0,                   &rbar6, 0.17,      I6,    1.734483854166667e-05,           41.379310344827587,      0.000162338,      &Tc6};
            
            link_assign(&l1, &l1_tmp);
            link_assign(&l2, &l2_tmp);
            link_assign(&l3, &l3_tmp);
            link_assign(&l4, &l4_tmp);
            link_assign(&l5, &l5_tmp);
            link_assign(&l6, &l6_tmp);
            
            break;
        }

        default: /* LR Mate 200iD/7L with gripper */
            // need to assign even for default case so that multiple robots can use different robot models in a same simulink model 
            // (the static variables seem to be shared among different robots in the same simulink model)
            // values are updated to reflect current handware setting (Wenjie Chen, 2016/08/19)
        {
            // link cog
            Vect rbar1_tmp={-0.031409041,   -0.07709071,  -0.003370095};
            Vect rbar2_tmp={-0.307386075,   -0.007107989,  0.024780079};
            Vect rbar3_tmp={-0.029206867,   0.020732428,  -0.00590663};
            Vect rbar4_tmp={-0.000245714,   0.213938872,  -0.002822415};
            Vect rbar5_tmp={ 0.000048044,   0.004438151,  -0.027383968};
            Vect rbar6_tmp={-0.000032124,   -0.000032123,  0.0925557257};
            
            vect_assign(&rbar1, &rbar1_tmp);
            vect_assign(&rbar2, &rbar2_tmp);
            vect_assign(&rbar3, &rbar3_tmp);
            vect_assign(&rbar4, &rbar4_tmp);
            vect_assign(&rbar5, &rbar5_tmp);
            vect_assign(&rbar6, &rbar6_tmp);

            // link inertia
            double I1_tmp[9]={0.023317972863,0,0,                   0,0.019377617871,0,                        0,0,0.013914152634};
            double I2_tmp[9]={0.032897173905,0,0,                    0,0.207036579327,0,                         0,0,0.188399193896};
            double I3_tmp[9]={0.008101947424,0,0,                    0,0.006869708266,0,                         0,0,0.007981851348};
            double I4_tmp[9]={0.052910105926,0,0,                  0,0.005704618963,0,                        0,0,0.053150971445};
            double I5_tmp[9]={0.00247955645,0,0,                  0,0.002437545307,0,                       0,0,0.001200343197};
            double I6_tmp[9]={0.005637,-0.000001,0.000001,                      -0.000001,0.004814,-0.000001,                            0.000001,-0.000001,0.001338};
            
            double9_assign(I1, I1_tmp);
            double9_assign(I2, I2_tmp);
            double9_assign(I3, I3_tmp);
            double9_assign(I4, I4_tmp);
            double9_assign(I5, I5_tmp);
            double9_assign(I6, I6_tmp);

            // coulomb friction (lumped at motor side)
            Vect2 Tc1_tmp={0.103,-0.103};
            Vect2 Tc2_tmp={0.103,-0.103};
            Vect2 Tc3_tmp={0.082,-0.082};
            Vect2 Tc4_tmp={0.0877,-0.0877};
            Vect2 Tc5_tmp={0.0235,-0.0235};
            Vect2 Tc6_tmp={0.0567,-0.0567};
            
            vect2_assign(&Tc1, &Tc1_tmp);
            vect2_assign(&Tc2, &Tc2_tmp);
            vect2_assign(&Tc3, &Tc3_tmp);
            vect2_assign(&Tc4, &Tc4_tmp);
            vect2_assign(&Tc5, &Tc5_tmp);
            vect2_assign(&Tc6, &Tc6_tmp);

            //Link:alpha,A,D,theta,offset,*rbar,m,*I,Jm,G,B,*Tc (B=viscous damping lumped at motor side)
            Link l1_tmp={-1.570796326794897,     0.05,   0,      0,                  0,                   &rbar1, 2.39847,   I1,    8.942106763972944e-05,           1.145945945945946e+02,   0.000467915532690172,      &Tc1};
            Link l2_tmp={3.141592653589793,      0.44,   0,      0,                  -1.570796326794897,  &rbar2, 7.801869,  I2,    6.010000000000000e-05,           121.0,                   0.00056340849854531,       &Tc2};
            Link l3_tmp={-1.570796326794897,     0.035,  0,      0,                  0,                   &rbar3, 2.984721,  I3,    5.193730387143901e-05,           1.020689655172414e+02,   0.000229183118052329,      &Tc3};
            Link l4_tmp={1.570796326794897,      0,      -0.42,  0,                  0,                   &rbar4, 4.144178,  I4,    7.177676417233561e-05,           73.043478260869563,      0.000303667631419336,      &Tc4};
            Link l5_tmp={-1.570796326794897,     0,      0,      0,                  0,                   &rbar5, 1.70042,   I5,    1.441192000000000e-05,           83.333333333333329,      0.000106952121757754,      &Tc5};
            Link l6_tmp={3.141592653589793,      0,      -0.08,  0,                  0,                   &rbar6, 1.476305, I6,    1.734483854166667e-05,           41.379310344827587,      0.000211994384198405,      &Tc6};
            
            link_assign(&l1, &l1_tmp);
            link_assign(&l2, &l2_tmp);
            link_assign(&l3, &l3_tmp);
            link_assign(&l4, &l4_tmp);
            link_assign(&l5, &l5_tmp);
            link_assign(&l6, &l6_tmp);
            
            break;
        }
    }
}

// to compute the torque
void cmt_trq(double *q, double *qd, double *qdd, real_T *tau, int model_no)
{
    int p, j;
    Vect qdv, qddv;
    Vect t1, t2, t3, t4;
    
    double t=0;
    
    // Update the parameter values according to model_no
    param_change(model_no);
    
    //*********************************************************************
    nq = 1;
    //*********************************************************************
    qdv=qddv=zero;
    for (p=0; p<nq; p++){
        
        /* put recursive newton-euler routine here */
        
        /* forward recursion */
        for (j=0; j<njoints; j++){
            //rotation & translation for each link
            rot_mat(M16iB[j],&ROT[j],&PSTAR[j],q[j*nq+p]+M16iB[j]->offset,M16iB[j]->D);
            qdv.z=qd[j*nq+p];
            qddv.z=qdd[j*nq+p];
            //omega[j]
            if (j==0)
                t1=qdv;            
            else
                vect_add(&t1,&OMEGA[j-1],&qdv);
            rot_trans_vect_mult(&OMEGA[j],&ROT[j],&t1);
            //alpha[j]
            if (j==0)
                t3=qddv;
            else {
                vect_add(&t1,&OMEGADOT[j-1],&qddv);
                vect_cross(&t2,&OMEGA[j-1],&qdv);
                vect_add(&t3,&t1,&t2);
            }
            rot_trans_vect_mult(&OMEGADOT[j],&ROT[j],&t3);
            //acc[j]
            vect_cross(&t1,&OMEGADOT[j],&PSTAR[j]);
            vect_cross(&t2,&OMEGA[j],&PSTAR[j]);
            vect_cross(&t3,&OMEGA[j],&t2);
            vect_add(&ACC[j],&t1,&t3);
            if (j==0){
                rot_trans_vect_mult(&t1,&ROT[j],&gravity);
            } else
                rot_trans_vect_mult(&t1,&ROT[j],&ACC[j-1]);
            vect_add(&ACC[j],&ACC[j],&t1);
            //abar[j]
            vect_cross(&t1,&OMEGADOT[j],R_COG(j));
            vect_cross(&t2,&OMEGA[j],R_COG(j));
            vect_cross(&t3,&OMEGA[j],&t2);
            vect_add(&ACC_COG[j],&t1,&t3);
            vect_add(&ACC_COG[j],&ACC_COG[j],&ACC[j]);            
        }
        /* backward recursion */
        for (j = njoints-1; j>=0; j--){
            //f[j]
            scal_mult(&t4,&ACC_COG[j],M(j));
            if (j!=njoints-1){
                rot_vect_mult(&t1,&ROT[j+1],&f[j+1]);
                vect_add(&f[j],&t4,&t1);
            } else
                vect_add(&f[j],&t4,&zero);
            //n[j]
            vect_add(&t2,&PSTAR[j],R_COG(j));
            vect_cross(&t1,&t2,&t4);
            if (j!=njoints-1){
                rot_trans_vect_mult(&t2,&ROT[j+1],&PSTAR[j]);
                vect_cross(&t3,&t2,&f[j+1]);
                
                vect_add(&t3,&t3,&n[j+1]);
                rot_vect_mult(&t2,&ROT[j+1],&t3);
                vect_add(&t1,&t1,&t2);
            } else {
                vect_cross(&t2,&PSTAR[j],&zero);
                vect_add(&t1,&t1,&t2);
                vect_add(&t1,&t1,&zero);
            }
            mat_vect_mult(&t2,INERTIA(j),&OMEGADOT[j]);
            mat_vect_mult(&t3,INERTIA(j),&OMEGA[j]);
            vect_cross(&t4,&OMEGA[j],&t3);
            vect_add(&t2,&t2,&t4);
            vect_add(&n[j],&t1,&t2);
        }
        /* compute torque total for each axis */
        for (j=0; j<njoints; j++){
            rot_trans_vect_mult(&t1,&ROT[j],&z0);
            vect_dot(&t,&n[j],&t1);
            /* add actuator dynamics and friction */
                     
//             t  += (M16iB[j]->G)*((M16iB[j]->Jm)*(M16iB[j]->G)*qdd[j*nq+p]+
//                     (M16iB[j]->B)*(M16iB[j]->G)*qd[j*nq+p]+
//                     ((qd[j*nq+p]>0 ? (M16iB[j]->Tc->x) : 0.0)+
//                      (qd[j*nq+p]<0 ? (M16iB[j]->Tc->y) : 0.0)) *
//                               (atan(5.*fabs(qd[j*nq+p]*100))-atan(-5.*fabs(qd[j*nq+p]*100)))/3.14159265
//                              );
            // using saturation function
            
            t  += (M16iB[j]->G)*((M16iB[j]->Jm)*(M16iB[j]->G)*qdd[j*nq+p]+
                    (M16iB[j]->B)*(M16iB[j]->G)*qd[j*nq+p]+                    
                    ((qd[j*nq+p]>0 ? (M16iB[j]->Tc->x) : 0.0)+
                     (qd[j*nq+p]<0 ? (M16iB[j]->Tc->y) : 0.0)) *
                    ((fabs(qd[j*nq+p]*SG[j]))<1 ? (fabs(qd[j*nq+p]*SG[j])) : 1.0)
                    );
                    
            tau[j*nq+p]=t;
        }
    }
}

static void rot_mat (Link *l, Rot *R, Vect *r, double th, double d){
    // R for link rotation, r for link translation
    double      st, ct, sa, ca;
    st = sin(th);
    ct = cos(th);
    sa = sin(l->alpha);
    ca = cos(l->alpha);
    
    R->n.x=ct;      R->o.x=-ca*st;      R->a.x=sa*st;
    R->n.y=st;      R->o.y=ca*ct;       R->a.y=-sa*ct;
    R->n.z=0.0;     R->o.z=sa;          R->a.z=ca;
    
    r->x=l->A;
    r->y=d*sa;
    r->z=d*ca;
    
}


//finvsim.c====================
// Written by Wenjie Chen, 06/10/2016

void	vect_assign (Vect *a, Vect *b)
{
    a->x = b->x;
    a->y = b->y;
    a->z = b->z;
}

void	vect2_assign (Vect2 *a, Vect2 *b)
{
    a->x = b->x;
    a->y = b->y;
}

void	double9_assign (double *a, double *b)
{
    int i;
    
    for (i = 0; i < 9; i++) {
        a[i] = b[i];
    }
}

void	link_assign (Link *a, Link *b)
{
//  *************** kinematic parameters *********************
    a->alpha = b->alpha;
    a->A = b->A;
    a->D = b->D;
    a->theta = b->theta;
    a->offset = b->offset;
    
//  ***************** dynamic parameters *********************
	/**************** of links ********************************/
    a->rbar = b->rbar;
    a->m = b->m;
    a->I = b->I;    
	/**************** of actuators *****************************/
    a->Jm = b->Jm;
    a->G = b->G;
    a->B = b->B;
    a->Tc = b->Tc;    
}


//vmathsim.c====================
void
vect_cross (Vect *r, Vect *a, Vect *b)
{
	r->x = a->y*b->z - a->z*b->y;
	r->y = a->z*b->x - a->x*b->z;
	r->z = a->x*b->y - a->y*b->x;
}

/* vector cross product for b=[0 0 b] */
/*
void
vect_cross_qdv (Vect *r, Vect *a, Vect *b)
{
	r->x = a->y*b->z;
	r->y = - a->x*b->z;
	r->z = 0;
}*/

/**
 * Vector cross product.
 *
 * @param a Vector.
 * @param b Vector.
 * @return Dot (inner) product.
 */
void
vect_dot (double *r, Vect *a, Vect *b)
{
	*r = a->x * b->x + a->y * b->y + a->z * b->z;
}

/**
 * Vector sum.
 *
 * @param r Return sum vector.
 * @param a Vector.
 * @param b Vector.
 *
 * @note Elementwise addition of two vectors.
 */
void
vect_add (Vect *r, Vect *a, Vect *b)
{
	r->x = a->x + b->x;
	r->y = a->y + b->y;
	r->z = a->z + b->z;
}
/* vect add if b=[0,0,b] */
/*
void
vect_add_qdv (Vect *r, Vect *a, Vect *b)
{
	r->x = a->x;
	r->y = a->y;
	r->z = a->z + b->z;
}*/

/**
 * Vector scalar product.
 *
 * @param r Return scaled vector.
 * @param a Vector.
 * @param s Scalar.
 *
 * @note Elementwise scaling of vector.
 */
void
scal_mult (Vect *r, Vect *a, double s)
{
	r->x = s*a->x;
	r->y = s*a->y;
	r->z = s*a->z;
}

/**
 * Matrix vector product.
 *
 * @param r Return rotated vector.
 * @param m 3x3 rotation matrix.
 * @param v Vector.
 */
void
rot_vect_mult (Vect *r, Rot *m, Vect *v)
{
	r->x = m->n.x*v->x + m->o.x*v->y + m->a.x*v->z;
	r->y = m->n.y*v->x + m->o.y*v->y + m->a.y*v->z;
	r->z = m->n.z*v->x + m->o.z*v->y + m->a.z*v->z;
}

/**
 * Matrix transpose vector product.
 *
 * @param r Return rotated vector.
 * @param m 3x3 rotation matrix.
 * @param v Vector.
 *
 * @note Multiplies \p v by transpose of \p m.
 */
void
rot_trans_vect_mult (Vect *r, Rot *m, Vect *v)
{
	r->x = m->n.x*v->x + m->n.y*v->y + m->n.z*v->z;
	r->y = m->o.x*v->x + m->o.y*v->y + m->o.z*v->z;
	r->z = m->a.x*v->x + m->a.y*v->y + m->a.z*v->z;
}


/**
 * General matrix vector product.
 *
 * @param r Return vector.
 * @param m 3x3 matrix.
 * @param v Vector.
 *
 * @note Assumes matrix is organized in column major order.
 */
void
mat_vect_mult (Vect *r, double *m, Vect *v)
{
	r->x = m[0]*v->x + m[3]*v->y + m[6]*v->z;
	r->y = m[1]*v->x + m[4]*v->y + m[7]*v->z;
	r->z = m[2]*v->x + m[5]*v->y + m[8]*v->z;
}

