/**
   EN.601.463/663
   Assignment #1 

   Cartesian trajectory generation
   
 */
#include "assignment1.hpp"
#include <math.h>

// Compute the forward kinematics (position and orientation)
// input: the joints angles
// output: the 4x4 homogeneous transformation
void ForwardKinematics( double q1, double q2, double q3, double q4, double q5, double q6, double E[4][4] ){
  
  for(int r=0; r<4; r++)
    for(int c=0; c<4; c++)
      E[r][c] = 0.0;
    
  // Fill the values of the forward kinematics (homogeneous matrix E)
  E[0][0] = -cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q2+q3+q4)*cos(q1)*sin(q6);
  E[0][1] = sin(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q2+q3+q4)*cos(q1)*cos(q6);
  E[0][2] = -cos(q5)*sin(q1)+cos(q2+q3+q4)*cos(q1)*sin(q5);
  E[0][3] = sin(q1)*(-1.333E-1)+cos(q1)*cos(q2)*(1.7E+1/4.0E+1)-cos(q5)*sin(q1)*2.746E-1-sin(q2+q3+q4)*cos(q1)*9.97E-2+cos(q2+q3)*cos(q1)*3.922E-1+cos(q2+q3+q4)*cos(q1)*sin(q5)*2.746E-1;
  E[1][0] = cos(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))+sin(q2+q3+q4)*sin(q1)*sin(q6);
  E[1][1] = -sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))+sin(q2+q3+q4)*cos(q6)*sin(q1);
  E[1][2] = cos(q1)*cos(q5)+cos(q2+q3+q4)*sin(q1)*sin(q5);
  E[1][3] = cos(q1)*1.333E-1+cos(q1)*cos(q5)*2.746E-1+cos(q2)*sin(q1)*(1.7E+1/4.0E+1)-sin(q2+q3+q4)*sin(q1)*9.97E-2+cos(q2+q3)*sin(q1)*3.922E-1+cos(q2+q3+q4)*sin(q1)*sin(q5)*2.746E-1;
  E[2][0] = cos(q2+q3+q4)*sin(q6)+sin(q2+q3+q4)*cos(q5)*cos(q6);
  E[2][1] = cos(q2+q3+q4)*cos(q6)-sin(q2+q3+q4)*cos(q5)*sin(q6);
  E[2][2] = -sin(q2+q3+q4)*sin(q5);
  E[2][3] = cos(q2+q3+q4)*(-9.97E-2)-sin(q2+q3)*3.922E-1-sin(q2)*(1.7E+1/4.0E+1)-sin(q2+q3+q4)*sin(q5)*2.746E-1+1.3E+1/8.0E+1;
  E[3][3] = 1.0;

}

// Compute the inverse of the forward kinematics (position and orientation)
// input: the joints angles
// output: the 4x4 homogeneous transformation
void ForwardKinematicsInverse( double q1, double q2, double q3, double q4, double q5, double q6, double E[4][4] ){

  for(int r=0; r<4; r++)
    for(int c=0; c<4; c++)
      E[r][c] = 0.0;

  // Fill the values of the inverse of the forward kinematics (homogeneous matrix E)
  E[0][0] = sin(q2+q3+q4)*cos(q1)*sin(q6)-cos(q6)*sin(q1)*sin(q5)-cos(q2+q3+q4)*cos(q1)*cos(q5)*cos(q6);
  E[0][1] = sin(q2+q3+q4)*sin(q1)*sin(q6)+cos(q1)*cos(q6)*sin(q5)-cos(q2+q3+q4)*cos(q5)*cos(q6)*sin(q1);
  E[0][2] = cos(q2+q3+q4)*sin(q6)+sin(q2+q3+q4)*cos(q5)*cos(q6);
  E[0][3] = sin(q6)*9.97E-2-cos(q6)*sin(q5)*1.333E-1-sin(q4)*sin(q6)*3.922E-1+cos(q4)*cos(q5)*cos(q6)*3.922E-1-cos(q3)*sin(q4)*sin(q6)*(1.7E+1/4.0E+1)-cos(q4)*sin(q3)*sin(q6)*(1.7E+1/4.0E+1)+cos(q3)*cos(q4)*cos(q5)*cos(q6)*(1.7E+1/4.0E+1)-cos(q2)*cos(q3)*cos(q4)*sin(q6)*(1.3E+1/8.0E+1)-cos(q5)*cos(q6)*sin(q3)*sin(q4)*(1.7E+1/4.0E+1)+cos(q2)*sin(q3)*sin(q4)*sin(q6)*(1.3E+1/8.0E+1)+cos(q3)*sin(q2)*sin(q4)*sin(q6)*(1.3E+1/8.0E+1)+cos(q4)*sin(q2)*sin(q3)*sin(q6)*(1.3E+1/8.0E+1)-cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4)*(1.3E+1/8.0E+1)-cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3)*(1.3E+1/8.0E+1)-cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2)*(1.3E+1/8.0E+1)+cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4)*(1.3E+1/8.0E+1);
  E[1][0] = sin(q1)*sin(q5)*sin(q6)+sin(q2+q3+q4)*cos(q1)*cos(q6)+cos(q2+q3+q4)*cos(q1)*cos(q5)*sin(q6);
  E[1][1] = sin(q2+q3+q4)*cos(q6)*sin(q1)-cos(q1)*sin(q5)*sin(q6)+cos(q2+q3+q4)*cos(q5)*sin(q1)*sin(q6);
  E[1][2] = cos(q2+q3+q4)*cos(q6)-sin(q2+q3+q4)*cos(q5)*sin(q6);
  E[1][3] = cos(q6)*9.97E-2-cos(q6)*sin(q4)*3.922E-1+sin(q5)*sin(q6)*1.333E-1-cos(q3)*cos(q6)*sin(q4)*(1.7E+1/4.0E+1)-cos(q4)*cos(q6)*sin(q3)*(1.7E+1/4.0E+1)-cos(q4)*cos(q5)*sin(q6)*3.922E-1-cos(q2)*cos(q3)*cos(q4)*cos(q6)*(1.3E+1/8.0E+1)-cos(q3)*cos(q4)*cos(q5)*sin(q6)*(1.7E+1/4.0E+1)+cos(q2)*cos(q6)*sin(q3)*sin(q4)*(1.3E+1/8.0E+1)+cos(q3)*cos(q6)*sin(q2)*sin(q4)*(1.3E+1/8.0E+1)+cos(q4)*cos(q6)*sin(q2)*sin(q3)*(1.3E+1/8.0E+1)+cos(q5)*sin(q3)*sin(q4)*sin(q6)*(1.7E+1/4.0E+1)+cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6)*(1.3E+1/8.0E+1)+cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6)*(1.3E+1/8.0E+1)+cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6)*(1.3E+1/8.0E+1)-cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6)*(1.3E+1/8.0E+1);
  E[2][0] = -cos(q5)*sin(q1)+cos(q2+q3+q4)*cos(q1)*sin(q5);
  E[2][1] = cos(q1)*cos(q5)+cos(q2+q3+q4)*sin(q1)*sin(q5);
  E[2][2] = -sin(q2+q3+q4)*sin(q5);
  E[2][3] = cos(q5)*(-1.333E-1)-cos(q4)*sin(q5)*3.922E-1+sin(q3)*sin(q4)*sin(q5)*(1.7E+1/4.0E+1)-cos(q3)*cos(q4)*sin(q5)*(1.7E+1/4.0E+1)+cos(q2)*cos(q3)*sin(q4)*sin(q5)*(1.3E+1/8.0E+1)+cos(q2)*cos(q4)*sin(q3)*sin(q5)*(1.3E+1/8.0E+1)+cos(q3)*cos(q4)*sin(q2)*sin(q5)*(1.3E+1/8.0E+1)-sin(q2)*sin(q3)*sin(q4)*sin(q5)*(1.3E+1/8.0E+1)-2.746E-1;
  E[3][3] = 1.0;
}

// Compute the Adjoint transformation inverse matrix
// input E: the rotation/translation between the base and the hand frame
//          (as computed by the forward kinematics)
// output Ad: the 6x6 adjoint transformation inverse matrix
void AdjointTransformationInverse( double E[4][4], double Ad[6][6] ){

  for(int r=0; r<6; r++)
    for(int c=0; c<6; c++)
      Ad[r][c] = 0.0;  
  // Compute the Adjoint Transformation Inverse A^-1
  // Ad[0][0] = sin(q2+q3+q4)*cos(q1)*sin(q6)-cos(q6)*sin(q1)*sin(q5)-cos(q2+q3+q4)*cos(q1)*cos(q5)*cos(q6);
  // Ad[0][1] = sin(q2+q3+q4)*sin(q1)*sin(q6)+cos(q1)*cos(q6)*sin(q5)-cos(q2+q3+q4)*cos(q5)*cos(q6)*sin(q1);
  // Ad[0][2] = cos(q2+q3+q4)*sin(q6)+sin(q2+q3+q4)*cos(q5)*cos(q6);
  // Ad[0][3] = sin(q1)*sin(q5)*sin(q6)*2.746E-1-cos(q1)*cos(q6)*sin(q5)*(1.3E+1/8.0E+1)-cos(q5)*cos(q6)*sin(q1)*9.97E-2+cos(q4)*sin(q1)*sin(q6)*3.922E-1+cos(q1)*cos(q6)*sin(q2)*sin(q5)*(1.7E+1/4.0E+1)+cos(q3)*cos(q4)*sin(q1)*sin(q6)*(1.7E+1/4.0E+1)+cos(q5)*cos(q6)*sin(q1)*sin(q4)*3.922E-1-sin(q1)*sin(q3)*sin(q4)*sin(q6)*(1.7E+1/4.0E+1)+cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q6)*1.333E-1+cos(q1)*cos(q2)*cos(q3)*cos(q6)*sin(q4)*2.746E-1+cos(q1)*cos(q2)*cos(q4)*cos(q6)*sin(q3)*2.746E-1+cos(q1)*cos(q3)*cos(q4)*cos(q6)*sin(q2)*2.746E-1+cos(q1)*cos(q2)*cos(q6)*sin(q3)*sin(q5)*3.922E-1+cos(q1)*cos(q3)*cos(q6)*sin(q2)*sin(q5)*3.922E-1+cos(q3)*cos(q5)*cos(q6)*sin(q1)*sin(q4)*(1.7E+1/4.0E+1)+cos(q4)*cos(q5)*cos(q6)*sin(q1)*sin(q3)*(1.7E+1/4.0E+1)-cos(q1)*cos(q2)*sin(q3)*sin(q4)*sin(q6)*1.333E-1-cos(q1)*cos(q3)*sin(q2)*sin(q4)*sin(q6)*1.333E-1-cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q6)*1.333E-1-cos(q1)*cos(q6)*sin(q2)*sin(q3)*sin(q4)*2.746E-1-cos(q2)*cos(q3)*sin(q1)*sin(q4)*sin(q6)*(1.3E+1/8.0E+1)-cos(q2)*cos(q4)*sin(q1)*sin(q3)*sin(q6)*(1.3E+1/8.0E+1)-cos(q3)*cos(q4)*sin(q1)*sin(q2)*sin(q6)*(1.3E+1/8.0E+1)+sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q6)*(1.3E+1/8.0E+1)+cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6)*2.746E-1+cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q5)*9.97E-2+cos(q1)*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4)*1.333E-1+cos(q1)*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3)*1.333E-1+cos(q1)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2)*1.333E-1+cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q1)*(1.3E+1/8.0E+1)-cos(q1)*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6)*2.746E-1-cos(q1)*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q5)*9.97E-2-cos(q1)*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6)*2.746E-1-cos(q1)*cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q5)*9.97E-2-cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6)*2.746E-1-cos(q1)*cos(q4)*cos(q6)*sin(q2)*sin(q3)*sin(q5)*9.97E-2-cos(q1)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4)*1.333E-1-cos(q2)*cos(q5)*cos(q6)*sin(q1)*sin(q3)*sin(q4)*(1.3E+1/8.0E+1)-cos(q3)*cos(q5)*cos(q6)*sin(q1)*sin(q2)*sin(q4)*(1.3E+1/8.0E+1)-cos(q4)*cos(q5)*cos(q6)*sin(q1)*sin(q2)*sin(q3)*(1.3E+1/8.0E+1);
  // Ad[0][4] = cos(q1)*cos(q5)*cos(q6)*9.97E-2-cos(q1)*cos(q4)*sin(q6)*3.922E-1-cos(q1)*sin(q5)*sin(q6)*2.746E-1-cos(q6)*sin(q1)*sin(q5)*(1.3E+1/8.0E+1)-cos(q1)*cos(q3)*cos(q4)*sin(q6)*(1.7E+1/4.0E+1)-cos(q1)*cos(q5)*cos(q6)*sin(q4)*3.922E-1+cos(q1)*sin(q3)*sin(q4)*sin(q6)*(1.7E+1/4.0E+1)+cos(q6)*sin(q1)*sin(q2)*sin(q5)*(1.7E+1/4.0E+1)-cos(q1)*cos(q3)*cos(q5)*cos(q6)*sin(q4)*(1.7E+1/4.0E+1)-cos(q1)*cos(q4)*cos(q5)*cos(q6)*sin(q3)*(1.7E+1/4.0E+1)+cos(q1)*cos(q2)*cos(q3)*sin(q4)*sin(q6)*(1.3E+1/8.0E+1)+cos(q1)*cos(q2)*cos(q4)*sin(q3)*sin(q6)*(1.3E+1/8.0E+1)+cos(q1)*cos(q3)*cos(q4)*sin(q2)*sin(q6)*(1.3E+1/8.0E+1)+cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q6)*1.333E-1+cos(q2)*cos(q3)*cos(q6)*sin(q1)*sin(q4)*2.746E-1+cos(q2)*cos(q4)*cos(q6)*sin(q1)*sin(q3)*2.746E-1+cos(q3)*cos(q4)*cos(q6)*sin(q1)*sin(q2)*2.746E-1+cos(q2)*cos(q6)*sin(q1)*sin(q3)*sin(q5)*3.922E-1+cos(q3)*cos(q6)*sin(q1)*sin(q2)*sin(q5)*3.922E-1-cos(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q6)*(1.3E+1/8.0E+1)-cos(q2)*sin(q1)*sin(q3)*sin(q4)*sin(q6)*1.333E-1-cos(q3)*sin(q1)*sin(q2)*sin(q4)*sin(q6)*1.333E-1-cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q6)*1.333E-1-cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q4)*2.746E-1-cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*(1.3E+1/8.0E+1)+cos(q1)*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4)*(1.3E+1/8.0E+1)+cos(q1)*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4)*(1.3E+1/8.0E+1)+cos(q1)*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*(1.3E+1/8.0E+1)+cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1)*sin(q6)*2.746E-1+cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q1)*sin(q5)*9.97E-2+cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q1)*sin(q4)*1.333E-1+cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q1)*sin(q3)*1.333E-1+cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q1)*sin(q2)*1.333E-1-cos(q2)*cos(q5)*sin(q1)*sin(q3)*sin(q4)*sin(q6)*2.746E-1-cos(q2)*cos(q6)*sin(q1)*sin(q3)*sin(q4)*sin(q5)*9.97E-2-cos(q3)*cos(q5)*sin(q1)*sin(q2)*sin(q4)*sin(q6)*2.746E-1-cos(q3)*cos(q6)*sin(q1)*sin(q2)*sin(q4)*sin(q5)*9.97E-2-cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)*sin(q6)*2.746E-1-cos(q4)*cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q5)*9.97E-2-cos(q5)*cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q4)*1.333E-1;
  // Ad[0][5] = sin(q2+q5+q6)*(1.7E+1/1.6E+2)-sin(q2+q3-q5-q6)*9.805E-2-cos(q2+q3+q4+q5-q6)*1.04E-2+cos(q2+q3+q4-q5+q6)*7.705E-2+sin(q2+q5-q6)*(1.7E+1/1.6E+2)-sin(q2-q5+q6)*(1.7E+1/1.6E+2)+sin(-q2+q5+q6)*(1.7E+1/1.6E+2)+cos(q2+q3+q4+q6)*2.0395E-1+sin(q2+q3+q5+q6)*9.805E-2-cos(q2+q3+q4-q5-q6)*6.025E-2+cos(q2+q3+q4-q6)*7.065E-2+sin(q2+q3+q5-q6)*9.805E-2-sin(q2+q3-q5+q6)*9.805E-2+cos(q2+q3+q4+q5+q6)*1.269E-1;
  // Ad[1][0] = sin(q1)*sin(q5)*sin(q6)+sin(q2+q3+q4)*cos(q1)*cos(q6)+cos(q2+q3+q4)*cos(q1)*cos(q5)*sin(q6);
  // Ad[1][1] = sin(q2+q3+q4)*cos(q6)*sin(q1)-cos(q1)*sin(q5)*sin(q6)+cos(q2+q3+q4)*cos(q5)*sin(q1)*sin(q6);
  // Ad[1][2] = cos(q2+q3+q4)*cos(q6)-sin(q2+q3+q4)*cos(q5)*sin(q6);
  // Ad[1][3] = cos(q4)*cos(q6)*sin(q1)*3.922E-1+cos(q1)*sin(q5)*sin(q6)*(1.3E+1/8.0E+1)+cos(q5)*sin(q1)*sin(q6)*9.97E-2+cos(q6)*sin(q1)*sin(q5)*2.746E-1+cos(q3)*cos(q4)*cos(q6)*sin(q1)*(1.7E+1/4.0E+1)-cos(q1)*sin(q2)*sin(q5)*sin(q6)*(1.7E+1/4.0E+1)-cos(q6)*sin(q1)*sin(q3)*sin(q4)*(1.7E+1/4.0E+1)-cos(q5)*sin(q1)*sin(q4)*sin(q6)*3.922E-1+cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q6)*1.333E-1-cos(q1)*cos(q2)*cos(q3)*sin(q4)*sin(q6)*2.746E-1-cos(q1)*cos(q2)*cos(q4)*sin(q3)*sin(q6)*2.746E-1-cos(q1)*cos(q2)*cos(q6)*sin(q3)*sin(q4)*1.333E-1-cos(q1)*cos(q3)*cos(q4)*sin(q2)*sin(q6)*2.746E-1-cos(q1)*cos(q3)*cos(q6)*sin(q2)*sin(q4)*1.333E-1-cos(q1)*cos(q4)*cos(q6)*sin(q2)*sin(q3)*1.333E-1-cos(q2)*cos(q3)*cos(q6)*sin(q1)*sin(q4)*(1.3E+1/8.0E+1)-cos(q2)*cos(q4)*cos(q6)*sin(q1)*sin(q3)*(1.3E+1/8.0E+1)-cos(q3)*cos(q4)*cos(q6)*sin(q1)*sin(q2)*(1.3E+1/8.0E+1)-cos(q1)*cos(q2)*sin(q3)*sin(q5)*sin(q6)*3.922E-1-cos(q1)*cos(q3)*sin(q2)*sin(q5)*sin(q6)*3.922E-1-cos(q3)*cos(q5)*sin(q1)*sin(q4)*sin(q6)*(1.7E+1/4.0E+1)-cos(q4)*cos(q5)*sin(q1)*sin(q3)*sin(q6)*(1.7E+1/4.0E+1)+cos(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q6)*2.746E-1+cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q4)*(1.3E+1/8.0E+1)+cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*2.746E-1-cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5)*sin(q6)*9.97E-2-cos(q1)*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6)*1.333E-1-cos(q1)*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6)*1.333E-1-cos(q1)*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4)*2.746E-1-cos(q1)*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6)*1.333E-1-cos(q1)*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4)*2.746E-1-cos(q1)*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*2.746E-1-cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1)*sin(q6)*(1.3E+1/8.0E+1)+cos(q1)*cos(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6)*9.97E-2+cos(q1)*cos(q3)*sin(q2)*sin(q4)*sin(q5)*sin(q6)*9.97E-2+cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5)*sin(q6)*9.97E-2+cos(q1)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6)*1.333E-1+cos(q2)*cos(q5)*sin(q1)*sin(q3)*sin(q4)*sin(q6)*(1.3E+1/8.0E+1)+cos(q3)*cos(q5)*sin(q1)*sin(q2)*sin(q4)*sin(q6)*(1.3E+1/8.0E+1)+cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)*sin(q6)*(1.3E+1/8.0E+1);
  // Ad[1][4] = sin(q1)*sin(q5)*sin(q6)*(1.3E+1/8.0E+1)-cos(q1)*cos(q4)*cos(q6)*3.922E-1-cos(q1)*cos(q5)*sin(q6)*9.97E-2-cos(q1)*cos(q6)*sin(q5)*2.746E-1-cos(q1)*cos(q3)*cos(q4)*cos(q6)*(1.7E+1/4.0E+1)+cos(q1)*cos(q6)*sin(q3)*sin(q4)*(1.7E+1/4.0E+1)+cos(q1)*cos(q5)*sin(q4)*sin(q6)*3.922E-1-sin(q1)*sin(q2)*sin(q5)*sin(q6)*(1.7E+1/4.0E+1)+cos(q1)*cos(q2)*cos(q3)*cos(q6)*sin(q4)*(1.3E+1/8.0E+1)+cos(q1)*cos(q2)*cos(q4)*cos(q6)*sin(q3)*(1.3E+1/8.0E+1)+cos(q1)*cos(q3)*cos(q4)*cos(q6)*sin(q2)*(1.3E+1/8.0E+1)+cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q1)*1.333E-1+cos(q1)*cos(q3)*cos(q5)*sin(q4)*sin(q6)*(1.7E+1/4.0E+1)+cos(q1)*cos(q4)*cos(q5)*sin(q3)*sin(q6)*(1.7E+1/4.0E+1)-cos(q1)*cos(q6)*sin(q2)*sin(q3)*sin(q4)*(1.3E+1/8.0E+1)-cos(q2)*cos(q3)*sin(q1)*sin(q4)*sin(q6)*2.746E-1-cos(q2)*cos(q4)*sin(q1)*sin(q3)*sin(q6)*2.746E-1-cos(q2)*cos(q6)*sin(q1)*sin(q3)*sin(q4)*1.333E-1-cos(q3)*cos(q4)*sin(q1)*sin(q2)*sin(q6)*2.746E-1-cos(q3)*cos(q6)*sin(q1)*sin(q2)*sin(q4)*1.333E-1-cos(q4)*cos(q6)*sin(q1)*sin(q2)*sin(q3)*1.333E-1-cos(q2)*sin(q1)*sin(q3)*sin(q5)*sin(q6)*3.922E-1-cos(q3)*sin(q1)*sin(q2)*sin(q5)*sin(q6)*3.922E-1+sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q6)*2.746E-1+cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6)*(1.3E+1/8.0E+1)+cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q1)*2.746E-1-cos(q1)*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6)*(1.3E+1/8.0E+1)-cos(q1)*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6)*(1.3E+1/8.0E+1)-cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6)*(1.3E+1/8.0E+1)-cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5)*sin(q6)*9.97E-2-cos(q2)*cos(q3)*cos(q5)*sin(q1)*sin(q4)*sin(q6)*1.333E-1-cos(q2)*cos(q4)*cos(q5)*sin(q1)*sin(q3)*sin(q6)*1.333E-1-cos(q2)*cos(q5)*cos(q6)*sin(q1)*sin(q3)*sin(q4)*2.746E-1-cos(q3)*cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q6)*1.333E-1-cos(q3)*cos(q5)*cos(q6)*sin(q1)*sin(q2)*sin(q4)*2.746E-1-cos(q4)*cos(q5)*cos(q6)*sin(q1)*sin(q2)*sin(q3)*2.746E-1+cos(q2)*sin(q1)*sin(q3)*sin(q4)*sin(q5)*sin(q6)*9.97E-2+cos(q3)*sin(q1)*sin(q2)*sin(q4)*sin(q5)*sin(q6)*9.97E-2+cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5)*sin(q6)*9.97E-2+cos(q5)*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q6)*1.333E-1;
  // Ad[1][5] = sin(q2+q3+q4+q5+q6)*(-1.269E-1)+cos(q2+q5+q6)*(1.7E+1/1.6E+2)+cos(q2+q3-q5-q6)*9.805E-2-sin(q2+q3+q4+q5-q6)*1.04E-2-sin(q2+q3+q4-q5+q6)*7.705E-2-cos(q2+q5-q6)*(1.7E+1/1.6E+2)-cos(q2-q5+q6)*(1.7E+1/1.6E+2)+cos(-q2+q5+q6)*(1.7E+1/1.6E+2)+cos(q2+q3+q5+q6)*9.805E-2-sin(q2+q3+q4+q6)*2.0395E-1-sin(q2+q3+q4-q5-q6)*6.025E-2-cos(q2+q3+q5-q6)*9.805E-2-cos(q2+q3-q5+q6)*9.805E-2+sin(q2+q3+q4-q6)*7.065E-2;
  // Ad[2][0] = -cos(q5)*sin(q1)+cos(q2+q3+q4)*cos(q1)*sin(q5);
  // Ad[2][1] = cos(q1)*cos(q5)+cos(q2+q3+q4)*sin(q1)*sin(q5);
  // Ad[2][2] = -sin(q2+q3+q4)*sin(q5);
  // Ad[2][3] = cos(q1)*cos(q5)*(-1.3E+1/8.0E+1)+sin(q1)*sin(q5)*9.97E-2-sin(q1)*sin(q4)*sin(q5)*3.922E-1+cos(q1)*cos(q5)*sin(q2)*(1.7E+1/4.0E+1)+cos(q1)*cos(q2)*cos(q5)*sin(q3)*3.922E-1+cos(q1)*cos(q3)*cos(q5)*sin(q2)*3.922E-1-cos(q3)*sin(q1)*sin(q4)*sin(q5)*(1.7E+1/4.0E+1)-cos(q4)*sin(q1)*sin(q3)*sin(q5)*(1.7E+1/4.0E+1)+cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*9.97E-2-cos(q1)*cos(q2)*cos(q3)*sin(q4)*sin(q5)*1.333E-1-cos(q1)*cos(q2)*cos(q4)*sin(q3)*sin(q5)*1.333E-1-cos(q1)*cos(q2)*cos(q5)*sin(q3)*sin(q4)*9.97E-2-cos(q1)*cos(q3)*cos(q4)*sin(q2)*sin(q5)*1.333E-1-cos(q1)*cos(q3)*cos(q5)*sin(q2)*sin(q4)*9.97E-2-cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3)*9.97E-2-cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5)*(1.3E+1/8.0E+1)+cos(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*1.333E-1+cos(q2)*sin(q1)*sin(q3)*sin(q4)*sin(q5)*(1.3E+1/8.0E+1)+cos(q3)*sin(q1)*sin(q2)*sin(q4)*sin(q5)*(1.3E+1/8.0E+1)+cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5)*(1.3E+1/8.0E+1);
  // Ad[2][4] = cos(q1)*sin(q5)*(-9.97E-2)-cos(q5)*sin(q1)*(1.3E+1/8.0E+1)+cos(q5)*sin(q1)*sin(q2)*(1.7E+1/4.0E+1)+cos(q1)*sin(q4)*sin(q5)*3.922E-1+cos(q2)*cos(q5)*sin(q1)*sin(q3)*3.922E-1+cos(q3)*cos(q5)*sin(q1)*sin(q2)*3.922E-1+cos(q1)*cos(q3)*sin(q4)*sin(q5)*(1.7E+1/4.0E+1)+cos(q1)*cos(q4)*sin(q3)*sin(q5)*(1.7E+1/4.0E+1)+cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5)*(1.3E+1/8.0E+1)+cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1)*9.97E-2-cos(q1)*cos(q2)*sin(q3)*sin(q4)*sin(q5)*(1.3E+1/8.0E+1)-cos(q1)*cos(q3)*sin(q2)*sin(q4)*sin(q5)*(1.3E+1/8.0E+1)-cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5)*(1.3E+1/8.0E+1)-cos(q2)*cos(q3)*sin(q1)*sin(q4)*sin(q5)*1.333E-1-cos(q2)*cos(q4)*sin(q1)*sin(q3)*sin(q5)*1.333E-1-cos(q2)*cos(q5)*sin(q1)*sin(q3)*sin(q4)*9.97E-2-cos(q3)*cos(q4)*sin(q1)*sin(q2)*sin(q5)*1.333E-1-cos(q3)*cos(q5)*sin(q1)*sin(q2)*sin(q4)*9.97E-2-cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)*9.97E-2+sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*1.333E-1;
  // Ad[2][5] = cos(q2-q5)*(1.7E+1/8.0E+1)+cos(q2+q3+q5)*1.961E-1+cos(q2+q3-q5)*1.961E-1-sin(q2+q3+q4+q5)*1.165E-1+cos(q2+q5)*(1.7E+1/8.0E+1)+sin(q2+q3+q4-q5)*1.68E-2;
  // Ad[3][3] = sin(q2+q3+q4)*cos(q1)*sin(q6)-cos(q6)*sin(q1)*sin(q5)-cos(q2+q3+q4)*cos(q1)*cos(q5)*cos(q6);
  // Ad[3][4] = sin(q2+q3+q4)*sin(q1)*sin(q6)+cos(q1)*cos(q6)*sin(q5)-cos(q2+q3+q4)*cos(q5)*cos(q6)*sin(q1);
  // Ad[3][5] = cos(q2+q3+q4)*sin(q6)+sin(q2+q3+q4)*cos(q5)*cos(q6);
  // Ad[4][3] = sin(q1)*sin(q5)*sin(q6)+sin(q2+q3+q4)*cos(q1)*cos(q6)+cos(q2+q3+q4)*cos(q1)*cos(q5)*sin(q6);
  // Ad[4][4] = sin(q2+q3+q4)*cos(q6)*sin(q1)-cos(q1)*sin(q5)*sin(q6)+cos(q2+q3+q4)*cos(q5)*sin(q1)*sin(q6);
  // Ad[4][5] = cos(q2+q3+q4)*cos(q6)-sin(q2+q3+q4)*cos(q5)*sin(q6);
  // Ad[5][3] = -cos(q5)*sin(q1)+cos(q2+q3+q4)*cos(q1)*sin(q5);
  // Ad[5][4] = cos(q1)*cos(q5)+cos(q2+q3+q4)*sin(q1)*sin(q5);
  // Ad[5][5] = -sin(q2+q3+q4)*sin(q5);
  
  Ad[0][0] = (E[1][1]*E[2][2]-E[1][2]*E[2][1])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[0][1] = -(E[0][1]*E[2][2]-E[0][2]*E[2][1])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[0][2] = (E[0][1]*E[1][2]-E[0][2]*E[1][1])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[0][3] = (E[0][1]*E[1][2]*E[1][3]-E[0][2]*E[1][1]*E[1][3]+E[0][1]*E[2][2]*E[2][3]-E[0][2]*E[2][1]*E[2][3])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[0][4] = -(E[0][1]*E[0][3]*E[1][2]-E[0][2]*E[0][3]*E[1][1]-E[1][1]*E[2][2]*E[2][3]+E[1][2]*E[2][1]*E[2][3])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[0][5] = -(E[0][1]*E[0][3]*E[2][2]-E[0][2]*E[0][3]*E[2][1]+E[1][1]*E[1][3]*E[2][2]-E[1][2]*E[1][3]*E[2][1])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[1][0] = -(E[1][0]*E[2][2]-E[1][2]*E[2][0])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[1][1] = (E[0][0]*E[2][2]-E[0][2]*E[2][0])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[1][2] = -(E[0][0]*E[1][2]-E[0][2]*E[1][0])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[1][3] = -(E[0][0]*E[1][2]*E[1][3]-E[0][2]*E[1][0]*E[1][3]+E[0][0]*E[2][2]*E[2][3]-E[0][2]*E[2][0]*E[2][3])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[1][4] = (E[0][0]*E[0][3]*E[1][2]-E[0][2]*E[0][3]*E[1][0]-E[1][0]*E[2][2]*E[2][3]+E[1][2]*E[2][0]*E[2][3])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[1][5] = (E[0][0]*E[0][3]*E[2][2]-E[0][2]*E[0][3]*E[2][0]+E[1][0]*E[1][3]*E[2][2]-E[1][2]*E[1][3]*E[2][0])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[2][0] = (E[1][0]*E[2][1]-E[1][1]*E[2][0])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[2][1] = -(E[0][0]*E[2][1]-E[0][1]*E[2][0])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[2][2] = (E[0][0]*E[1][1]-E[0][1]*E[1][0])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[2][3] = (E[0][0]*E[1][1]*E[1][3]-E[0][1]*E[1][0]*E[1][3]+E[0][0]*E[2][1]*E[2][3]-E[0][1]*E[2][0]*E[2][3])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[2][4] = -(E[0][0]*E[0][3]*E[1][1]-E[0][1]*E[0][3]*E[1][0]-E[1][0]*E[2][1]*E[2][3]+E[1][1]*E[2][0]*E[2][3])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[2][5] = -(E[0][0]*E[0][3]*E[2][1]-E[0][1]*E[0][3]*E[2][0]+E[1][0]*E[1][3]*E[2][1]-E[1][1]*E[1][3]*E[2][0])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[3][3] = (E[1][1]*E[2][2]-E[1][2]*E[2][1])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[3][4] = -(E[0][1]*E[2][2]-E[0][2]*E[2][1])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[3][5] = (E[0][1]*E[1][2]-E[0][2]*E[1][1])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[4][3] = -(E[1][0]*E[2][2]-E[1][2]*E[2][0])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[4][4] = (E[0][0]*E[2][2]-E[0][2]*E[2][0])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[4][5] = -(E[0][0]*E[1][2]-E[0][2]*E[1][0])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[5][3] = (E[1][0]*E[2][1]-E[1][1]*E[2][0])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[5][4] = -(E[0][0]*E[2][1]-E[0][1]*E[2][0])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);
  Ad[5][5] = (E[0][0]*E[1][1]-E[0][1]*E[1][0])/(E[0][0]*E[1][1]*E[2][2]-E[0][0]*E[1][2]*E[2][1]-E[0][1]*E[1][0]*E[2][2]+E[0][1]*E[1][2]*E[2][0]+E[0][2]*E[1][0]*E[2][1]-E[0][2]*E[1][1]*E[2][0]);

}

// Compute and return the Jacobian of the robot given the current joint positions
// input: the joints angles
// output: the 6x6 Jacobian
void Jacobian( double q1, double q2, double q3, double q4, double q5, double q6, double J[6][6] ){
  
  for(int r=0; r<6; r++)
    for(int c=0; c<6; c++)
      J[r][c] = 0.0;

  // Fill the values of the Jacobian matrix J
  J[0][1] = cos(q1)*(-1.3E+1/8.0E+1);
  J[0][2] = (cos(q1)*(sin(q2)*3.4E+1-1.3E+1))/8.0E+1;
  J[0][3] = (cos(q1)*(sin(q2+q3)*3.922E+3+sin(q2)*4.25E+3-1.625E+3))/1.0E+4;
  J[0][4] = sin(q1-q4)*(-1.961E-1)-sin(q1+q3+q4)*(1.7E+1/8.0E+1)+sin(-q1+q3+q4)*(1.7E+1/8.0E+1)-cos(q1+q2+q3+q4)*1.479E-1-sin(q1+q4)*1.961E-1+cos(-q1+q2+q3+q4)*1.46E-2;
  J[0][5] = cos(q1)*cos(q5)*(-1.3E+1/8.0E+1)+sin(q1)*sin(q5)*9.97E-2-sin(q1)*sin(q4)*sin(q5)*3.922E-1+cos(q1)*cos(q5)*sin(q2)*(1.7E+1/4.0E+1)+cos(q1)*cos(q2)*cos(q5)*sin(q3)*3.922E-1+cos(q1)*cos(q3)*cos(q5)*sin(q2)*3.922E-1-cos(q3)*sin(q1)*sin(q4)*sin(q5)*(1.7E+1/4.0E+1)-cos(q4)*sin(q1)*sin(q3)*sin(q5)*(1.7E+1/4.0E+1)+cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*9.97E-2-cos(q1)*cos(q2)*cos(q3)*sin(q4)*sin(q5)*1.333E-1-cos(q1)*cos(q2)*cos(q4)*sin(q3)*sin(q5)*1.333E-1-cos(q1)*cos(q2)*cos(q5)*sin(q3)*sin(q4)*9.97E-2-cos(q1)*cos(q3)*cos(q4)*sin(q2)*sin(q5)*1.333E-1-cos(q1)*cos(q3)*cos(q5)*sin(q2)*sin(q4)*9.97E-2-cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3)*9.97E-2-cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5)*(1.3E+1/8.0E+1)+cos(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*1.333E-1+cos(q2)*sin(q1)*sin(q3)*sin(q4)*sin(q5)*(1.3E+1/8.0E+1)+cos(q3)*sin(q1)*sin(q2)*sin(q4)*sin(q5)*(1.3E+1/8.0E+1)+cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5)*(1.3E+1/8.0E+1);
  J[1][1] = sin(q1)*(-1.3E+1/8.0E+1);
  J[1][2] = (sin(q1)*(sin(q2)*3.4E+1-1.3E+1))/8.0E+1;
  J[1][3] = (sin(q1)*(sin(q2+q3)*3.922E+3+sin(q2)*4.25E+3-1.625E+3))/1.0E+4;
  J[1][4] = cos(q1-q4)*1.961E-1+cos(q1+q3+q4)*(1.7E+1/8.0E+1)+cos(-q1+q3+q4)*(1.7E+1/8.0E+1)-sin(q1+q2+q3+q4)*1.479E-1+cos(q1+q4)*1.961E-1-sin(-q1+q2+q3+q4)*1.46E-2;
  J[1][5] = cos(q1)*sin(q5)*(-9.97E-2)-cos(q5)*sin(q1)*(1.3E+1/8.0E+1)+cos(q5)*sin(q1)*sin(q2)*(1.7E+1/4.0E+1)+cos(q1)*sin(q4)*sin(q5)*3.922E-1+cos(q2)*cos(q5)*sin(q1)*sin(q3)*3.922E-1+cos(q3)*cos(q5)*sin(q1)*sin(q2)*3.922E-1+cos(q1)*cos(q3)*sin(q4)*sin(q5)*(1.7E+1/4.0E+1)+cos(q1)*cos(q4)*sin(q3)*sin(q5)*(1.7E+1/4.0E+1)+cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5)*(1.3E+1/8.0E+1)+cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1)*9.97E-2-cos(q1)*cos(q2)*sin(q3)*sin(q4)*sin(q5)*(1.3E+1/8.0E+1)-cos(q1)*cos(q3)*sin(q2)*sin(q4)*sin(q5)*(1.3E+1/8.0E+1)-cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5)*(1.3E+1/8.0E+1)-cos(q2)*cos(q3)*sin(q1)*sin(q4)*sin(q5)*1.333E-1-cos(q2)*cos(q4)*sin(q1)*sin(q3)*sin(q5)*1.333E-1-cos(q2)*cos(q5)*sin(q1)*sin(q3)*sin(q4)*9.97E-2-cos(q3)*cos(q4)*sin(q1)*sin(q2)*sin(q5)*1.333E-1-cos(q3)*cos(q5)*sin(q1)*sin(q2)*sin(q4)*9.97E-2-cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)*9.97E-2+sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*1.333E-1;
  J[2][2] = cos(q2)*(1.7E+1/4.0E+1);
  J[2][3] = cos(q2+q3)*3.922E-1+cos(q2)*(1.7E+1/4.0E+1);
  J[2][4] = sin(q2+q3+q4)*1.333E-1;
  J[2][5] = cos(q2-q5)*(1.7E+1/8.0E+1)+cos(q2+q3+q5)*1.961E-1+cos(q2+q3-q5)*1.961E-1-sin(q2+q3+q4+q5)*1.165E-1+cos(q2+q5)*(1.7E+1/8.0E+1)+sin(q2+q3+q4-q5)*1.68E-2;
  J[3][1] = -sin(q1);
  J[3][2] = -sin(q1);
  J[3][3] = -sin(q1);
  J[3][4] = -sin(q2+q3+q4)*cos(q1);
  J[3][5] = -cos(q5)*sin(q1)+cos(q2+q3+q4)*cos(q1)*sin(q5);
  J[4][1] = cos(q1);
  J[4][2] = cos(q1);
  J[4][3] = cos(q1);
  J[4][4] = -sin(q2+q3+q4)*sin(q1);
  J[4][5] = cos(q1)*cos(q5)+cos(q2+q3+q4)*sin(q1)*sin(q5);
  J[5][0] = pow(cos(q5)*sin(q1)-cos(q2+q3+q4)*cos(q1)*sin(q5),2.0)+pow(-sin(q2+q3+q4)*cos(q1)*sin(q6)+cos(q6)*sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5)*cos(q6),2.0)+pow(sin(q1)*sin(q5)*sin(q6)+sin(q2+q3+q4)*cos(q1)*cos(q6)+cos(q2+q3+q4)*cos(q1)*cos(q5)*sin(q6),2.0);
  J[5][4] = (cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))*(cos(q5)*sin(q1)-cos(q2+q3+q4)*cos(q1)*sin(q5))-cos(q6)*(cos(q1)*cos(q5)+cos(q2+q3+q4)*sin(q1)*sin(q5))*(-sin(q2+q3+q4)*cos(q1)*sin(q6)+cos(q6)*sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5)*cos(q6))-sin(q6)*(cos(q1)*cos(q5)+cos(q2+q3+q4)*sin(q1)*sin(q5))*(sin(q1)*sin(q5)*sin(q6)+sin(q2+q3+q4)*cos(q1)*cos(q6)+cos(q2+q3+q4)*cos(q1)*cos(q5)*sin(q6));
  J[5][5] = -sin(q2+q3+q4)*sin(q5);
}

