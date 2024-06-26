#ifndef FBL_KINEMATICS_H
#define FBL_KINEMATICS_H


#include <cmath>

float l_1 = 8.0, l_4 = 8.0, l_2 = 20.0, l_3 = 20.0, l_5 = 18.0;

float x_A = 0, y_A = 0, x_E = l_5, y_E = 0;

float phi_1, phi_2, phi_3, phi_4;

class AIM_C{
public:
    float x_C;
    float y_C;
    float p_1;
    float p_4;
};

// forward position
// the calculation of position in forward kinematics
// in:  phi_1与phi_4角度
// out: C点的期望位置
void cal_fk_p(AIM_C & aim){
    phi_1 = aim.p_1;
    phi_4 = aim.p_4;

    float x_B = l_1*cos(phi_1) + x_A;
    float y_B = l_1*sin(phi_1) + y_A;
    float x_D = l_4*cos(phi_4)+x_E;
    float y_D = l_4*sin(phi_4)+y_E;
        
    float A_0 = 2*l_2*(x_D-x_B);
    float B_0 = 2*l_2*(y_D-y_B);
    float C_0 = l_2*l_2 + (x_D-x_B)*(x_D-x_B) + (y_D-y_B)*(y_D-y_B) - l_3*l_3;

    phi_2 = 2*atan2((B_0+sqrt(A_0*A_0 + B_0*B_0 -C_0*C_0)), (A_0 + C_0));
    aim.x_C = x_B + l_2*cos(phi_2);
    aim.y_C = y_B + l_2*sin(phi_2);

}


// inverse position
// the calculation of position in forward kinematics
// in:  C点的期望位置
// out: phi_1与phi_4角度
void cal_ik_p(AIM_C & aim){
    float l_1 = 8.0;
    float l_4 = 8.0;
    float l_2 = 20.0;
    float l_3 = 20.0;
    float l_5 = 18.0;   //cm

    float x_A = 0;
    float y_A = 0;
    float x_E = l_5;
    float y_E = 0;

    float x = aim.x_C;
    float y = aim.y_C;

    float A_1 = (x - x_A)*(x - x_A) + (y - y_A)*(y - y_A) + l_1*l_1 - l_2*l_2;
    float B_1 = -2*(x - x_A)*l_1;
    float C_1 = -2*(y - y_A)*l_1;
    aim.p_1 = 2*atan2((-C_1+sqrt(C_1*C_1 + B_1*B_1 - A_1*A_1)), (A_1 - B_1));

    float A_2 = (x - x_E)*(x - x_E) + (y - y_E)*(y - y_E) + l_4*l_4 - l_3*l_3;
    float B_2 = -2*(x - x_E)*l_4;
    float C_2 = -2*(y - y_E)*l_4;
    aim.p_4 = 2*atan2((-C_2+sqrt(C_2*C_2 + B_2*B_2 - A_2*A_2)), (A_2 - B_2));

}
 

void test(AIM_C &a){
    // float radius = 2;
    static float theta = 25;
    a.x_C = (x_A + x_E)/2;
    a.y_C = theta;
    if(theta<12) theta = 25;
    theta -= 0.001;

}


void test1(AIM_C &a){
    static float k = 4;
    a.x_C = k;
    a.y_C = 17;
    if(k > 20) k = 4;
    k += 0.002;

}

void test2(AIM_C &a){
    float radius = 5;
    static float theta = 0; 


    a.x_C = radius * cos(theta) +(x_A + x_E)/2;
    a.y_C = radius * sin(theta) +18;
    theta += 0.002;
    if (theta > 2*PI) theta = 0;
}

#endif
