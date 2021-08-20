  
#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

#define PI 3.14159

int main(){
    std::cout <<"------Homework 0-------"<<std::endl;
    //Homework 0
    Eigen::Vector3f origin(2,1,1);
    Eigen::Vector3f point(0,0,0);
    Eigen::Matrix3f transform;
    transform << cos(45/180.*PI),-cos(45)/180.*PI,1,
                -cos(45/180.*PI), cos(45/180.*PI),4,
                0,        0      ,1;
    point = transform * origin;
    std::cout<<point<<std::endl;

    return 0;
}