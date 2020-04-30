/*作业描述
    给定一个点P=(2,1), 将该点绕原点先逆时针旋转45◦，再平移(1,2), 计算出换后点的坐标（要求用齐次坐标进行计算）。
*/


#include<cmath>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<iostream>

int main(){


    Eigen::Vector3f v(2.0f,1.0f,1.0f);
    Eigen::Matrix3f m;
    m << std::cos(45.0), -std::sin(45.0), 1.0, std::sin(45.0), std::cos(45.0), 2.0, 0.0, 0.0, 1.0;
    // matrix output
    v = m*v;

    std::cout << "new coordinate: \n" << std::endl;
    std::cout << "(" << v(0) << ", " << v(1) << ")"<< std::endl;

    return 0;
}
