#ifndef UTILS_H_
#define UTILS_H_
#include <array>
#include <cmath>

std::array<double, 3> get_xyz(const std::array<double, 16>& O_T_EE)
{
    std::array<double, 3> xyz = {O_T_EE[12], O_T_EE[13], O_T_EE[14]};
    return xyz;
}

void xyz_2_OTEE(std::array<double, 3> xyz, std::array<double, 16>& O_T_EE)
{
    O_T_EE[12] = xyz[0];
    O_T_EE[13] = xyz[1];
    O_T_EE[14] = xyz[2];
}

double dist(double delta_x, double delta_y)
{
    return std::sqrt(delta_x * delta_x + delta_y * delta_y);
}


#endif