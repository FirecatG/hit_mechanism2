#include "cam.hpp"
#include <iostream>
int main() {
    Cam cam(60, 125, 30, 125, 65, 60, 50, 30, 10, 10, moveType::SIN_ACCELERATION, moveType::COS_ACCELERATION);
    auto result = cam.calculate();

    for (double s:result.s_points) {
        std::cout << s << std::endl;
    }

    return 0;
}