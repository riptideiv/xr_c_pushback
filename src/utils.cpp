#include "utils.hpp"

int sign(double val) {
    if (val > 0)
        return 1;
    if (val < 0)
        return -1;
    return 0;
}