#include "../include/common.hpp"
#include <chrono>
using namespace std;
using namespace std::chrono;

int main(){
    double exp_tgo, eigen_tgo;
    Eigen::Matrix<double,5,1> coeff(6.734024999999999, 0, -73.21148415361623, -316.0267504005409, -2368.9809380330944);

    auto start = high_resolution_clock::now();
    eigen_tgo = find_minimum_positive_real_root(coeff);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    std::cout << "using eigen, tgo = " << eigen_tgo << ", measured time is: "<< duration.count() << "[microseconds]" << endl;

    start = high_resolution_clock::now();
    exp_tgo = explicit_find_minimum_positive_real_root(coeff);
    stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stop - start);
    std::cout << "using explicit, tgo = " << exp_tgo << ", measured time is: "<< duration.count() << "[microseconds]" << endl;
    
    return 0;
}