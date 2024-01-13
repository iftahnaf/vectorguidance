#include "../include/common.hpp"
#include <chrono>
using namespace std;
using namespace std::chrono;

int main(){
    auto start = high_resolution_clock::now();
    find_minimum_positive_real_root();
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    std::cout << "using eigen, measured time is: "<< duration.count() << endl;

    start = high_resolution_clock::now();
    explicit_find_minimum_positive_real_root();
    stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stop - start);
    std::cout << "using explicit, measured time is: "<< duration.count() << endl;
}