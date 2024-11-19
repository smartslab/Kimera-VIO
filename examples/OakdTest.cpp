#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "depthai/depthai.hpp"
#include <cstdio>
#include <stdlib.h>
#include "../kimera-vio/src/dataprovider/OakdSpinner.h"


int main(){
    
    auto spinner = std::make_unique<VIO::OAKdSpinner1>();
    
    while (spinner->spin()) {
      continue;
    }
    
    return 0;
}
    

