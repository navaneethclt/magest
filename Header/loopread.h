#include <chrono>
#include <thread>
#include "pch.h"
#include "ArduSerial.h"
#include <iostream>
#pragma once

#include <vector>
#include <mutex>
class loopread
{
public:
    loopread();
    void start();
    std::vector<double> getParsedVariables();  // Added function to retrieve parsedVariables



private:
    void mainloop();

    std::mutex mutex_lock_;
    int status_=0;
    std::vector<double> parsedVariables = { 0.0, 0.0, 0.0, 0.0 ,0.0, 0.0, 0.0};  // Initialize parsedVariables
    double variable1, variable2, variable3, variable4, variable5, variable6, variable7;


}; 

