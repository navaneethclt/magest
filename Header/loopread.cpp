

#include"loopread.h"
#include "Iir.h" //filter design
#include <iostream>
#include <algorithm>


#include <stdlib.h>
#include <stdio.h>






loopread::loopread()
{

    Serial6.begin(115200);
    std::cout << "Connected" << std::endl;
    
   




}





void loopread::start()
{
    std::thread thread(&loopread::mainloop, this);
    thread.detach();
}


std::vector<double> loopread::getParsedVariables() {
   std::lock_guard<std::mutex> lock(mutex_lock_);
    
    return this->parsedVariables;
}

void loopread::mainloop()
{
    std::string line;
    std::vector<int> vec;
    while (true) {



        if (Serial6.available())
        {
            status_ = 1;

            char data = Serial6.read();
            // Check for newline character
            if (data == '\n')
            {
                // Process the received line
              // std::cout << "Received line: " << line << std::endl;

                // Reset the line buffer
                this->mutex_lock_.lock();

                try {

                    std::istringstream ss(line.c_str());



                    // Parse the input using comma as the delimiter
                    char delimiter;
                    ss >> variable1 >> delimiter >> variable2 >> delimiter >> variable3 >> delimiter >> variable4 >> delimiter>>variable5 >> delimiter >> variable6 >> delimiter >> variable7;
                    this->parsedVariables = { variable1, variable2, variable3, variable4, variable5, variable6, variable7 };
                    


                }
                catch (std::exception& err)
                {
                    //do nothing
                }
                line.clear();
                this->mutex_lock_.unlock();


            }
            else
            {
                // Append character to the line
               // std::cout << "Received line: " << line << std::endl;

                line += data;

            }
        }
    }
}
