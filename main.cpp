//
//  main.cpp
//  ME-791_SF_Project_Delta_project
//
//  Created by Scott S Forer on 4/6/17.
//  Copyright © 2017 Scott S Forer. All rights reserved.
//

#include <iostream>
#include <cstdlib>
#include <vector>
#include <cmath>
#include <time.h>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <cassert>
#include <ctime>
#include <random>

#include "Parameters.hpp"
#include "Policy.hpp"
#include "Agent.hpp"
#include "Neural_Network.h"
#include "EA.hpp"
#include "Simulator.hpp"


using namespace std;


int main()
{
    srand(time(NULL));
    Parameters P;
    EA E;
    E.pP = &P;
    E.Run_Project_Delta();
    return 0;
}
