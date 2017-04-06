//
//  Agent.hpp
//  ME-791_SF_Project_Delta_project
//
//  Created by Scott S Forer on 4/6/17.
//  Copyright © 2017 Scott S Forer. All rights reserved.
//

#ifndef Agent_hpp
#define Agent_hpp

#include <stdio.h>
using namespace std;

class Agent
{
    friend class EA;
    friend class Simulator;
    friend class Parameters;
    friend class Policy;
    
public:
    vector<Policy> pol;
};

#endif /* Agent_hpp */
