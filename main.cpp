/**
 *
 * A custom module for collision avoidance of the leg
 *
 * \author Raffaello Camoriano
 * 
 * CopyPolicy: Released under the terms of GPL 2.0 or later
 */ 

#include <iostream>
#include <iomanip>

#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>
#include <iCub/iKin/iKinIpOpt.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;

int main()
{
    // some useful variables
    Vector q,jmin,jmax,pFoot,pThigh,pCalf;

    // you can get the same result by creating an iCubArm object;
    // iKin already provides internally coded limbs for iCub, such as
    // iCubArm, iCubLeg, iCubEye, ..., along with the proper H0 matrix
    iCubLeg libLeg("left");

    // get a chain on the arm; you can use arm object directly but then some
    // methods will not be available, such as the access to links through
    // [] or () operators. This prevent the user from adding/removing links
    // to iCub limbs as well as changing their properties too easily.
    // Anyway, arm object is affected by modifications on the chain.
    iKinChain *chain;
        chain=libLeg.asChain();

    // dump DOF bounds using () operators.
    // Remind that angles are expressed in radians
    jmin.resize(chain->getDOF());
    jmax.resize(chain->getDOF());
    for (unsigned int i=0; i<chain->getDOF(); i++)
    {    
        jmin(i) = (*chain)(i).getMin() + CTRL_RAD2DEG*10;
        jmax(i) = (*chain)(i).getMax() - CTRL_RAD2DEG*10;
    }

    // there are three links for the torso which do not belong to the
    // DOF set since they are blocked. User can access them through [] operators
    cout << "Torso blocked links at:" << endl;
    for (unsigned int i=0; i<chain->getN()-chain->getDOF(); i++)
        cout << CTRL_RAD2DEG*(*chain)[i].getAng() << " ";
    cout << endl;

    
    while(true)
    {
        // retrieve the end-effector position.
        // Translational part is in meters.
        pFoot = chain->EndEffPosition();
        cout << "Current leg end-effector position: " << pFoot.toString().c_str() << endl;
        
        // Retrieve current thigh position
        pThigh = chain->Position();
        cout << "Current thigh position: " << pThigh.toString().c_str() << endl;
            
        // Retrieve current calf position
        pCalf = chain->Position();
        cout << "Current calf position: " << pCalf.toString().c_str() << endl;
        
        // ------------------------------
        // Perform cartesian limits check
        // ------------------------------
        
        //Check the y component
        if (pFoot(1) > -0.1 || pThigh(1) > -0.1 || pCalf(1) > -0.1) // Set to 10 cm
        {
            cout << "Cartesian space limits not respected" << endl;
            return false;
        }
        
        // ------------------------------
        //   Perform joint limits check
        // ------------------------------
        
        // get initial joints configuration
        q = chain->getAng();
        bool check = true;
        for (int i = 0 ; i<q.length() && check == true; ++i)
        {
            if (q(i) > jmax(i) || q(i) > jmin(i))
                check = false;
        }
        
        if  (!check)
        {
            cout << "Joint space limits not respected" << endl;
            return false;
        }
        
    }
    return 0;
}


