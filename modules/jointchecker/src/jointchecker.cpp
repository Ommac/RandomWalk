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

#include <yarp/os/all.h>
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
    
    Network yarp;

    RpcServer inPort;            // Create a port.
    inPort.open("/jointchecker/in");     // Give it a name on the network.
    Bottle in, out;    // Make places to store things.    
    
    // some useful variables
    Vector q,jmin,jmax,pFoot,pKnee;

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
    for (unsigned int i=0; i<chain->getDOF(); ++i)
    {    
        jmin(i) = CTRL_RAD2DEG*(*chain)(i).getMin() + 5.0;
        jmax(i) = CTRL_RAD2DEG*(*chain)(i).getMax() - 5.0;
    }
    
    cout << jmin.toString() << endl;
    cout << jmax.toString() << endl;
    
    while(true)
    {
        inPort.read(q,true); // Read from the port, warn that we'll be replying.
        printf("Got %s\n", q.toString().c_str());   
        
        // retrieve the end-effector position.
        // Translational part is in meters.
        pFoot = chain->EndEffPosition(q);
        cout << "Current leg end-effector position: " << pFoot.toString().c_str() << endl;
        
        bool check = true;
        
        //Check the y component
        if (pFoot(1) > -0.15)
        {
            cout << "Cartesian space limits not respected" << endl;
            check = false;
        }
        
        // ------------------------------
        //   Perform joint limits check
        // ------------------------------
        
//         // get initial joints configuration
//         for (int i = 0 ; i<q.length() && check == true; ++i)
//         {
//             if ((q(i) > jmax(i)) || (q(i) < jmin(i)))
//                 check = false;
//         }
//         
//         // Extra limitation on  joint 1
//         if ((q(1) < (jmin(1) + 10.0)))
//                 check = false;
    
        if  (!check)
        {
            cout << "Joint space limits not respected" << endl;
            
            out.clear();
            out.addInt(0);
            inPort.reply(out);    // send reply.
        }
        else
        {
            out.clear();
            out.addInt(1);
            inPort.reply(out);    // send reply.
        }
    }
    return 0;
}


