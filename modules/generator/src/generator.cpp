// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <iostream>


#include <stdio.h>
#include <stdlib.h>
#include <yarp/os/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>

#include <string>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace std;

int sign(int valore){
if(valore>=0){return 1;}
else{return -1;}
}

int main(int argc, char *argv[]) 
{
    Network yarp;
    
    RpcClient p;            // Create a port.
    p.open("/generator/out");    // Give it a name on the network.
    Vector out;
    Bottle in;   // Make places to store things.

    
    Property params;
    params.fromCommand(argc, argv);

    if (!params.check("robot"))
    {
        fprintf(stderr, "Please specify the name of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub)\n");
        return -1;
    }
    std::string robotName=params.find("robot").asString().c_str();
    std::string remotePorts="/";
    remotePorts+=robotName;
    remotePorts+="/left_leg";

    std::string localPorts="/test/client";

    Property options;
    options.put("device", "remote_controlboard");
    options.put("local", localPorts.c_str());   //local port names
    options.put("remote", remotePorts.c_str());         //where we connect to

    // create a device
    PolyDriver robotDevice(options);
    if (!robotDevice.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }

    IPositionControl *pos;
    IEncoders *encs;

    bool ok;
    ok = robotDevice.view(pos);
    ok = ok && robotDevice.view(encs);

    if (!ok) {
        printf("Problems acquiring interfaces\n");
        return 0;
    }

    int nj=0;
    pos->getAxes(&nj);
    Vector encoders;
    Vector command;
    Vector tmp;
    encoders.resize(nj);
    tmp.resize(nj);
    command.resize(nj);
    
    int i;
    for (i = 0; i < nj; i++) {
         tmp[i] = 100.0;
    }
    pos->setRefAccelerations(tmp.data());

    for (i = 0; i < nj; i++) {
        int vel = 20;
        tmp[i] = vel;
        pos->setRefSpeed(i, tmp[i]);
    }

    //pos->setRefSpeeds(tmp.data()))
    
    //first read all encoders
    //
    printf("waiting for encoders");
    while(!encs->getEncoders(encoders.data()))
    {
        Time::delay(0.1);
        printf(".");
    }
    printf("\n;");

    command=encoders;
    //now set the shoulder to some value
    command[0]= 0.0;
    command[1]= 40.0;
    command[2]= 0.0;
    command[3]= 0.0;
    command[4]= 0.0;
    command[5]= 0.0;
    pos->positionMove(command.data());
    
    bool done=false;

    while(!done)
    {
        pos->checkMotionDone(&done);
        Time::delay(0.1);
    }
    
    double qmin= 0.01;
    double qmax= 1.0;
    double q0=0.0, q1=10.0, q2=0.0, q3=-60.0, q4=0.0, q5=0.0;
    int times=0;
    
    double sign0 = 1.0;
    double sign1 = 1.0;
    double sign2 = 1.0;
    double sign3 = 1.0;
    double sign4 = 1.0;
    double sign5 = 1.0;       
    
    while(true)
    {
        int feasability=0;
        double q0tmp=0.0;
        double q1tmp=0.0;
        double q2tmp=0.0;
        double q3tmp=0.0;
        double q4tmp=0.0;
        double q5tmp=0.0;
        times++;
        
 
        
        while(!feasability)
        {

            
            if(times%100==0)
            {
                sign0 = (double)sign(rand()%2-1);
                sign1 = (double)sign(rand()%2-1);
                sign2 = (double)sign(rand()%2-1);
                sign3 = (double)sign(rand()%2-1);
                sign4 = (double)sign(rand()%2-1);
                sign5 = (double)sign(rand()%2-1);
            }
            cout << "sign0: " << sign0 << endl;
            
            q0tmp=q0+sign0*(qmin+((double)(rand()%100) / 100.0) *(qmax-qmin)) ; //
            q1tmp=q1+sign1*(qmin+((double)(rand()%100) / 100.0) *(qmax-qmin)) ;
            q2tmp=q2+sign2*(qmin+((double)(rand()%100) / 100.0) *(qmax-qmin)) ;
            q3tmp=q3+sign3*(qmin+((double)(rand()%100) / 100.0) *(qmax-qmin)) ;
            q4tmp=q4+sign4*(qmin+((double)(rand()%100) / 100.0) *(qmax-qmin)) ;
            q5tmp=q5+sign5*(qmin+((double)(rand()%100) / 100.0) *(qmax-qmin)) ;
            printf("%.001lf %.001lf %.001lf %.001lf %.001lf %.001lf\n", q0tmp,q1tmp,q2tmp,q3tmp,q4tmp,q5tmp);
            
            if(-30.0<q0tmp && q0tmp<90.0 && 0.0<q1tmp && q1tmp<90.0 && -80.0<q2tmp && q2tmp<78.0 && -125.0<q3tmp && q3tmp<15.0 && -20.0<q4tmp && q4tmp<44.0 && -22.0<q5tmp && q5tmp<44.0 ) //
            {
                cout << "CIAO" << endl;
                out.clear();
                out.push_back(q0tmp);
                out.push_back(q1tmp);
                out.push_back(q2tmp);
                out.push_back(q3tmp);
                out.push_back(q4tmp);
                out.push_back(q5tmp);    
                cout << "CIAO2" << endl;
                p.write(out,in); // send command, wait for reply.
                                cout << "CIAO3" << endl;

                feasability = in.get(0).asInt();
                // process response "in".
            }
            
            if (feasability)
            {
                q0=q0tmp;
                q1=q1tmp;
                q2=q2tmp;
                q3=q3tmp;
                q4=q4tmp;
                q5=q5tmp;
            }
            printf("%d\n", feasability);
        }
        
            command[0]=q0;//-30 90
            command[1]=q1;//0 90
            command[2]=q2;//-80 78
            command[3]=q3;//-125 15
            command[4]=q4;//-20 44
            command[5]=q5;//-22 22
            // To obtain random positions not linked to each other
            /* times++;
            red_factor = .2;
            if (times%2)
            {
                command[0]=rand()% 120*red_factor -30*red_factor ;//-30 90
                command[1]=rand()% 90*red_factor +0;//0 90
                command[2]=rand()% 158*red_factor -78*red_factor;//-80 78
                command[3]=rand()% 140*red_factor -125*red_factor;//-125 15
                command[4]=rand()% 44*red_factor -20*red_factor;//-20 44
                command[5]=rand()% 44*red_factor -22*red_factor;//-22 22
                

            }
            else
            {
                command[0]=rand()% 120 -30 ;//-30 90
                command[1]=rand()% 90 +0;//0 90
                command[2]=rand()% 158 -78;//-80 78
                command[3]=rand()% 140 -125;//-125 15
                command[4]=rand()% 44 -20;//-20 44
                command[5]=rand()% 44 -22;//-22 22
            }
            */
            pos->positionMove(command.data());

//             int count=50;
//             while(count--)
//                 {
//                     Time::delay(0.005);
//                     bool ret=encs->getEncoders(encoders.data());
//                     
//                     if (!ret)
//                     {
//                         fprintf(stderr, "Error receiving encoders, check connectivity with the robot\n");
//                     }
//                     else
//                     { 
//                         printf("%.1lf %.1lf %.1lf %.1lf\n", encoders[0], encoders[1], encoders[2], encoders[3]);
//                     }
//                 }
    }

    robotDevice.close();
    
    return 0;
}


