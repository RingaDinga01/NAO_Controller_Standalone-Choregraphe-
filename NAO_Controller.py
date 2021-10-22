#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

import qi
import argparse
import sys
import almath
import math
import numpy as np
import time

def main(session):
    # Get the services ALMotion & ALRobotPosture.
    useSensorValues = False
    motion_service  = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture")
    initRobotPosition = almath.Pose2D(motion_service.getRobotPosition(useSensorValues))
    motion_service.stopMove()

    #Initalize the arms for better walk stability   
    leftArmEnable  = True
    rightArmEnable = True
    motion_service.setMoveArmsEnabled(leftArmEnable, rightArmEnable)

    # Go to rest position
    #motion_service.rest()
    # Wake up robot
    motion_service.wakeUp()

    # Send robot to Stand Init
    posture_service.goToPosture("StandInit", 0.5)

    # Initialize the move
    motion_service.moveInit()

 

    t_start = time.time()#Start the clock
    t_end = float('inf')#float('inf')##We want to do do this forever so set = inf
    t = 0 ##set current time to 0 before loop begins

    f =  0.01#frequency of walk
    r = 0.75 #radius of turn
    w = 2*math.pi*f
    Kp = 0.30 #Position Gain
    Kthy = 0.50 #Turn Rate Gain

    # Create some text files to write data
    NaoPos = open('NaoPosition.txt', 'w')
    PartPos = open('ParticlePosition.txt', 'w')
    TimeFile = open('Time.txt', 'w')
    SpeedFile = open('Speed.txt', 'w')
    time.sleep(1)
    #Main Control Loop
    while(t<t_end):
        #Start Timer
        t=time.time()-t_start
  
    
        #Define NAOs current position Global NAO (x up y left) Simulation Purposes
        RobotPosition = almath.Pose2D(motion_service.getRobotPosition(useSensorValues))
        RobotGlobalThy = math.atan2(RobotPosition.y,RobotPosition.x) #NAO Thy Relative to his Global Axis
        NaoGlobalPos= np.matrix([[RobotPosition.x],[RobotPosition.y]]) #NAO Position Relative to his Global Axis

        #Define Desired Global Position (x right y up) 
        posx =1#r*np.cos(w*t) #Current desired x global position (m)
        posy =1#r*np.sin(w*t)#r*np.sin(2*w*t)/2 #Current desired y globalposition (m)

        #Transform these Global Coordinates (x right y up) into GlobalNAO (x up y left) by rotating around z axis clockwise relative to NAO global thy
        coordMatrixClock = np.matrix([[np.cos(math.pi/2), np.sin(math.pi/2)], [-np.sin(math.pi/2), np.cos(math.pi/2)]])
        PosGlobal= np.matrix([[posx],[posy]])
        PosNaoGlobal = np.matmul(coordMatrixClock,PosGlobal)
        PosNaoGlobalx =PosNaoGlobal[0,0]
        PosNaoGlobaly =PosNaoGlobal[1,0]

        #Transform the GlobalNAO Position Axis (x up y left) into GLOBAL by making Axis (x right y up), rotaion around z axis counter-clockwise relative to NAO global thy
        coordMatrixCounterClock = np.matrix([[np.cos(math.pi/2), -np.sin(math.pi/2)], [np.sin(math.pi/2), np.cos(math.pi/2)]])
        NaoGlobal = np.matmul(coordMatrixCounterClock,NaoGlobalPos)
        NaoGlobalx =NaoGlobal[0,0]
        NaoGlobaly =NaoGlobal[1,0]

        #Writing Data to Files
        NaoPos.write("%s\n" % np.column_stack((RobotPosition.x,RobotPosition.y)))
        #Write particle pos to file
        PartPos.write("%s\n" % np.column_stack((posx,posy)))
        #Write Time Vector to file
        TimeFile.write("%s\n" % np.column_stack((t,t)))
        #print particle, "particle",math.atan2(posy,posx)*(180/np.pi)
        
        #Setting Global Velocity 
        xGdesDeriv = 0#-r*w*np.sin(w*t) #Derivative of desired x global (m/s)
        yGdesDeriv = 0#r*w*np.cos(w*t)#+2*r*w*np.cos(2*w*t)/2 #Derivative of desired y global (m/s)

        #Get NAOs current Local heading from the Sim (Use VICON GLOBAL Heading with Vicon)
        cTheta = RobotPosition.theta #Grab NAOs Heading
        
        #Create matrix with Global difference in position (x right y up)
        PosErrorMat2 = np.matrix([[posx-NaoGlobalx],[posy-NaoGlobaly]])

        #Transform Global Difference (x right y up) to NAO Global (x up y left)
        PosErrorMatNao = np.matmul(coordMatrixClock,PosErrorMat2)

        #Create the desired velocity vector in Global (x right y up)
        VdesGlobal = np.matrix([[xGdesDeriv],[yGdesDeriv]])

        #Create the desired velocity vector in Global NAO (x up y left)
        VdesGlobalNAO = np.matmul(coordMatrixClock,VdesGlobal)

        #Create Vc command Velocity in NAO Global (x up y left)
        Vcc = VdesGlobalNAO+Kp*(PosErrorMatNao)
        Vccx = Vcc[0,0]
        Vccy = Vcc[1,0]

        #Compute the Desired Global Thy
        ThyDes=(math.atan2(Vccy,Vccx))

        #Compute Desired turn angle error
        thyAng3 = ThyDes-cTheta
 
        #Deal with the heading flip around the unit circle
        if thyAng3>np.pi:
            finalHeading = thyAng3-(2*np.pi)
        elif thyAng3<-np.pi:
            finalHeading = thyAng3+(2*np.pi)
        else:
            finalHeading = thyAng3
                   
        #Compute Desired turn rate
        turnRate = Kthy*finalHeading

        #Compute Desired Forward Speed
        vcxsquare = np.square(Vccx)
        vcysquare = np.square(Vccy)
        Sdes = math.sqrt(vcxsquare+vcysquare)
        #If sDes (Forward Speed) is greater than 0.08m/s then set it to 0.08m/s as this is approximately NAOs fastest forward movement speed (10.5cms/s actually)
        if Sdes>0.08:
            Sdes=0.08
        
        #Check if turn rate is greater than 0.53rad/s in both directions (NAO Max Turn Rate)
        if turnRate>0.5:
            turnRate=0.5
        if turnRate<-0.5:
            turnRate=-0.5
        SpeedFile.write("%s\n" % np.column_stack((Sdes,t)))
        motion_service.move(Sdes,0,turnRate)
        #print motion_service.getMoveConfig("Max")
        time.sleep(0.2)
     


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")##127.0.0.1 or 192.168.1.22
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    main(session)

