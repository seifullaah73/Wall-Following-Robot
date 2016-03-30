#!/usr/bin/python
"""
Python Interface Extensions for Pioneer and PeopleBot
M L Walters. Feb 2008.  V0.95
M L Walters. June 2009  V2.1
M L Walters. Aug 2009 V2.4
M L Walters. March 2011 V2.8, Changes to work with Aria 2.7.2



This file contains some useful functions for programming and 
interacting with the Pioneer and PeopleBots using the Python wrapper
for the Aria robot API system
"""
# Setup and Global variables
# Configuration, Setup the robot with the various hardware bits attached
# Just insert, change or remove to suit your Pioneer/PeopleBot configuration

# Which base? Either 'Pioneer' or 'PeopleBot'
robotType = "Pioneer"

#Real or sim,ulated robot? Uncomment as appropriate
# Windows may be "COMn" n = serial port number, Linux may be "/dev/ttyUSB0" etc. 
# Use "localhost" to onnect to MobileSim simulated robot
robotPort = "localhost"
#robotPort = "com3"

#Other options:
# Camera/head type: May be 'None','VCC4', 'CMU2+', or 'V50i', ' 
camType= "None" 
camPort = "com4" # Windows may be "COMn" n = serial port number, Linux may be "/dev/ttyUSB0" etc.

# Laser fitted: May be 'None', 'Sick'
laser="None"
laserPort = "" # Windows may be "COMn" n = serial port number, Linux may be "/dev/ttyUSB0" etc.


#Set Private Global variables
LiftHeight=0
tcpSock=0           # Global TCIP socket object name variable
moving = False      # Can Test to see if robot is moving(=True). Note, not set by ArRobot functions
lastMoveError=0     # Test to see if the last move was successful
objectDistance = 0  # Distance to nearest (front or rear) object if motion not completed
robot= 0            # The main Aria robot object
wandering = 0       # True if robot is wandering, set to false to stop wandering behaviour

# First import the required modules

# Standard Python modules with useful functions 
from math import *
#import readline
import sys
import os
import socket
from time import sleep
from thread import start_new_thread
from threading import Timer

# Non-stanard python libraries
if robotPort != "localhost": import serial # PySerial, also reqires win32api for windows systems

# AriaPy requires that the _AriaPy.so shared object or dll library(s) are 
# either in the current working directory or the appropriate python
# lib directory, AriaPy.py in the CWD or sitepackages directory, and
# for windows, Aria.dll is in the CWD, dll or windows/system directory.  
from AriaPy import *


#Initialise the Aria system
a = Aria()
a.init()

robot = ArRobot()
if robotType == "PeopleBot":
    gripper = ArGripper(robot) 
else:
    gripper=0
     
# Camera or head uses the cmu2+ cam, or none
if camType=="VCC4":
    camDev = ArVCC4(robot, 0,0,0)# Supported directly by Aria
elif (camType=="CMU2+") or (camType=="Head"):
    try:
        if os.name == "nt":
            camDev = serial.Serial(camPort, 9600, timeout = 1)
        elif os.name == "posix":
            camDev = serial.Serial(camPort, 9600, timeout = 1)
        else:
            camPort = (raw_input("Camera or head serial port device name? : "))
            camDev = serial.Serial(camPort, 9600, timeout = 1)
        print "Connection made to camera ok"
    except:
        print "Camera not connected"
     
# Add the sonars to the robot system
sonar=ArSonarDevice()
robot.addRangeDevice(sonar)

#Laser configure stuff to go here 
if laser =="Sick":
    laser=ArSick()
    robot.addRangeDevice(laser)
elif laser=="None":
    print "No laser range device connected"

#Which serial port is the robot conected to?
# To real robot
if robotPort == "localhost":
    # Connect to simulator
    # Start simulator?
    conn = ArSimpleConnector(sys.argv)
    if (not conn.connectRobot(robot)):
        print "Could not connect to robot, exiting"
        Aria.exit(1)
else:
    # Connect to real robot
    conn = ArSerialConnection()
    if os.name =="posix":
        conn.setPort(robotPort)
    elif os.name=="nt":
        conn.setPort(robotPort)
    else:
        robotPort = raw_input("Serial Device connected to robot?/n")
        conn.setPort(robotPort)
    print conn.getPort()  
    robot.setDeviceConnection(conn)
    if (robot.blockingConnect() != 1):
        print "Problem connecting to robot. Exiting program"
        Aria.exit(1)
        

    
robot.runAsync(1)
robot.enableMotors()
print "Robot position (", robot.getX(), ",", robot.getY(), ",", robot.getTh(), ")"
#robot.unlock()
#robot.stop()
#cam.init()
sleep(1)
print "Connected OK"

#Behaviour based actions. Create the various behavious that may be required:

# For general obstacle avoidance
acAvoidFrontNear=ArActionAvoidFront("Avoid Front Near",500,0)
acAvoidFrontFar=ArActionAvoidFront("Avoid Front Far", 1000, 200, 5)
acGotoAvoidFront=ArActionAvoidFront("Goto Avoid front",300,100,10)
acAvoidSide=ArActionAvoidSide("Avoid side", 250,2)
acBumpers=ArActionBumpers()
acStallRecover=ArActionStallRecover()
acConstantVelocity=ArActionConstantVelocity()

# General useful behavious
acStop=ArActionStop()
acTurn=ArActionTurn()
acLimiterForwards=ArActionLimiterForwards("General speed limiter",150,500,300)
acLimiterBackwards=ArActionLimiterBackwards()
acLimiterTableSensor=ArActionLimiterTableSensor()

# For move functions using ArActionGoto()
acGotoAvoidSide=ArActionAvoidSide("Goto Avoid Side",150,2)
acAvoidFrontNear=ArActionAvoidFront("Goto Avoid Front",200,200)
goalPose = ArPose()
currentPose = ArPose()
acGoto=ArActionGotoStraight()
goalPose.setPose(0,0,0)
#acGoto.setGoal(goalPose)
#acGoto.activate()

#while not acGoto.haveAchievedGoal():
#    print "Setting up acGoto()"
#    sleep(.1)



#First stop the robot from moving. 
robot.stop()

#Add the actions to the robot behaviour executive
#Note: does not run these behaviours until robot.clearDirectMotion()
# enables robot to move under activated behaviours

#Basic obstacle avoidance
robot.addAction(acStallRecover,100)
robot.addAction(acBumpers,75)
robot.addAction(acAvoidFrontNear,61)  
robot.addAction(acAvoidFrontFar,41)
robot.addAction(acAvoidSide,32)

# Those used for wander/heading behaviour
robot.addAction(acConstantVelocity,25)

#Goto goal position x,y behaviour setup
robot.addAction(acGoto,30)
robot.addAction(acGotoAvoidSide,47)
robot.addAction(acGotoAvoidFront,47)
#Miscellaneous turns under limit condi7ions
robot.addAction(acTurn,42)

#Social and safety speed limiters
robot.addAction(acLimiterForwards,51)
robot.addAction(acLimiterBackwards,61)

#Table top/front sensor. Sets speed to 0
robot.addAction(acLimiterTableSensor,65)

# do not move yet
robot.deactivateActions()
robot.clearDirectMotion()
#robot.clearDirectMotion()


def stop(errmsg="Stopped"):
    """
    Stops the robot.  All physical movements are stopped and cancelled.
    """
    global moving, wandering
    robot.stop()
    robot.deactivateActions()
    robot.clearDirectMotion()
    if robotType=="PeopleBot":
        gripper.gripStop()
        gripper.liftStop()
    moving=False
    wandering=False
    #print"Stopped"
    return errmsg

def wander():
    """
    The robot will wander around, hopefully avoiding all objects and 
    collisions.  You can play with the action priorities in the action
    setup section at the top of this file
    """
    global moving
    moving = True
    robot.clearDirectMotion()
    acStallRecover.activate()
    acBumpers.activate()
    acAvoidFrontNear.activate()
    acAvoidFrontFar.activate
    acAvoidSide.activate()
    acLimiterForwards.activate()
    acConstantVelocity.activate()
    #if robot.isRunning() != 1: robot.run(1)
    #if robot.tryLock()==1: robot.unlock()
    #robot.clearDirectMotion()

def wander2():
    """
    The robot will wander around, hopefully avoiding all objects and 
    collisions. call stop() to stop wandering
    """
    start_new_thread(_wander())
    
    
def _wander():
    """
    The wander primitive, implemented in python, differently than the
    behaviourbased wander found in the ARia action examples.  
    The robot will wander around, hopefully avoiding all objects and 
    collisions.  Under development.
    """
    global wandering
    wandering = True
    while wandering==True:
        _move() # Move until an obstacle is encountered
        # Then turn left ot right approriately
        if nearest(90, 90) > nearest(-90, 90):
            _move(25)
        elif nearest(-90, 90) > mearest(90, 90):
            _move(-25)
        else:
            # unless both sides blocked, then go back
            _move(0, 180)
        moveWait()
        print wandering
      
    
   
        
    

def posXY():
    """
    Returns the current x, y, theta poosition as a tuple
    """
    return  (robot.getX(),  robot.getY(), robot.getTh())

def setXY( x=0,y=0,th=0):
    """
    Sets the robots current position as x,y,theta
    If x, y, or th not supplied sets current position as origin,
    current heading as 0
    """
    currentPose.setPose(x,y,th)
    robot.moveTo(currentPose)

def _move(heading=0, distance=0, guardDistance=700, vel=0, timeout=1.0, guardAngle = 20):
    """
    This is the move primitive. Normally it is called via the various move() and gotoXY()
    functions so it can run as a background thread/task while your program can perform other
    tasks.
    The robot will first turn by heading (degrees), then move forward by distance (mm) at
    vel (mm/sec). If distance and heading both == 0, the robot will move until it encounters an
    obstacle. If guardDistance == 0, then the move will be unguarded and the robot may colide with
    people or objects - use with caution. Consider reducing the guardAngle if you need to
    pass through a door of narrow opening. Note, the Bumper auto stop system will not be
    disabled. If guardAngle <15, unreliable obstacle detection will occur for systems
    fitted only with sonar.

    If guardDistance is negative, than the move will use obstacle avoidance behaviour to attempt
    to reach the given heading and distance. 
    The timout parameter is the minimum time that the function will wait for the robot reach the
    target distance or position. It is automatically increased/adjusted proportionally to longer
    distances speicfied, but for longer obstacle avoidance runs, or _move() until obstacle
    encountered, you may want to increase the default timeout value.
    
    """
    global moving, objectDistance, lastMoveError
    moving = True
    objectDistance=0
    goalDistance = 0
    lastMoveError = 0
    #timeout=0
    # Set the robot velocity for move
    if vel != 0:
        robot.setAbsoluteMaxTransVel(vel)
        robot.setAbsoluteMaxRotVel(vel / 5 )
    else:
        vel = robot.getAbsoluteMaxTransVel()
    # Set up the timer
    if guardDistance<0:
        timeout = timeout + (abs(distance)/100) + (abs(heading))
    else:
        timeout = timeout + 5
    #print "Timeout = ",timeout, vel
    timer=Timer(timeout, stop,("Timeout",))
    timer.start()
    # Change the heading first to point in the right direction
    if heading != 0:
        if heading > 90:
            robot.setDeltaHeading(90)
            heading = heading -90
        elif heading < -90:
            robot.setDeltaHeading(-90)
            heading = heading + 90
        while not robot.isHeadingDone():
            #print robot.isHeadingDone(), "**"
            sleep(0.2)
            pass          
        robot.setDeltaHeading(heading)
        while not robot.isHeadingDone():
            sleep(0.2)
            pass   
    #print "heading done"
    # Then do the move
    if guardDistance < 0:
        # The do obstacle avoidance to reach position
        # This does not work very well at present!
        # Need to rewrite this to use mainly python code
        # as aria behaviours are unreliable.
        guardDistance = abs(guardDistance)
        #print "guardDistance", guardDistance
        # Work out XY positions
        th = robot.getTh()
        thRad = th * pi/180
        dx = distance * cos(thRad)
        dy = distance * sin(thRad)
        x = robot.getX() + dx
        y = robot.getY() + dy
        #print dx, dy, x,
        #print y, th, thRad
        # goto/avoid behaviours here
        goalPose.setPose(x, y, th)
        #print goalPose
        robot.clearDirectMotion()
        if robotType=="PeopleBot":
            cLimiterTableSensor.activate()
        robot.clearDirectMotion()
        acStallRecover.activate()
        acBumpers.activate()
        acAvoidFrontNear.activate()
        acAvoidFrontFar.activate
        acAvoidSide.activate()
        #acLimiterForwards.activate()
        #acGotoAvoidFront.activate()
        #acGotoAvoidSide.activate()
        acGoto.setGoal(goalPose)
        acGoto.activate()
        while (not acGoto.haveAchievedGoal()) or moving:
            # Calculate distance to goal
            distance = (((x - robot.getX())**2 + (y - robot.getY())**2)**0.5)
            if distance < guardDistance:
                print "Near to goal position: ", distance
                break
            #Check if stalled or way off course
            #if robot.isLeftMotorStalled() and robot.isRightMotorStalled():
            #    # Basic damage limtation!
            #    print "Stalled"
            #    break 
            sleep(0.3)
        robot.deactivateActions()
    elif guardDistance >= 0:
        # Then either keep moving until stopped
        if distance==0 and heading==0:
            #set constant velocity
            robot.clearDirectMotion()
            #acConstantVelocity.activate()
            robot.setVel(500)
            #robot.enableMotors();
			#robot.unlock();

        else:
        #Or move the specified distance
            robot.move(distance)
        # Then wait for move to complete
        
        while (not robot.isMoveDone()) or moving==True:
            #print "Moving " ,distance, moving, robot.isMoveDone(), guardDistance, objectDistance
            # Note global flag moving will be set by other functions to abort current
            # move, but only useful if _move runs in own thread, e/g via
            #start_new_thread(). Note, stop() will change moving to False in order to abort
            # current move.
            #print moving
            if guardDistance > 0:
                # Object avoidance
                if distance >= 0:
                    # Check front sonars
                    objectDistance = robot.checkRangeDevicesCurrentPolar(-1 * guardAngle, guardAngle)
                elif distance<0:
                    # Else check rear sonars if reversing
                    objectDistance = robot.checkRangeDevicesCurrentPolar(180-guardAngle, 180+guardAngle)
                #print objectDistance, guardDistance
                if objectDistance < (guardDistance + 400):
                    # Slow down!
                    #print "Slowing down"
                    robot.setAbsoluteMaxTransVel(300)
                if objectDistance < guardDistance:
                    # Stop, avoid collision!
                    #print "Object detected", guardDistance, objectDistance
                    break
            elif robot.isLeftMotorStalled() and robot.isRightMotorStalled():
                # Basic damage limtation!
                #print "Stalled"
                break
            sleep(0.2) # Otherwise upsets Aria if called too often!
    # Finish the move, and reset all signals etc.
    stop()
    timer.cancel()
    robot.setAbsoluteMaxTransVel(vel)
    moving = False
    if objectDistance > guardDistance:
        objectDistance=0
    

def moveWait():
    """
    Waits for current direct move to complete before returning
    the distance to an object in the path (via gloc=bal variable objectDistance),
    or 0 if move completed without interference
    """
    global moving, objectDistance
    while moving == True:
        pass
        sleep(.5)
    return objectDistance

def nearest(direction=0, sweepAngle=20):
    """
    Returns the distance to the nearest object between sweepAngle
    (where sweepAangle = 0 is the relative front/heading angle).Use sweepAngle = 180
    to sweep all round the robot. Note, if only sonar fitted, sweepAngles < 20 return
    unreliable results.
    The default direction = 0 returns objects in front of the
    sides of the robot. Use 180 for objects behind the robot, - 90 for RHS,
    90 for LHS of robot. 
    """
    distance=0
    objectAngle=-1
    sweepAngle=sweepAngle/2
    distance= robot.checkRangeDevicesCurrentPolar(direction-sweepAngle,direction+sweepAngle)
    return distance

def move (heading=0, distance=0, guardDistance=600, timeout = 1.0, vel=0, guardAngle=30 ):
    """ 
    This causes the robot to move an Incremental distance in mm, at a relative heading
    in degrees, at the specified vel(ocity) in mm per second. Note: If guardDistance =0
    the movement is not guarded and collisions can occur. 
    The robot will first turn by heading (degrees), then move forward by distance (mm)
    at vel (mm/sec). Normally, if vel is not specified, the robot will move at it's max
    rtransfer velocity. If distance and heading both == 0, the robot will move forward until
    it encounters an obstacle.
    
    If guardDistance == 0, then the move will be unguarded and the robot may colide with
    people or objects, so use with caution. Rather consider reducing the guardAngle if you need to
    pass through a door of narrow opening. Notes: the Bumper auto stop system will not be
    disabled; If guardAngle <15, unreliable obstacle detection will occur for systems
    fitted only with sonar.

    If guardDistance is negative, than the move will use obstacle avoidance behaviour to attempt
    to reach the given heading and distance. 
    The timout parameter is the minimum time that the function will wait for the robot to reach
    the target distance or position. It is automatically increased/adjusted proportionally for
    longer distances specified, but for longer obstacle avoidance runs, or _move() until obstacle
    encountered, you may want to increase the default timeout value.
    
    The function will return before the move has been completed, so that
    your Python program (or the interpreter) can do other things while the movement is
    executing.  To find out if the move was successful, use the moveWait() and/or
    nearest() functions. If the move is currently executing the global variable
    moving = True and objectDistance == the distance in mm to the nearest object (in
    the direction of travel) or if the robot is/has stopped because an object was in the way. 
    
    The move() function will always wait for the previous move() moves to complete
    before the specified move is started.  If the current move is to be replaced or aborted
    use the stop() function first to clear out any pending moves or behaviour
    _move(heading=0, distance=0, guardDistance=600, vel=0, timeout=1.0, guardAngle = 30)
    """
    moveWait()
    stop()
    start_new_thread(_move, (heading, distance, guardDistance, vel, timeout,guardAngle))
    return "Moving"
                     

def gotoXY(x = 0, y = 0, goalDistance=20, timeout = 2, vel=0, guardAngle=30):
    """
    This function is currently under development! See _move() for details!
    The Robot will turn to face towards the x,y point and then drive  to the specified
    absolute x, y position using obstacle avoidance. If vel is specified and is > 0,
    the velocity will be changed to the new given value in mm/sec, otherwise the
    robot will drive at it's normal max transfer velocity.
    The goalDistance parameter specifies proportionally how close to the x,y position the
    robot needs to get in order for the move to be classed as successful, i.e. the default
    value 20 means the goalDistance need only be within 1/20th of the total distance travelled
    (as the crow flies) towards the goal position.
        
    See moveInc() for details of timeout and guardAngle.
    
    To find out if the move was successful, use the moveWait() and/or
    nearest() functions. If the move is currently executing the global variables
    moving == True and objectDistance == the distance in mm to the nearest object (in
    the direction of travel) or if the robot is/has stopped because an object was in the way.
    The function will return before the move has been completed, so that
    your Python program (or the interpreter) can do other things while the movement is
    executing.  
    
    The moveXY() function will always wait for the previous move to complete before
    the specified move is started.  If the current move is to be replaced or aborted
    use the stop() function to clear out any pending moves.
    """
    moveWait()
    stop()
    # Direct move    
    dx = x - robot.getX()
    dy = y - robot.getY()
    distance = (dx**2 +dy**2)**0.5
    heading = asin(dy/distance) * 180/pi
    goalDistance =  distance/goalDistance
    print heading, distance, 
    _move(heading)
    start_new_thread(_move, (heading, distance,  goalDistance, vel, timeout, guardAngle))
    return "Moving"



def end():
    """
    Stops the robot and then disconnects from the robot
    Shuts down Aria and cleans up as much as possible
    """
    try:
        Aria.exit()
    except:
        print "problem exiting Aria"

def look(panv=0,tiltv=0,zoomv=0,slewv=0):
    """
    Moves and controls the robots cam
    All parameters optional and if called without will drive cam to:
        pan=0,tilt=0,zoom=0
    panv = pan angle (degrees). 0 = centre, +ve = left, -ve = right
    tiltv = tilt angle (degrees). 0 = level, +ve = up (90 max),m -ve = down(-30 min)
    zoomv = zoom in increments of 1/100 from 0 to 30000 (0 to 30x)
    slewv = cam slew rate in degrees per sec
    """
    if camType== "VCC4":
        if slewv !=0:
            cam.panSlew(slewv)
            cam.tiltSlew(slewv)
        cam.pan(panv)
        cam.tilt(tiltv)
        cam.zoom(zoomv)
    elif camType == "CMU2+":
        #CMUCam PT Unit
        if panv == "OFF":
            camDev.write(chr(255)+chr(2)+chr(0))
            camDev.write(chr(255)+chr(3)+chr(0))
            camDev.write(chr(255)+chr(4)+chr(0))
            camDev.write(chr(255)+chr(5)+chr(0))
        elif panv == "ON":
            camDev.write(chr(255)+chr(2)+chr(255))
            camDev.write(chr(255)+chr(3)+chr(255))
            camDev.write(chr(255)+chr(4)+chr(255))
            camDev.write(chr(255)+chr(5)+chr(255))
        else:
            panv = 125 + panv
            #print panv
            camDev.write (chr(255)+chr(0)+chr(panv))
            tiltv = 150 - tiltv
            camDev.write(chr(255)+chr(1)+chr(tiltv))
    elif camType == "Head":
        # Robot Head attached
        #First switch all LEDS off
        camDev.write(chr(255)+chr(0)+chr(0))
        camDev.write(chr(255)+chr(1)+chr(0))
        camDev.write(chr(255)+chr(2)+chr(0))
        camDev.write(chr(255)+chr(3)+chr(0))
        camDev.write(chr(255)+chr(4)+chr(0))
        camDev.write(chr(255)+chr(5)+chr(0))
        camDev.write(chr(255)+chr(6)+chr(0))
        camDev.write(chr(255)+chr(7)+chr(0))
        if panv == "ON":
            panv=0
            tiltv=0
        if panv != "OFF":
             #Then switch on the required LEDS
            if panv < 0 : camDev.write(chr(255)+chr(4)+chr(250))
            if panv > 0 : camDev.write(chr(255)+chr(6)+chr(250))
            #if panv == 0 : camDev.write(chr(255)+chr(3)+chr(250))
            if tiltv < 0: camDev.write(chr(255)+chr(7)+chr(250))
            if tiltv > 0: camDev.write(chr(255)+chr(5)+chr(250))
            if tiltv == 0 and panv == 0: camDev.write(chr(255)+chr(3)+chr(250))
    # mouth LEDS, TBA



def gripper(gripCommand="Stop"):
    """
    Useful function which provides several easy to use commands for operating the
    PeopleBots single DoF lifting gripper. gripCommand may be any of the following:
    strings: "Init", "Park", "Wait", "PickUp" or "PutDown". If gripCommand is an integer,
    then the gripper will move by a distance corresponding to the time while the
    gripper is moving.
    Init - Initialises the gripper and puts it in position to perform a pickUp() operation
    If the gripper is closed, gripperInit() assumes that there is an object held by
    the gripper, so first, it will perform a putdown operation to allow the gripper
    to be made ready for a pickUp().  Note does not yet set LiftHeight correctly
    Park - Puts the gripper into it's parking or storage position:- down with the paddles
    closed.
    Wait - Waits for the gripper to complete a move or operation.  Useful for synchronising
    other moves or sequences, or preventing the robot from moving off before the 
    gripper has completed a pickUp() or putDown()
    PickUp, PutDown - picks and object up 
    """
    global LiftHeight
    if gripCommand =="Init":
        if (gripper.getPaddleState() != 0) or (gripper.getBreakBeamState() != 0) :
            gripper.liftDown()
            sleep(.2)
            gripperWait()
        gripper.gripOpen()
        gripperWait()
        sleep(.5)
        gripper.liftUp()
    elif gripCommand =="Park":
        gripperWait()
        gripper.liftDown()
        sleep(.2)
        while (gripper.isLiftMoving()==1):
            #print "Lift moving down"
            pass
        gripper.gripClose()
        LiftHeight=0
    elif gripperCommand=="PickUp":
        """
        Will pick up any object it finds as the gripper moves vertically downwards.
        If the paddle beams are broken or the paddles hit a solid surface, the 
        vertical motion will stop and the paddles will close, hopefully picking up
        any object that is present. There is no way to directly verify if an object
        is in the gripper, other than the fact that the gripper is closed.  
        """
        gripperWait()
        if gripper.getPaddleState() == 0:
            gripper.liftDown()
            sleep(.2)
            while gripper.isLiftMoving()==1 and gripper.getBreakBeamState() == 0:
                # print "Nothing to pick up yet"
                pass 
            gripper.gripperHalt()
            gripper.gripClose()
            gripperWait()
            sleep(.2)
            gripper.liftUp()
    elif gripperCommand=="PutDown":
        """
        Will put down any object held in the gripper paddles.  There is no way to directly
        verify that there is an object in the paddles, only that they are closed.  
        The gripper will move vertically downwards until a solid surface is touched.
        The gripper paddles will then open, allowing the held object to be released
        """
        gripperWait()
        if gripper.getPaddleState != 0:
            gripper.liftDown()
            sleep(.2)
            gripperWait()
            gripper.gripOpen()
            gripperWait()
            sleep(.2)
            gripper.liftUp()
    elif type(gripperCommand)== 'int':
        liftwait=(height-LiftHeight)*20
        if liftwait>0:
            gripper.liftUp()
        if liftwait<0:
            gripper.liftDown()
        liftwait=abs(liftwait)
        sleep(liftwait)
        gripper.liftStop()
        LiftHeight=height

def grippperWait():
    while (gripper.isLiftMoving()==1) or (gripper.isGripMoving()==1):
        #Print "Gripper moving"
        pass
    
def say(textmsg = "O K", femaleVoice=0, pitch=50):
    """;
    This function speaks the text passed as a string parameter, enclosed within quotes
    If supplied, the second parameter adjusts the pitch of the speaker,
    If female != 0 then female voice is assumed, else male. Note espeak must be installed
    and in the executable search path.
    """
    if femaleVoice==1:
        # Female voice
        msg = 'espeak -v f3 "' + textmsg + ' "'
        print msg
    else:
        # Male voice
        msg = 'espeak "'+ textmsg + ' "'
    print msg
    os.system(msg)

def beep(n=1):
    msg = 'echo -e "\a" > /dev/console'
    i = 0
    while i < n:
        os.system(msg)
        i = i + 1
        sleep(.25)

def tcpCon():
    """
    opens a TC IP socket and passes the message on to be executed and
    waits for input from the TCIP socket and pases it on to Python for 
    evaluation. If "q" input, ends connection, "Q" input ends server. 
    """
    #local variables
    msg = ""
    rmsg = ""
    passw=""
    tcpOk=0
    try:
        # Create IP socket and wait for customers
        tcpSock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    except:
        print "Error creating socket"
    print "Please wait: Binding address to socket"
    while tcpOk==0:
        try:
            tcpSock.bind(("",9000))
            tcpSock.listen(3)
            tcpOk=1
            # when cusomer calls, service requests
        except:
            sleep(2.0)  
    print "Socket ready now"
    try:
        while msg != "Q":
            msg = ""
            rmsg = ""
            #tcpSock.listen(3)
            cli_sock, cli_ipAdd = tcpSock.accept()
            cli_sock.send("Password please:")
            passw = cli_sock.recv(1024)
            print "Client Connected:*"+passw+"*"
            if passw == "cornwall":
                #Then recive input and pass to eval
                msg=""
                try:
                    while (msg != "Q") and (msg != "q"):
                        msg=cli_sock.recv(1024)
                        print msg
                        try:
                            rmsg = eval(msg)
                        except:
                            #print "Error!"
                            rmsg="Error!"
                        print rmsg
                        cli_sock.send(str(rmsg))
                        #cli_sock.send(chr(13)+ ">>> ")
                except:
                    print "Connection aborted"
            print "Connection Closed"
            cli_sock.close()
    except:
        print "IP socket Aborted" 
    #clean up
    tcpSock.close()
    print "IP server closed"
    




    

if __name__ == "__main__":
	# Run TCIP server
	tcpCon()
	# End nicely
	end()
	print "End program"

