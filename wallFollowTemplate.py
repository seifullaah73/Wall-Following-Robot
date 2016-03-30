#!/usr/bin/python
"""
rbIP-Template
M L Walters, V1 March 2011
This is a template for using the rbIP python interface to the Aria 
controlled simulated Pioneer mobile robot. 
"""

"""
Name:Seifullaah Sherrief ID:07158462

Wall Following Program Robot

The work carried out was using back propagation as a learning method to train a network to
learn a logic from a determined truth table acquired from the input sensors of the robot
and create a program the robot to move towards the wall and follow it on, keeping the wall
on its left.

Firstly data was acquired from the sensors and stored in variable e.g. the data from
the front sensor in distances is measured if a certain distance front variable set to 1
so it is like a flag. i.e. nearest(0,40) front sensor with sweep angle of 40 sense distance to
nearest wall. if nearest(0,40)<700, if that distance is less than 700 than wall in from too close
set front variable to 1. so later on in the code when the front or any other sensor variable
flags are set certain actions are carried out i.e. move(20) rotate 20 degrees left.
The different data collected were:

    1.distance from the front sensor nearest(0,40) with sweep angle 40
    2.distance from the left nearest(90) to determine if wall to left is far or close or
      normal distance
    3.both distance from left and bottom left and if there is a difference than outside corner
      detected
    4.if distance of left sensor and (upper left and lower left) sensor are different than
      inside corner
      detected.

Then using the back propagation neural network class file, to generate a truth table
according to the different inputs in this case the sensors.
e.g.
Input

Front|Left Close|Left Far|Corner|Inside Corner|
  x1 |   x2     |  x3    |  x4  |    x5       |
     
Output

Rototae Right| Rot Slightly Right| rotate slightly left|rotate hard left|rotate hard right|

so if e.g input was front==1
then x1=1,x2=0,x3=0,x4=0,x5=0
and y=[1,0,0,0,0]

truth table becomes the following:

       Input                Output
X1 | X2 | X3 | X4 | X5 | Y1 | Y2 |Y3 | Y4 | Y5         
 1   0    0    0     0   1     0   0   0    0

then neural network learns this using back propagation using momentum.

A goood rule of thumb is for the number of hidden layers to be 2 more than the number of output nodes
so in this case, we 5 inputs, 5 outputs so a good number for the number of nodes in the hidden layer is 7.

Architecture:- firstly distance is measured from the different sensors i.e. front
left to detect the current position of the robot. The trained neural network function
is called whenever these flags are set fed to the neural network, which a truth table
is create in which an output is determine for every different pattern input. The network
then recogineses each output pattern to a certain action example move. then the certain action
is returned to the calling function. e.g if wall on left is too far so leftfar variable
set to 1 and the truth table is generated accordingly and a matching output is produced
from the network, then the robot wil be move/ rotate slightly to the left. if the the
left sensor is higher than 900 and the back left sensor is less than 900 than an outside
corner is detected and it should turn hard to the left. once finished it will try
to move then it stops and then loops to the first part where distance is measured from
the sensors and this procedure is repeated.

Problems Encountered:-
    1.problems encountered were that if the left sensor detected the wall to be far it would
      rotate anticlockwise and would do this continuously.
    2.from start position the robot would rotate slightly to left, even though the left sensor
      detects the wall to be far it would move forward so therefore moving further away from the wall.
    3.it would approach outside corner and keep moving forward even passing the outside corner and heading
      straight forward until wall encounterd in front and continue by sensing obstbacle in front and to
      turn right.
    4.it would get stuck in an inside corner where front sensor detects nothing so robot rotates right
      then a little left because of the left wall being too far then right and left and would continue
      doing this.

Control Performance:-
The control performance of the trained neural network is based on a few factors which include the following:
    1. Iteration no. - the iteration is the number of loops required when training to bring the error between the
       desired output and the actual output as close to zero as possible.
    2. Learning Factor - this is the rate at which the network learns at having it too high would cause instability
       with the output not reaching the desired output.

Suitability:-

The type of neural network used here would be suitable for this type of control application as a trained neural
network behaves as the brain of the robot, so according to what it has learnt/ been programmed to learn it would behave
accordingly in this case when it approaches a wall it would try and keep the wall on its side.

Additional Features:- Additional features added were that a flag was set up to indicate whether the robot was in an inside corner
so that the robot turned hard right.

Also the move function of the robot would cause the robot to move up to a spot passing spots where it should have
detected the wall on the left being too far only when the front sensor is detected does it stop so the program was change
for the robot to make fixed distance movements then stop then sense that way it would sense better once it has stopped.
"""

# Useful function, sleep(n) where n is in secs
from time import sleep


# Comment or uncomment as required to use the Simulator
from rbIPSim import *
# If you want use the real robot use the following
# from rbIPV28 import *


# Just allows user to input valid robot commands (all python functions
# defined in rbIPV28. See manual for details
# You can try wander(), nearest(), move(90, 1000) etc.

from neuralnetwork import *

def wallFollow():
	"""
	Robot should exhibits the following behviours:
	If not close to a wall, will move until a wall is encountered
	Will then keep the wall on the left side of the robot, keeping
	a constant distance from the wall round corners
	"""
	print "wallfollow"
	
	# Main loop
	while 1:
		stop()
		# Read range sensors, 
		# Front distance
		if nearest(0, 40)< 700: front =1
		elif nearest(40,40)<700: front =1
		elif nearest(-40,40)<700: front =1
		else: front = 0
		
		# Left sensors, keep robot within lmits:- leftclose and leftfar
		n = nearest(90)
		print "Wall distance ",n
		if n < 500: leftclose = 1; leftfar =0
		elif n > 650: leftclose = 0; leftfar =1
		elif n > 900: leftclose=0; leftfar=0
		else: leftclose =0; leftfar=0

		# Outside corner detect;- uses difference between front/left and rear/left 
		# distances to indicate corner
		if (nearest(80)>900) and (nearest(120)< 900): corner = 1; print "outisde corner detected"
		else: corner = 0

		#Inside corner detect;- uses diagonal sensor and lower left sensor
		if (nearest(40,40)<700 and (nearest(100,40)<700) and nearest(90)>700): incorner = 1; print "Inside corner detected"
		else: incorner = 0

		# If wall in front
		if front==1:
			# turn right
			testneural(1,0,0,0,0)
		#if wall is too close on left
		elif leftclose==1:
			#turn slightly to right
			testneural(0,1,0,0,0)

                #If going around an inside corner
		elif incorner==1:
			#turn slightly to right
			testneural(0,0,0,0,1)
			
		#if wall to far away, 
		elif leftfar==1:
			# turn slightly to left
			testneural(0,0,1,0,0)           
		# If going past an outside corner
		elif corner==1:
			# turn hard left
			testneural(0,0,0,1,0)
			
						
		# try moving forward
		sleep(1)
		stop()
		move(0,300)
		sleep(2)
		stop()
		
	# End loop. Repeat

		
	
	
	
# Just allows user to input valid robot commands (all python functions
# defined in rbIPV28. See manual for details
# You can try wander(), nearest(), move(90, 1000) etc. 
# You can try out any of the above or rbIPSim functions here 
# wallFollow() to run the wall following function (above)
msg=""
while msg!= "q":
	msg = raw_input("Robot command: ") 
	if msg!="q" or msg!="":
		print eval(msg)
		#try:
		#   print eval(msg)
		#except:
		#   print "ERROR!"
	#moveWait()

