from rbIPSim import *
from bpnn import *

def testneural(fr=0,lc=0,lf=1,crnr=0,incrnr=0):
    # Teach network new truth table of sensors of robot function

    if fr==1: y=[0,1,0,0,0]
    elif lc==1: y=[0,0,1,0,0]
    elif lf==1: y=[0,0,0,1,0]
    elif crnr==1: y=[0,0,0,0,1]
    elif incrnr==1: y=[1,0,0,0,0]
    
    testneural=[[[-1,fr,lc,lf,crnr,incrnr],y]]
        
    # create a network with five input including bias input, seven hidden, and five output nodes
    n = NN(6, 7, 5) #default uses tanh activation function  
    # train it with some patterns
    n.train(testneural,200, .2, 0.1)
    # test it
    n.test(testneural)

    if y==[1,0,0,0,0]:
        return move(-35);
    
    elif y==[0,1,0,0,0]:
        return move(-25);
        
    elif y==[0,0,1,0,0]:
        return move(-15);
    
    elif y==[0,0,0,1,0]:
        return move(15);
    
    elif y==[0,0,0,0,1]:
        return move(35);
       
if __name__ == '__main__':
    testneural()

    

