from rbIPSim import *
from bpnn import *

def testneural(front,leftclose,left,corner):
    # Teach network new truth table of sensors of robot function

    if front==1: y=[1,0,0,0]
    elif leftclose==1: y=[0,1,0,0]
    elif left==1: y=[0,0,1,0]
    elif corner==1: y=[0,0,0,1]

    testneural=[[[-1,front,leftclose,left,corner],y]]
        
    # four input including bias input, seven hidden, and four output nodes
    n = NN(5, 6, 4) # tanh activation function  
    #  some patterns
    n.train(testneural,200, .2, 0.1)
    # test it
    n.test(testneural)

    
    if y==[1,0,0,0]:
        return move(-35);
        
    elif y==[0,1,0,0]:
        return move(-10);
    
    elif y==[0,0,1,0]:
        return move(10);
    
    elif y==[0,0,0,1]:
        return move(55);
       
if __name__ == '__main__':
    testneural()

    
