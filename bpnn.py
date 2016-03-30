#!/usr/bin/python
"""
Back-Propagation Neural Networks
 
Written in Python.  See http://www.python.org/
Placed in the public domain.
Neil Schemenauer <nas@arctrix.com>

Modified by M. L. Walters, March, 2011

"""

import math
import random

# can set different seed values for pseudo random number generator
random.seed(0)

# calculate a random number where:  a <= rand < b
def rand(a, b):
    return (b-a)*random.random() + a

# Make a matrix (we could use NumPy to speed this up)
def makeMatrix(I, J, fill=0.0):
    m = []
    for i in range(I):
        m.append([fill]*J)
    return m

# Standard sig function
def sig(x):
    return 1/(1 + math.exp(-x))
# Derivative of sig function (dE/dW)    
def dsig(y):
    return y*(1-y)


# our sigmoid function, tanh is a little nicer than the standard 1/(1+e^-x)
def sigtanh(x):
    return math.tanh(x)

# derivative of our sigmoid function, in terms of the output (i.e. y)
def dsigtanh(y):
    return 1.0 - y**2

class NN:
    """
    Neural network class for a 3 layer MLP, with backpropagation training
    Parameters: ( ni = no. of inputs,
                  nh = no. of hidden units
                  no = no. of output units,
                  sigmoid = sigtanh (default) or sig,  activation function,
                  dsigmoid = dtanh (default) or dsig, act func. dE/dW)
    """
    def __init__(self, ni, nh, no, sigmoid=sigtanh , dsigmoid=dsigtanh):
        # number of input, hidden, and output nodes
        self.ni = ni + 1 # +1 for bias node
        self.nh = nh
        self.no = no
        self.sigmoid=sigmoid
        self.dsigmoid=dsigmoid

        # activations for nodes
        self.ai = [1.0]*self.ni
        self.ah = [1.0]*self.nh
        self.ao = [1.0]*self.no
        
        # create weights
        self.wi = makeMatrix(self.ni, self.nh)
        self.wo = makeMatrix(self.nh, self.no)
        # set them to random vaules
        for i in range(self.ni):
            for j in range(self.nh):
                self.wi[i][j] = rand(-0.2, 0.2)
        for j in range(self.nh):
            for k in range(self.no):
                self.wo[j][k] = rand(-2.0, 2.0)

        # last change in weights for momentum   
        self.ci = makeMatrix(self.ni, self.nh)
        self.co = makeMatrix(self.nh, self.no)

    def update(self, inputs):
        if len(inputs) != self.ni-1:
            raise ValueError('wrong number of inputs')

        # input activations
        for i in range(self.ni-1):
            #self.ai[i] = sigmoid(inputs[i])# can use to limit imput values
            self.ai[i] = inputs[i]

        # hidden activations
        for j in range(self.nh):
            sum = 0.0
            for i in range(self.ni):
                sum = sum + self.ai[i] * self.wi[i][j]
            self.ah[j] = self.sigmoid(sum)

        # output activations
        for k in range(self.no):
            sum = 0.0
            for j in range(self.nh):
                sum = sum + self.ah[j] * self.wo[j][k]
            self.ao[k] = self.sigmoid(sum)
        return self.ao[:]


    def backPropagate(self, targets, N, M):
        if len(targets) != self.no:
            raise ValueError('wrong number of target values')

        # calculate error terms for output
        output_deltas = [0.0] * self.no
        for k in range(self.no):
            error = targets[k]-self.ao[k]
            output_deltas[k] = self.dsigmoid(self.ao[k]) * error

        # calculate error terms for hidden
        hidden_deltas = [0.0] * self.nh
        for j in range(self.nh):
            error = 0.0
            for k in range(self.no):
                error = error + output_deltas[k]*self.wo[j][k]
            hidden_deltas[j] = self.dsigmoid(self.ah[j]) * error

        # update output weights
        for j in range(self.nh):
            for k in range(self.no):
                change = output_deltas[k]*self.ah[j]
                self.wo[j][k] = self.wo[j][k] + N*change + M*self.co[j][k]
                self.co[j][k] = change
                #print N*change, M*self.co[j][k]

        # update input weights
        for i in range(self.ni):
            for j in range(self.nh):
                change = hidden_deltas[j]*self.ai[i]
                self.wi[i][j] = self.wi[i][j] + N*change + M*self.ci[i][j]
                self.ci[i][j] = change

        # calculate error
        error = 0.0
        for k in range(len(targets)):
            error = error + 0.5*(targets[k]-self.ao[k])**2
        return error


    def test(self, patterns):
        """
        Applies test/verification patterns set
        """
        for p in patterns:
            print(p[0], '->', self.update(p[0]))

    def showweights(self):
        """
        Pretty prints out weights
        """
        print('Input weights:')
        for i in range(self.ni):
            print(self.wi[i])
        print()
        print('Output weights:')
        for j in range(self.nh):
            print(self.wo[j])

    def train(self, patterns, iterations=1000, N=0.5, M=0.1):
        """
        Trains the network using the training patterns set, for default
        1000 iterations, with learning rate N (default 0.5) and momentum
        M (default = 0.1)
        """
        # N: learning rate
        # M: momentum factor
        for i in range(iterations):
            error = 0.0
            for p in patterns:
                inputs = p[0]
                targets = p[1]
                self.update(inputs)
                error = error + self.backPropagate(targets, N, M)
            if i % 100 == 0:
                print('error %-.5f' % error)


def demo():
    # Teach network XOR function
    xorPat = [
            [[-1, 0,0], [0]],
            [[-1, 0,1], [1]],
            [[-1, 1,0], [1]],
            [[-1, 1,1], [0]]
            ]

    # create a network with two input, two hidden, and one output nodes
    n = NN(3, 2, 1) #default uses tanh activation function  
    # train it with some patterns
    n.train(xorPat, 1000, .2, 0.1)
    # test it
    n.test(xorPat)

def demo2():
    # Teach network XOR function
    xnorPat = [
            [[-1, 0,0], [1]],
            [[-1, 0,1], [0]],
            [[-1, 1,0], [0]],
            [[-1, 1,1], [1]]
            ]

    # create a network with two input, two hidden, and one output nodes
    n = NN(3, 2, 1) #default uses tanh activation function  
    # train it with some patterns
    n.train(xnorPat, 1000, .2, 0.1)
    # test it
    n.test(xnorPat)

def demo3():
    # Teach network OR function
    orPat = [
            [[-1, 0,0], [0]],
            [[-1, 0,1], [1]],
            [[-1, 1,0], [1]],
            [[-1, 1,1], [1]]
            ]

    # create a network with two input, two hidden, and one output nodes
    n = NN(3, 2, 1) #default uses tanh activation function  
    # train it with some patterns
    n.train(orPat, 1000, .2, 0.1)
    # test it
    n.test(orPat)

def demo4():
    # Teach network AND function
    andPat = [
            [[-1, 0,0], [0]],
            [[-1, 0,1], [0]],
            [[-1, 1,0], [0]],
            [[-1, 1,1], [1]]
            ]

    # create a network with two input, two hidden, and one output nodes
    n = NN(3, 2, 1) #default uses tanh activation function  
    # train it with some patterns
    n.train(andPat, 1000, .2, 0.1)
    # test it
    n.test(andPat)

def demo5():
    # Teach network XNOR function
    nandPat = [
            [[-1, 0,0], [1]],
            [[-1, 0,1], [1]],
            [[-1, 1,0], [1]],
            [[-1, 1,1], [0]]
            ]

    # create a network with two input, two hidden, and one output nodes
    n = NN(3, 2, 1) #default uses tanh activation function  
    # train it with some patterns
    n.train(nandPat, 1000, .2, 0.1)
    # test it
    n.test(nandPat)
if __name__ == '__main__':
    demo()
    demo2()
    demo3()
    demo4()
