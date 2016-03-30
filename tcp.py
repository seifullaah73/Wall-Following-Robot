#! /usr/bin/env python
"""
TCP Client Socket for the PeopleBot Robot using Python. Test file.
M Walters. V0.5 February 2008
M L Walters, V1.0 March 2010

This program if imported implements one function:

send(msg)

This sends a text string in msg to the rbIp Python robot server
(which can be connected to a simulator or reall robot).

Example;

import tcp
tcp.send("move()")   # Robot will move until it senses an obstacle
tcp.send("move(90)") # Robot will trun ACW 90 degrees
tcp.send("stop()")   #Robot stops
tcp.send("q")        # closes tcp connection with robot/simulator

"""

import socket
msg=""
rmsg=""
# Create the IP socket
tcpSock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Then connect to the server address, specifying the remote portno
# to use

# UHP2B = Pioneer base B = IP Address 192.168.0.196
#tcpSock.connect(("192.168.0.196",9000))

# MobileSim = Simulator on localhost
tcpSock.connect (("localhost",9000))

rmsg=""
rmsg = tcpSock.recv(1024)
print rmsg
tcpSock.send("cornwall")
print "Password sent ok"

def send(msg):
    """
    Sends a text string (msg) to the robot or simulator Python
    server. See above for example/usage
    """
    rmsg=""
    tcpSock.setblocking(0)
    while rmsg != "":
        try:
            rmsg = tcpSock.recv(1024)
            print rmsg
        except:
            rmsg = ""
            print rmsg
    if msg == "" : msg = chr(13)
    tcpSock.send(msg)
    tcpSock.setblocking(1)
    try:
        rmsg = tcpSock.recv(1024)
    except:
        rmsg = ""
    tcpSock.setblocking(1)
    return rmsg

def close():
    """
    closes the tcp connection and finishes the session nicely
    """
    tcpSock.send("q")
    tcpSock.close()
    


def terminal():
    """
    This program just implements a simple terminal link to the
    Robot or Simulator python server. For list of commands see
    the example program above, or the rbIP manual.
    """
    msg=""
    rmsg = ""
    print " q to quit, Q to quit and shutdown server"
    # Then simply send messages in (msg)and print return message(s)
    # Until "Q" recieved or sent
    tcpSock.setblocking(0)
    while (msg != "Q") and (msg != "q"):
        tcpSock.setblocking(0)
        while rmsg != "":
            try:
                rmsg = tcpSock.recv(1024)
                print rmsg
            except:
                rmsg = ""
                print rmsg
        msg = raw_input(": ")
        if msg == "" : msg = chr(13)
        tcpSock.send(msg)
        tcpSock.setblocking(1)
        try:
            rmsg = tcpSock.recv(1024)
            print rmsg
        except:
            rmsg = ""
            print rmsg
    tcpSock.setblocking(1)
    

if __name__ == "__main__":
    terminal()
    print "Finished OK"
    tcpSock.close()

