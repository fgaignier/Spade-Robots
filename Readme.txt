
The goal of this project is to propose a multi-agent system using SPADE as multi-agent plateform

We have two roles:

NAO Agent
TURTLE Agent

Each agent is runing (or connected to) a physical robot.

NAO Agent on NAO
TURTLE AGENT on a Turtlebot 2 with a widowx arm mounted on it

Prior to starting the turtle agent it is necessary to start the services on the turtlebot machine (cf. turtlebotcomment.sh)

In turtlebotcomment.sh we have listed the various steps to 
1) install all the packages on the turtlebot (and there are many of them)
2) start the needed services

SPADE also proposes a web interface where you can see all the registered agents.
You can use this interface to send manually mesages to any agent.

In the course of their interactions, agents send also messages to each other.


