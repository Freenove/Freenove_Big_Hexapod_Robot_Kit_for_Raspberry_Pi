#Import everything in the control module, 
#including functions, classes, variables, and more.
from Control import *

#Creating object 'control' of 'Control' class.
c=Control()

#example:
#data=['CMD_MOVE', '1', '0', '25', '10', '0']
#Move command:'CMD_MOVE'
#Gait Mode: "1"
#Moving direction: x='0',y='25'
#Delay:'10'
#Action Mode : '0'   Angleless turn 

#Move forward in action mode 1 and gait mode 1
for i in range(3):
    data=['CMD_MOVE', '1', '0', '35', '10', '0']
    c.run(data)

#Move right in action mode 1 and gait mode 1
for i in range(3):
    data=['CMD_MOVE', '1', '35', '0', '10', '0']
    c.run(data)
#Move backward in action mode 2 and gait mode 2    
for i in range(3):
    data=['CMD_MOVE', '2', '0', '-35', '10', '10']
    c.run(data)
    
#Move right in action mode 2 and gait mode 2    
for i in range(3):
    data=['CMD_MOVE', '2', '35', '0', '10', '10']
    c.run(data)

