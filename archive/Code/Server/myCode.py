# Import the Control class from the Control module
from control import Control

# Creating object 'control' of 'Control' class.
c = Control()

# Move forward in action mode 1 and gait mode 1
for i in range(3):
    data = ['CMD_MOVE', '1', '0', '35', '10', '0']
    c.run_gait(data)  # Run gait with specified parameters

# Move right in action mode 1 and gait mode 1
for i in range(3):
    data = ['CMD_MOVE', '1', '35', '0', '10', '0']
    c.run_gait(data)  # Run gait with specified parameters

# Move backward in action mode 2 and gait mode 2    
for i in range(3):
    data = ['CMD_MOVE', '2', '0', '-35', '10', '10']
    c.run_gait(data)  # Run gait with specified parameters
    
# Move right in action mode 2 and gait mode 2    
for i in range(3):
    data = ['CMD_MOVE', '2', '35', '0', '10', '10']
    c.run_gait(data)  # Run gait with specified parameters