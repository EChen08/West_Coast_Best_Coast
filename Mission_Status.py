from BWSI_Sandshark import Sandshark
from BWSI_BuoyField import BuoyField
from AUV_Controller import AUVController

import numpy as np
import matplotlib.pyplot as plt


doPlots = True

# create a buoy field
datum = (42.4, -171.3)
myAUV = Sandshark(latlon=(42.4, -171.3), heading=90, datum=datum)


### ################################

running_time = 0
dt = 1
cmd = "ENGINE HALF AHEAD"
reply = myAUV.engine_command(cmd)
print(f"{cmd} : {reply}")

# keep the track history
auv_track = list()
auv_track.append(myAUV.get_position())

if doPlots:
    fig = plt.figure()
    ax = fig.add_subplot(111)

done = False

auv_state = myAUV.get_state()

# ***YOUR CODE HERE****
auv_controller = AUVController(auv_state)

num_commands = 0
frame_skip = 10
frame = 0
while not done:
    myAUV.update_state(dt)
    current_position = myAUV.get_position()
        
    auv_track.append(current_position)
    
    # current heading of vehicle
    auv_state = myAUV.get_state()

    # ***YOUR CODE HERE. ***    
    command = auv_controller.decide(auv_state)
    
    if command is not None:
        reply = myAUV.helm_command(command)
        print(f"{reply}")
        num_commands += 1

    # ***YOUR CODE HERE***    
    tgt_hdg = auv_controller.get_desired_heading()
    
    print(f"auv_heading is {auv_state['heading']}, target heading is {tgt_hdg}")
        
    if doPlots and not (frame % frame_skip):
        trk = np.array(auv_track)
        plt.plot(trk[-300:,0], trk[-300:,1], 'k')            

        ax.set_xlim(current_position[0]-100, current_position[0]+100)
        ax.set_ylim(current_position[1]-100, current_position[1]+100)
        ax.set_aspect('equal')

        plt.pause(0.01)
        plt.draw()       
        
    frame += 1

    running_time = running_time + dt
    
