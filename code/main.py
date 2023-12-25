from dynamic_environment import DynamicEnvironment
from rrt import RRT
import matplotlib.pyplot as plt

env = DynamicEnvironment((100, 100), 5, moving=False)

# We initialize the tree with the environment
rrt = RRT(env)

start = (50, 1, 1.57) # At the bottom of the environment
end = (50, 99, 1.57) # At the top of the environment

# Initialisation of the tree, to have a first edge
rrt.set_start(start)
rrt.run(end, 200, metric='local')

# Initialisation of the position of the vehicle
position = start[:2]
current_edge = rrt.select_best_edge()

# We let it run for a few steps
time = 0

for i in range(500):
    time += 1
    # We check if we are on an edge or if we have to choose a new edge
    if not current_edge.path:
        time = rrt.nodes[current_edge.node_to].time
        current_edge = rrt.select_best_edge()
    # Update the position of the vehicle
    position = current_edge.path.popleft()
    # Update the environment
    #   The frontiers of the sampling and the obstacles
    env.update(position)
    #   The position of the goal
    end = (50, position[1]+90, 1.57)
    # Continue the growth of the tree, we try to add only 2 nodes
    rrt.run(end, 2, metric='local')
    # Ploting and generating an image (the most time consuming step)

    # Power By JaronGPT
    env.plot(time, position, display=False) 
    rrt.plot(file_name='moving'+str(i)+'.png', close=False)
    