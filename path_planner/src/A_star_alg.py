import numpy as np
import matplotlib.pyplot as plt
# Import heapq for priority queue
import heapq

def main():
    # Implement A* algorithm here to find the shortest path from start to goal

    # Map size
    map_size = 1000
    map = np.zeros((map_size, map_size))

    
    # Generate random obstacles in the map in form of rectangles of random size 
    for i in range(100):
        # Random position
        x = np.random.randint(0, map_size)
        y = np.random.randint(0, map_size)
        # Random size
        width = np.random.randint(1, 100)
        height = np.random.randint(1, 100)
        # Add obstacle to map
        map[x:x+width, y:y+height] = 1


    """
    # Inflate the obstacles in the map by r_index
    r_index = 5
    
    # 2d mask in a numpy array with size * 2 * r_index + 1 filled with the index of each element
    mask = np.zeros((r_index * 2 + 1, r_index * 2 + 1,2))
    mask[:,:,0] = np.arange(mask.shape[0])[:,None]
    mask[:,:,1] = np.arange(mask.shape[1])[None,:]
    
    #for i in range(mask.shape[0]):
    #    for j in range(mask.shape[1]):
    #        # Print i,j followed by the value in the mask
    #        print(i,j,mask[i,j])
    
    # Subtract the center of the mask from each element in the mask
    mask_ind = mask - np.array([r_index, r_index])
    
    # Square the mask and take the sum of the two dimensions and take the square root
    mask = np.sqrt(np.sum(mask_ind ** 2, axis=2)).astype(int)
    mask = mask <= r_index
    
    # Save the values of maskind where mask is true
    mask_ind = mask_ind[mask==True]

    # Get the indices of the obstacles in the map in a list of tuples
    obstacle_indices = np.argwhere(map == 1)
    
    obstacle_indices_inflated = []
    # Loop through the list of obstacle indices and add the inflated obstacles to the map
    for obstacle_index in obstacle_indices:
        new_obstacle_indices = mask_ind + obstacle_index
        obstacle_indices_inflated.append(new_obstacle_indices)
        pass
    
    # Flatten the list of obstacle indices
    obstacle_indices_inflated = np.concatenate(obstacle_indices_inflated).astype(int)

    # Remove duplicate indices
    obstacle_indices_inflated = np.unique(obstacle_indices_inflated, axis=0)

    # Remove the indices that are outside the map
    obstacle_indices_inflated = obstacle_indices_inflated[(obstacle_indices_inflated[:,0] >= 0) & (obstacle_indices_inflated[:,0] < map.shape[0])]
    obstacle_indices_inflated = obstacle_indices_inflated[(obstacle_indices_inflated[:,1] >= 0) & (obstacle_indices_inflated[:,1] < map.shape[1])]
    map[obstacle_indices_inflated[:,0], obstacle_indices_inflated[:,1]] = 1
    """
    for i in range(10):
        # Start and goal position of the robot randomly chosen in the map
        # Make sure that the start and goal position are not inside an obstacle
        start = (np.random.randint(0, map_size), np.random.randint(0, map_size))
        end = (np.random.randint(0, map_size), np.random.randint(0, map_size))
        while map[start[0], start[1]] == 1:
            start = (np.random.randint(0, map_size), np.random.randint(0, map_size))
        while map[end[0], end[1]] == 1:
            end = (np.random.randint(0, map_size), np.random.randint(0, map_size))


        # Add start and goal position to the map
        map[start[0], start[1]] = 2
        map[end[0], end[1]] = 3


        # Implement A* algorithm here to find the shortest path from start to goal
        # The path should be a list of tuples (x, y) of the coordinates of the path
        path = A_star(map, start, end)

        # Add the path to the map
        for pos in path:
            map[pos[0], pos[1]] = 1    # Start and goal position of the robot randomly chosen in the map



    # Plot the map in plt, different colors for different integers in the map
    plt.imshow(map)
    plt.show()



def A_star(map, start, end):
    def heuristic(a, b):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def neighbors(pos):
        for dx, dy in ((0, 1), (1, 0), (0, -1), (-1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1),(0,0)):
            x, y = pos[0] + dx, pos[1] + dy
            if 0 <= x < map.shape[0] and 0 <= y < map.shape[1] and map[x, y] != 1:
                yield (x, y)

    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}

    while frontier:
        _, current = heapq.heappop(frontier)

        if current == end:
            break

        for next in neighbors(current):
            new_cost = cost_so_far[current] + 1  # cost of movement is 1
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(end, next)
                heapq.heappush(frontier, (priority, next))
                came_from[next] = current

    path = []
    if end in came_from:
        node = end
        while node != start:
            path.append(node)
            node = came_from[node]
        path.append(start)
        path.reverse()

    return path        






main()