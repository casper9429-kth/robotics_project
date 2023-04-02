import numpy as np



# Di Dj
inf_range = 2
resolution = 1
Dij = 2*int(inf_range/resolution) + 1
Dij_half = int(inf_range/resolution)
circle_mask = np.zeros((Dij,Dij,2))
# Create array with its indices as values
circle_mask[:,:,0] = np.arange(-Dij_half,Dij_half+1,1)[None,:].T
circle_mask[:,:,1] = np.arange(-Dij_half,Dij_half+1,1)[None,:]
# Radius of circle mask is 0.4, normalized to resolution it is 
radius = inf_range/resolution
# save elements of circle_mask that are inside the circle
circle_mask = circle_mask[np.sqrt(circle_mask[:,:,0]**2 + circle_mask[:,:,1]**2) <= radius]
#add index to all elements of array circle_mask
circle_mask[:,0] = circle_mask[:,0] + 10
circle_mask[:,1] = circle_mask[:,1] + 10
circle_mask = circle_mask.astype(int)
# Create big array of zeros
big = np.zeros((30,30))
big[(circle_mask[:,0],circle_mask[:,1])] = 1

print(big)
