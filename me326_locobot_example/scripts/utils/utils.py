import numpy as np
import rospy

def wrapToPi(a):
    if isinstance(a, list):
        return [(x + np.pi) % (2*np.pi) - np.pi for x in a]
    return (a + np.pi) % (2*np.pi) - np.pi

class StochOccupancyGrid2D(object):
    def __init__(self, resolution, width, height, origin_x, origin_y,
                window_size, probs, thresh=0.5):
        self.resolution = resolution
        self.width = width
        self.height = height
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.probs = probs
        self.window_size = window_size
        self.thresh = thresh

    def snap_to_grid(self, x):
        return (self.resolution*round(x[0]/self.resolution), self.resolution*round(x[1]/self.resolution))

    def is_free(self, state):
        # combine the probabilities of each cell by assuming independence
        # of each estimation
        return True
        p_total = 1.0
        lower = -int(round((self.window_size-1)/2))
        upper = int(round((self.window_size-1)/2))
        for dx in range(lower,upper+1):
            for dy in range(lower,upper+1):
                x, y = self.snap_to_grid([state[0] + dx * self.resolution, state[1] + dy * self.resolution])
                grid_x = int((x - self.origin_x) / self.resolution)
                grid_y = int((y - self.origin_y) / self.resolution)
                if grid_y>0 and grid_x>0 and grid_x<self.width and grid_y<self.height:
                    p_total *= (1.0-max(0.0,float(self.probs[grid_y * self.width + grid_x])/100.0))
                    
        return (1.0-p_total) < self.thresh
