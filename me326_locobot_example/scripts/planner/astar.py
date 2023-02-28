import numpy as np
import scipy

class AStar(object):
    """Represents a motion planning problem to be solved using A*"""

    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, occupancy, resolution):
        self.statespace_lo = statespace_lo
        self.statespace_hi = statespace_hi
        self.occupancy = occupancy                 # occupancy grid 
        self.resolution = resolution               # resolution of the discretization of state space (cell/m)
        self.x_init = self.snap_to_grid(x_init)    # initial state
        self.x_goal = self.snap_to_grid(x_goal)    # goal state

        self.closed_set = set()    # the set containing the states that have been visited
        self.open_set = set()      # the set containing the states that are candidate for future expansion

        self.est_cost_through = {}  # dictionary of the estimated cost from start to goal passing through state (often called f score)
        self.cost_to_arrive = {}    # dictionary of the cost-to-arrive at state from start (often called g score)
        self.came_from = {}         # dictionary keeping track of each state's parent to reconstruct the path

        self.open_set.add(self.x_init)
        self.cost_to_arrive[self.x_init] = 0
        self.est_cost_through[self.x_init] = self.distance(self.x_init,self.x_goal)

        self.path = None        # the final path as a list of states

    def is_free(self, x):
        """
        Checks if a give state x is free, meaning it is inside the bounds of the map and
        is not inside any obstacle.
        Inputs:
            x: state tuple
        Output:
            Boolean True/False
        """
        for dim in range(len(x)):
            if x[dim] < self.statespace_lo[dim]:
                return False
            if x[dim] >= self.statespace_hi[dim]:
                return False
        return self.occupancy.is_free(x)

    def distance(self, x1, x2):
        return np.linalg.norm(np.array(x1)-np.array(x2))

    def snap_to_grid(self, x):
        """ Returns the closest point on a discrete state grid
        Input:
            x: tuple state
        Output:
            A tuple that represents the closest point to x on the discrete state grid
        """
        return (self.resolution*round(x[0]/self.resolution), self.resolution*round(x[1]/self.resolution))

    def get_neighbors(self, x):
        """
        Gets the FREE neighbor states of a given state x. Assumes a motion model
        where we can move up, down, left, right, or along the diagonals by an
        amount equal to self.resolution.
        Input:
            x: tuple state
        Ouput:
            List of neighbors that are free, as a list of TUPLES
        """
        neighbors = []
        #N
        N = self.snap_to_grid((x[0],x[1]+self.resolution))
        if self.is_free(N): neighbors.append(N)
        #NE
        NE = self.snap_to_grid((x[0]+self.resolution,x[1]+self.resolution))
        if self.is_free(NE): neighbors.append(NE)
        #E
        E = self.snap_to_grid((x[0]+self.resolution,x[1]))
        if self.is_free(E): neighbors.append(E)
        #SE
        SE = self.snap_to_grid((x[0]+self.resolution,x[1]-self.resolution))
        if self.is_free(SE): neighbors.append(SE)
        #S
        S = self.snap_to_grid((x[0],x[1]-self.resolution))
        if self.is_free(S): neighbors.append(S)
        #SW
        SW = self.snap_to_grid((x[0]-self.resolution,x[1]-self.resolution))
        if self.is_free(SW): neighbors.append(SW)
        #W
        W = self.snap_to_grid((x[0]-self.resolution,x[1]))
        if self.is_free(W): neighbors.append(W)
        #NW
        NW = self.snap_to_grid((x[0]-self.resolution,x[1]+self.resolution))
        if self.is_free(NW): neighbors.append(NW)
        return neighbors

    def find_best_est_cost_through(self):
        """
        Gets the state in open_set that has the lowest est_cost_through
        Output: A tuple, the state found in open_set that has the lowest est_cost_through
        """
        return min(self.open_set, key=lambda x: self.est_cost_through[x])

    def reconstruct_path(self):
        """
        Use the came_from map to reconstruct a path from the initial location to
        the goal location
        Output:
            A list of tuples, which is a list of the states that go from start to goal
        """
        path = [self.x_goal]
        current = path[-1]
        while current != self.x_init:
            path.append(self.came_from[current])
            current = path[-1]
        return list(reversed(path))

    def solve(self):
        """
        Solves the planning problem using the A* search algorithm. It places
        the solution as a list of tuples (each representing a state) that go
        from self.x_init to self.x_goal inside the variable self.path
        Input:
            None
        Output:
            Boolean, True if a solution from x_init to x_goal was found
        """
        
        max_iter = 0
        while self.open_set and max_iter < 1000:
            max_iter +=1
            xcurrent = self.find_best_est_cost_through()
            if xcurrent == self.x_goal:
                self.path = self.reconstruct_path()
                return True
            self.open_set.remove(xcurrent)
            self.closed_set.add(xcurrent)
            for xneigh in self.get_neighbors(xcurrent):
                if xneigh in self.closed_set:
                    continue
                g_score = self.cost_to_arrive[xcurrent] + self.distance(xcurrent,xneigh)
                if xneigh not in self.open_set:
                    self.open_set.add(xneigh)
                elif  g_score > self.cost_to_arrive[xneigh]:
                    continue
                self.came_from[xneigh] = xcurrent
                self.cost_to_arrive[xneigh] = g_score
                self.est_cost_through[xneigh] = g_score + self.distance(xneigh,self.x_goal)
        return False
    
def compute_smoothed_traj(path, V_des, alpha, dt):
    """
    Fit cubic spline to a path and generate a resulting trajectory for our
    wheeled robot.
    Inputs:
        path (np.array [N,2]): Initial path
        V_des (float): Desired nominal velocity, used as a heuristic to assign nominal
            times to points in the initial path
        alpha (float): Smoothing parameter (see documentation for
            scipy.interpolate.splrep)
        dt (float): Timestep used in final smooth trajectory
    Outputs:
        traj_smoothed (np.array [N,7]): Smoothed trajectory
        t_smoothed (np.array [N]): Associated trajectory times
    """
    times = [0]
    for i in range(1, len(path)):
        dist = np.linalg.norm(np.array(path[i]) - np.array(path[i-1]))
        times.append(times[i-1] + dist / V_des)

    spl_x = scipy.interpolate.splrep(times, [x[0] for x in path], k=3, s=alpha)
    spl_y = scipy.interpolate.splrep(times, [x[1] for x in path], k=3, s=alpha)

    t_smoothed = []
    curr_time = 0
    while curr_time < times[-1]:
        t_smoothed.append(curr_time)
        curr_time += dt

    x_d = scipy.interpolate.splev(t_smoothed, spl_x, der=0)
    y_d = scipy.interpolate.splev(t_smoothed, spl_y, der=0)
    xd_d = scipy.interpolate.splev(t_smoothed, spl_x, der=1)
    yd_d = scipy.interpolate.splev(t_smoothed, spl_y, der=1)
    xdd_d = scipy.interpolate.splev(t_smoothed, spl_x, der=2)
    ydd_d = scipy.interpolate.splev(t_smoothed, spl_y, der=2)
    theta_d = np.arctan2(yd_d, xd_d)
    traj_smoothed = np.stack([x_d, y_d, theta_d, xd_d, yd_d, xdd_d, ydd_d]).transpose()

    return traj_smoothed, t_smoothed