import numpy, openravepy
import pylab as pl
import math
from DiscreteEnvironment import DiscreteEnvironment

class Control(object):
    def __init__(self, omega_left, omega_right, duration):
        self.ul = omega_left
        self.ur = omega_right
        self.dt = duration

class Action(object):
    def __init__(self, control, footprint):
        self.control = control
        self.footprint = footprint

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.herb = herb
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5., -numpy.pi], [5., 5., numpy.pi]]
        lower_limits, upper_limits = self.boundary_limits
        self.discrete_env = DiscreteEnvironment(resolution, lower_limits, upper_limits)

        self.resolution = resolution
        self.ConstructActions()

    def GenerateFootprintFromControl(self, start_config, control, stepsize=0.01):

        # Extract the elements of the control
        ul = control.ul
        ur = control.ur
        dt = control.dt

        # Initialize the footprint
        config = start_config.copy()
        footprint = [numpy.array([0., 0., config[2]])]
        timecount = 0.0
        while timecount < dt:
            # Generate the velocities based on the forward model
            xdot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.cos(config[2])
            ydot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.sin(config[2])
            tdot = self.herb.wheel_radius * (ul - ur) / self.herb.wheel_distance
                
            # Feed forward the velocities
            if timecount + stepsize > dt:
                stepsize = dt - timecount
            config = config + stepsize*numpy.array([xdot, ydot, tdot])
            if config[2] > numpy.pi:
                config[2] -= 2.*numpy.pi
            if config[2] < -numpy.pi:
                config[2] += 2.*numpy.pi

            footprint_config = config.copy()
            footprint_config[:2] -= start_config[:2]
            footprint.append(footprint_config)

            timecount += stepsize
            
        # Add one more config that snaps the last point in the footprint to the center of the cell
        nid = self.discrete_env.ConfigurationToNodeId(config)
        snapped_config = self.discrete_env.NodeIdToConfiguration(nid)
        snapped_config[:2] -= start_config[:2]
        footprint.append(snapped_config)

        return footprint

    def PlotActionFootprints(self, idx):

        actions = self.actions[idx]
        fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        
        for action in actions:
            xpoints = [config[0] for config in action.footprint]
            ypoints = [config[1] for config in action.footprint]
            pl.plot(xpoints, ypoints, 'k')
                     
        pl.ion()
        pl.show()

        
    def ConstructActions(self):

        # Actions is a dictionary that maps orientation of the robot to
        #  an action set
        self.actions = dict()
              
        wc = [0., 0., 0.]
        grid_coordinate = self.discrete_env.ConfigurationToGridCoord(wc)

        # Iterate through each possible starting orientation
        for idx in range(int(self.discrete_env.num_cells[2])):
            self.actions[idx] = []
            grid_coordinate[2] = idx
            start_config = self.discrete_env.GridCoordToConfiguration(grid_coordinate)

            # TODO: Here you will construct a set of actions
            #  to be used during the planning process
            #
            wl_max = 1;
            wr_max = 1;
            t_l = 2
            t_s = 1
            t_semi = 5*math.pi/8
            t_cur = 5*math.pi/2
            #turn left in place
            C_1 = Control(-wl_max,wr_max,t_semi)
            A_1 = Action(C_1,GenerateFootprintFromControl(start_config, C_1))
            #Turn right in place
            C_2 = Control(wl_max,-wr_max,t_semi)
            A_2 = Action(C_2,GenerateFootprintFromControl(start_config, C_2))
            #go front long
            C_3 = Control(wl_max,wr_max,t_l)
            A_3 = Action(C_3,GenerateFootprintFromControl(start_config, C_3))
            # GO Back long
            C_4 = Control(-wl_max,-wr_max,t_l)
            A_4 = Action(C_4,GenerateFootprintFromControl(start_config, C_4))
            # go front small
            C_5 = Control(wl_max,wr_max,t_s)
            A_5 = Action(C_5,GenerateFootprintFromControl(start_config, C_5))
            # go back small
            C_6 = Control(-wl_max,-wr_max,t_s)
            A_6 = Action(C_6,GenerateFootprintFromControl(start_config, C_6))
            # curve left
            C_7 = Control(0.5*wl_max,wr_max,t_cur)
            A_7 = Action(C_7,GenerateFootprintFromControl(start_config, C_7))
            # curve right
            C_8 = Control(wl_max,0.5*wr_max,t_cur)
            A_8 = Action(C_8,GenerateFootprintFromControl(start_config, C_8))
            # curve back left
            C_9 = Control(-0.5*wl_max,-wr_max,t_cur)
            A_9 = Action(C_9,GenerateFootprintFromControl(start_config, C_9))
            # curve back right
            C_10 = Control(-wl_max,-0.5*wr_max,t_cur)
            A_10 = Action(C_10,GenerateFootprintFromControl(start_config, C_10))
            self.Actions[idx] = [A_1,A_2,A_3,A_4,A_5,A_6,A_7,A_8,A_9,A_10]
            
            PlotActionFootprints(0)

    def GetSuccessors(self, node_id):

        successors = []
        
        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids and controls that represent the neighboring
        #  nodes
        
        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        
        
        return cost

