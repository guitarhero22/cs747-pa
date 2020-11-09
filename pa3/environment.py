import numpy as np
import matplotlib.pyplot as plt

class WindyGridworld:
    
    def __init__(self, rows, cols, 
                start = None, end = None,  nstart = 1, nend = 1, 
                winds = None, max_wind = 2,
                gamma = 1, stochastic = False,
                seed = 0):
        
        """
        rows        number of rows
        cols        number of columns
        start       list of starting states
        goal        list of goal states
        actions     list of actions
        type
        stochastic

        methods:
        """

        self.dirs = ['N', 'S', 'E', 'W', 'NW', 'NE', 'SE', 'SW', 'X']
        self.add = [(1,0), (-1,0), (0,1), (0,-1), (1,-1), (1,1), (-1,1), (-1,-1), (0,0)]
        self.map = {self.dirs[i] : self.add[i] for i in range(len(self.dirs))} 
        self.rows, self.cols = rows, cols
        self.stochastic = stochastic
        self.gamma = gamma

        self.rand = np.random.default_rng(seed)
        
        if not start is None:
            if not isinstance(start, list): start = [start]
            self.start = start
        else:
            self.start = [(i,j) for i,j in zip(self.rand.choice(np.arange(self.rows, dtype = int), (nstart, )), 
                                            self.rand.choice(np.arange(self.cols, dtype = int), (nstart, )))]
        
        if not end is None:
            if not isinstance(end, list): end = [end]
            self.end = end
        else:
            self.end = [(i,j) for i,j in zip(self.rand.choice(np.arange(self.rows, dtype = int), (nend, )), 
                                            self.rand.choice(np.arange(self.cols, dtype = int), (nend, )))]
         
        if not winds is None:
            self.winds = winds
        else:
            self.winds = [i for i in self.rand.choice(np.arange(max_wind, dtype = int), (self.cols, ))]
            self.winds[0] = 0
            self.winds[-1] = 0
        if stochastic:
            self.act = self.act_stochastic
        else:
            self.act = self.act_deterministic
        return


    def visualize(self):

        sz = 1
        plt.axes()

        for i in range(self.rows + 1):
            i = i * sz
            line = plt.Line2D((0,self.cols * sz), (i, i), lw = 2.5)
            plt.gca().add_line(line)
       
        for i in range(self.cols + 1):
            i = i * sz
            line = plt.Line2D((i,i), (0, self.rows * sz), lw = 2.5)
            plt.gca().add_line(line)

        for i in self.start:
            i = (i[0]*sz, i[1]*sz)
            patch = plt.Rectangle((i[1], i[0]), sz, sz, fc='g')
            plt.gca().add_patch(patch)

        for i in self.end:
            i = (i[0]*sz, i[1]*sz)
            patch = plt.Rectangle((i[1], i[0]), sz, sz, fc='r')
            plt.gca().add_patch(patch)
        
        plt.xticks([i*sz + sz*0.5  for i in range(self.cols)],self.winds)
        plt.gca().tick_params(axis = u'both', which = u'both', length = 0)
        plt.yticks([],[])
        plt.axis('scaled')
        plt.show()

    def act_stochastic(self):
        pass

    def act_deterministic(self, curpos, action):
        
        if isinstance(action, str):
            action = self.map[action]
        else:
            action = self.add[action]

        r, c = curpos[0] + action[0] + self.winds[curpos[1]], curpos[1] + action[1]

        if r < 0:
            r = 0
        elif r >= self.rows:
            r = self.rows - 1

        if c < 0:
            c = 0
        elif c >= self.cols:
            c = self.cols - 1
       
        reward = -1
        if (r,c) in self.end:
            reward = 1

        return reward, (r,c) 

if __name__ == '__main__':
    
    grid = WindyGridworld(5, 4, )

