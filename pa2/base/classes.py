import numpy as np
from pulp import LpVariable, LpProblem, LpMinimize
from pulp.apis import list_solvers, PULP_CBC_CMD
import matplotlib.pyplot as plt

class Planner:

    def __init__(self, mdpfile = None, algorithm = None):
        
        self.algo = algorithm
        temp = {
            'vi' : (self.vi, self.vi_load),
            'hpi' : (self.hpi, self.com_load),
            'lp' : (self.lp, self.com_load),
        }
        if not algorithm is None:
            self.plan, self.loader = temp[algorithm]
        # self.loader = self.com_load
        self.mdpfile = mdpfile
        if not mdpfile is None:
            self.loader(mdpfile)     

    def vi(self):
        iters = 1
        V_new = np.max(self.R + np.sum(self.gamma * (self.T * self.V), 2), 1)        
        while np.linalg.norm(V_new - self.V) > 0.0000001:
            self.V = V_new
            V_new = np.max(self.R + np.sum(self.gamma * (self.T * self.V), 2), 1)
            iters += 1
        self.P = np.argmax(self.R + np.sum(self.gamma * (self.T * self.V), 2), 1) #final anaswer
        return (iters, self.V, self.P)

    def hpi(self):
        #TODO: implement howard's policy iteration
        iters = 1
        sel1 = [i for i in range(self.nstates)]

        #init
        self.P = np.random.choice(self.nactions, (self.nstates, ))
        T = self.T[sel1, self.P, :]
        R = self.R[sel1, self.P, :]
        # self.V = np.linalg.lstsq(np.eye(self.nstates) - self.gamma * T, np.sum(T * R, 1), rcond = None)[0]
        # self.V = np.linalg.solve(np.eye(self.nstates) - self.gamma * T, np.sum(T * R, 1))
        self.V = np.linalg.pinv(np.eye(self.nstates) - self.gamma * T) @ np.sum(T * R, 1)
        self.Q = np.sum(self.R * self.T + self.gamma * (self.T * self.V), 2)
        P_new = np.argmax(self.Q, 1)

        while not np.all(P_new == self.P):
            # print(iters)
            self.P = P_new
            T = self.T[sel1, self.P, :]
            R = self.R[sel1, self.P, :]
            # self.V = np.linalg.lstsq(np.eye(self.nstates) - self.gamma * T, np.sum(T * R, 1), rcond = None)[0]
            # self.V = np.linalg.solve(np.eye(self.nstates) - self.gamma * T, np.sum(T * R, 1))
            self.V = np.linalg.pinv(np.eye(self.nstates) - self.gamma * T) @ np.sum(T * R, 1)
            # if(np.all(V_new == self.V)): break
            # self.V = V_new
            self.Q = np.sum(self.R * self.T + self.gamma * (self.T * self.V), 2)
            P_new = np.argmax(self.Q, 1)

            iters += 1
        return (iters, self.V, self.P)

    def lp(self):

        #TODO: use pulp for linear programming
        prob = LpProblem('mdp_planner', LpMinimize)
        V_vars = np.array([LpVariable('V%d'%i, None, None) for i in range(self.nstates)])
        
        #Objective Function
        prob += np.sum(V_vars), "Objective Function"

        #Constraints
        mat = np.sum(self.T * self.R + self.gamma * (self.T * V_vars), 2)
        for i in range(self.nstates):
            for j in range(self.nactions):
                prob += mat[i, j] <= V_vars[i], 'constraint%d_%d'%(i,j) 
        
        # print(prob)
        prob.solve(PULP_CBC_CMD(msg = 0))
        self.V = np.array([i.varValue for i in V_vars]) + 0.0
        self.P = np.argmax(np.sum(self.R * self.T + self.gamma * (self.T * self.V), 2), 1)
        return (V_vars, self.V, self.P)

    def gen_mps(self):
        prob = LpProblem('mdp_planner', LpMinimize)
        V_vars = np.array([LpVariable('V%d'%i, None, None) for i in range(self.nstates)])
        
        #Objective Function
        prob += np.sum(V_vars), "Objective Function"

        #Constraints
        mat = np.sum(self.T * self.R + self.gamma * (self.T * V_vars), 2)
        for i in range(self.nstates):
            for j in range(self.nactions):
                prob += mat[i, j] <= V_vars[i], 'constraint%d_%d'%(i,j) 
        print(prob)
        prob.writeMPS('my_prob.mps')
    
    def vi_load(self, mdpfile = None):
        if not mdpfile is None:
            file = open(mdpfile)
            lines = [line for line in file]
            self.nstates = int(lines[0].split(' ')[-1])
            self.nactions = int(lines[1].split(' ')[-1])
            self.statrt = [int(i) for i in lines[2].split(' ')[1:]]
            self.end = [int(i) for i in lines[3].split(' ')[1:]]
            self.T = np.zeros((self.nstates, self.nactions, self.nstates))
            self.R = np.zeros((self.nstates, self.nactions))
            
            for i in lines[4:-2]:
                j = i.split(' ')[1:]
                j = [int(j[0]), int(j[1]), int(j[2]), float(j[3]), float(j[4])]
                self.R[j[0], j[1]] += j[3] * j[4]
                self.T[j[0], j[1], j[2]]  = j[4]

            self.type = lines[-2].split(' ')[-1]
            self.gamma = float(lines[-1].split(' ')[-1])
            
            self.P = np.zeros((self.nstates), int)
            self.V = np.zeros((self.nstates))
            self.Q = np.zeros((self.nstates))

        else:
            print('no mdpfile provided')     

    def hpi_load(self, mdpfile = None):
        pass

    def lp_load(self, mdpfile = None):
        pass

    def com_load(self, mdpfile = None):
        if not mdpfile is None:
            file = open(mdpfile)
            lines = [line for line in file]
            self.nstates = int(lines[0].split(' ')[-1])
            self.nactions = int(lines[1].split(' ')[-1])
            self.statrt = [int(i) for i in lines[2].split(' ')[1:]]
            self.end = [int(i) for i in lines[3].split(' ')[1:]]
            self.T = np.zeros((self.nstates, self.nactions, self.nstates))
            self.R = np.zeros((self.nstates, self.nactions, self.nstates))
            
            for i in lines[4:-2]:
                j = i.split(' ')[1:]
                j = [int(j[0]), int(j[1]), int(j[2]), float(j[3]), float(j[4])]
                self.T[j[0], j[1], j[2]]  = j[4]
                self.R[j[0], j[1], j[2]]  = j[3]

            self.type = lines[-2].split(' ')[-1]
            self.gamma = float(lines[-1].split(' ')[-1])
            
            self.P = np.zeros((self.nstates), int)
            self.V = np.zeros((self.nstates))
            self.Q = np.zeros((self.nstates))
        else:
            print('no mdpfile provided')     

class Maze:

    def __init__(self, mazefile):
        pass

        if not mazefile is None:
            self.grid = np.loadtxt(mazefile, int)[1:-1, 1:-1]
            self.rows, self.cols = self.grid.shape 
            self.actions = [(0, -1), (-1, 0), (0, 1), (1, 0)]
            self.start = [i for i in zip(*np.where(self.grid == 2))]
            self.end = [i for i in zip(*np.where(self.grid == 3))]
            self.action_map = {
                0 : 'W',
                1 : 'N',
                2 : 'E',
                3 : 'S'
            }
            self.raction_map = {
                'W' : 0,
                'N' : 1,
                'E' : 2,
                'S' : 3
            }
            self.nstates = 0
            self.encode = {}
            self.decode = {}

            for i in range(self.rows):
                for j in range(self.cols):
                    if(self.grid[i,j] == 1): continue 
                    self.encode[(i,j)] = self.nstates
                    self.decode[self.nstates] = (i,j)
                    self.nstates += 1
            self.estart = [self.encode[i] for i in self.start]
            self.eend = [self.encode[i] for i in self.end]

    def to_mdp(self):
        # encoding - i,j = (i*cols + col)
        print('numStates', self.nstates)
        print('numActions', 4)
        
        print('start', ' '.join([str(i) for i in self.estart]))
        print('end', ' '.join([str(i) for i in self.eend]))

        transitions = []

        for x in range(self.rows):
            for y in range(self.cols):
                if(self.grid[x,y] == 1): continue
                if((x,y) in self.end): continue
                idx = self.encode[(x,y)]
                for i in range(4):
                    a = self.actions[i]
                    app = 'transition '
                    app += '%d %d ' %(idx, i)
                    x1, y1 = x + a[0], y + a[1]

                    if x1<0 or x1>=self.rows or y1<0 or y1>=self.cols:
                        app += '%d -1 1' %self.eend[0]
                        continue
                    if self.grid[x1,y1] == 1:
                        app += '%d -1 1' %self.eend[0]
                        continue
                    idx1 = self.encode[(x1,y1)]
                    
                    if idx1 in self.eend:
                        app += '%d 10000 1' %idx1
                    else:
                        app += '%d -1 1' %idx1
                    
                    transitions.append(app)

        print('\n'.join(transitions))
        print('mdptype episodic')
        print('discount 1')

    def path_from_policy(self, value_policy):

        policy = np.loadtxt(value_policy, float)
        e_actions = policy[:, 1].astype(int)
        path = np.empty(self.nstates, dtype = str)
        vis = np.zeros(self.nstates , dtype = bool)
        nxt = [self.start[0]]
        while nxt:
            v = nxt.pop()
            ev = self.encode[v]
            vis[ev] = True
            if v in self.end: break
            a = self.actions[e_actions[ev]]
            x, y = v[0] + a[0], v[1] + a[1]
            ch = self.encode[(x,y)]
            if x < 0 or x >= self.rows or y < 0 or y >= self.cols: continue
            if self.grid[x, y] == 1: continue
            if vis[ch]: continue
            path[ch] = self.action_map[e_actions[ev]]
            nxt.append((x,y))

        if not vis[self.eend[0]]: return 'invlaid poicy'

        ret = []
        v = self.end[0]
        while v != self.start[0]:
            ev = self.encode[v]
            ret.append(path[ev])
            a = self.actions[self.raction_map[path[ev]]]
            v = (v[0] - a[0], v[1] - a[1])
        
        return ' '.join(ret)[::-1]

## Inefficient Maze class
# class Maze:
#     def __init__(self, mazefile = None):
#         if not mazefile is None:
#             self.grid = np.loadtxt(mazefile, int)[1:-1, 1:-1]
#             self.rows, self.cols = self.grid.shape 
#             self.actions = [(0, -1), (-1, 0), (0, 1), (1, 0)]
#             self.nstates = self.rows * self.cols + 1
#             self.start = [i for i in zip(*np.where(self.grid == 2))]
#             self.end = [i for i in zip(*np.where(self.grid == 3))]
#             self.estart = [i for i in map(self.sencode, self.start)]
#             self.eend = [i for i in map(self.sencode, self.end)]
#             self.action_map = {
#                 0 : 'W',
#                 1 : 'N',
#                 2 : 'E',
#                 3 : 'S'
#             }
#             self.raction_map = {
#                 'W' : 0,
#                 'N' : 1,
#                 'E' : 2,
#                 'S' : 3
#             }

#     def sencode(self, idx):
#         return idx[0] * self.cols + idx[1]

#     def sdecode(self, idx):
#         return ( (idx - (idx % self.cols)) // self.cols, idx % self.cols )
    
#     def tencode(self, idx):
#         ret = []
#         x, y = self.sdecode(idx)
#         # if(self.grid[x,y] == 1): return 'transition %d 0 %d -1 1\n' %(idx, idx)
#         if(self.grid[x,y] == 1): return ''
#         if(idx in self.eend): return ''
#         for i in range(4):
#             a = self.actions[i]
#             app = 'transition '
#             app += '%d %d ' %(idx, i)

#             x1, y1 = x + a[0], y + a[1]
#             idx1 = self.sencode((x1,y1))
#             if x1<0 or x1>=self.rows or y1<0 or y1>=self.cols:
#                 app += '%d -1 1' %(self.nstates - 1)
#             elif self.grid[x1,y1] == 1:
#                 app += '%d -1 1' %idx1
#             elif idx1 in self.eend:
#                 app += '%d 10000 1' %idx1
#             else:
#                 # app += '%d %d 1' %(self.sencode((x1,y1)), - (abs(x1 - self.end[0][0]) + abs(y1 - self.end[0][1])) / 100)
#                 app += '%d -1 1' %(idx1)
            
#             ret.append(app)
#         return ('\n'.join(ret)) + '\n'

#     def to_mdp(self):

#         print('numStates', self.nstates)
#         print('numActions', 4)
        
#         print('start', ' '.join([str(i) for i in self.estart]))
#         print('end', ' '.join([str(i) for i in self.eend]))

#         transitions = ''.join(map(self.tencode, range(self.nstates - 1)))
#         print(transitions, end = '')

#         print('mdptype episodic')
#         print('discount 1')
        
#         return 1
    
#     def path_from_policy(self, value_policy):
#         policy = np.loadtxt(value_policy, float)
#         opt_actions = policy[:, 1].astype(int)
#         opt_actions = np.reshape(opt_actions[:-1], (self.rows, self.cols))
#         path = np.empty((self.rows, self.cols), dtype = str)
#         vis = np.zeros( (self.rows, self.cols) , dtype = bool)

#         nxt = [self.start[0]]

#         while nxt:
#             v = nxt.pop()
#             vis[v] = True
#             if v in self.end: break
#             a = self.actions[opt_actions[v]] 
#             x, y = v[0] + a[0], v[1] + a[1]
#             if x < 0 or x >= self.rows or y < 0 or y >= self.cols: continue
#             if self.grid[x, y] == 1: continue
#             if vis[x, y]: continue
#             path[(x,y)] = self.action_map[opt_actions[v]]
#             nxt.append((x,y))

#         if not vis[self.end[0]]: return 'invlaid poicy'

#         ret = []
#         v = self.end[0]
#         while v != self.start[0]:
#             print(1)
#             ret.append(path[v])
#             a = self.actions[self.raction_map[path[v]]]
#             v = (v[0] - a[0], v[1] - a[1])
        
#         return ' '.join(ret)[::-1]