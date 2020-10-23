import numpy as np
from pulp import LpVariable, LpProblem, LpMinimize, lpSum
from pulp.apis import list_solvers, PULP_CBC_CMD
import matplotlib.pyplot as plt

class Planner:

    def __init__(self, mdpfile = None, algorithm = None):
        
        solvers = {
            'vi' : self.vi_solver,
            'hpi' : self.hpi_solver,
            'lp' : self.lp_solver,
        }
        self.plan = solvers[algorithm]
        self.load_mdp(mdpfile, algorithm)   
        self.states = [i for i in range(self.nstates)]  

    def vi_solver(self):
        iters = 1
        V_new = np.max(self.R + np.sum(self.gamma * (self.T * self.V), 2), 1)        
        while np.linalg.norm(V_new - self.V) > 0.0000001:
            self.V = V_new
            V_new = np.max(self.R + np.sum(self.gamma * (self.T * self.V), 2), 1)
            iters += 1
        self.P = np.argmax(self.R + np.sum(self.gamma * (self.T * self.V), 2), 1)
        return (iters, self.V, self.P)
    
    def solve_V(self, P, method = 'vi', tol = 0.000001):

        T = self.T[self.states, self.P, :]
        R = self.R[self.states, self.P, :]

        if method == 'vi':
            old_V = np.zeros((self.nstates, ))
            self.V = (np.sum(T * R, 1) +  self.gamma * np.dot(T, self.V))

            while np.linalg.norm(self.V - old_V) > tol:
                old_V = self.V
                self.V = (np.sum(T * R, 1) +  self.gamma * np.dot(T, self.V))

            return

        if method == 'pinv':
            self.V = np.linalg.pinv(np.eye(self.nstates) - self.gamma * T) @ np.sum(T * R, 1)
            return

        if method == 'lstsq':
            return

    def hpi_solver(self, method = 'vi'):

        iters = 1
        self.P = np.random.choice(self.nactions, (self.nstates, ))

        self.solve_V(self.P, method)
        self.Q = np.sum(self.R * self.T + self.gamma * (self.T * self.V), 2)
        P_new = np.argmax(self.Q, 1)

        while not np.all(P_new == self.P):
            self.P = P_new
            self.solve_V(self.P, method)
            self.Q = np.sum(self.R * self.T + self.gamma * (self.T * self.V), 2)
            P_new = np.argmax(self.Q, 1)
            iters += 1

        self.solve_V(self.P, tol = 0.0000001)
        return (iters, self.V, self.P)
    

    def lp_solver(self, use_dict = True):

        prob = LpProblem('mdp_planner', LpMinimize)
        V_vars = np.array([LpVariable('V%d'%i, None, None) for i in range(self.nstates)])
        
        prob += lpSum(V_vars), "Objective Function"
        if use_dict:
            for i, j in self.R_d.keys():
                prob += V_vars[i] >= lpSum([self.T_d[i,j][k] * (self.R_d[i,j][k] + self.gamma * V_vars[k]) for k in self.T_d[i,j].keys()]), 'constraint%d_%d'%(i,j)         
        else: 
            mat = np.sum(self.T * self.R + self.gamma * (self.T * V_vars), 2)
            for i in range(self.nstates):
                for j in range(self.nactions):
                    prob += mat[i, j] <= V_vars[i], 'constraint%d_%d'%(i,j) 
        
        prob.solve(PULP_CBC_CMD(msg = 0))

        self.V = np.array([i.varValue for i in V_vars]) + 0.0
        self.P = np.argmax(np.sum(self.R * self.T + self.gamma * (self.T * self.V), 2), 1)

        return (V_vars, self.V, self.P)
            
    def load_mdp(self, mdpfile = None, algorithm = None):
        if not mdpfile is None:
            file = open(mdpfile)
            lines = [line for line in file]
            self.nstates = int(lines[0].split(' ')[-1])
            self.nactions = int(lines[1].split(' ')[-1])
            self.statrt = [int(i) for i in lines[2].split(' ')[1:]]
            self.end = [int(i) for i in lines[3].split(' ')[1:]]
            self.T = np.zeros((self.nstates, self.nactions, self.nstates))
            
            if algorithm == 'vi':
                self.R = np.zeros((self.nstates, self.nactions))
                for i in lines[4:-2]:
                    j = i.split(' ')[1:]
                    j = [int(j[0]), int(j[1]), int(j[2]), float(j[3]), float(j[4])]
                    self.R[j[0], j[1]] += j[3] * j[4]
                    self.T[j[0], j[1], j[2]]  = j[4]

            elif algorithm == 'lp':
                self.R = np.zeros((self.nstates, self.nactions, self.nstates))
                self.R_d = {(i,j) : {} for i in range(self.nstates) for j in range(self.nactions)}
                self.T_d = {(i,j) : {} for i in range(self.nstates) for j in range(self.nactions)}
                for i in lines[4:-2]:
                    j = i.split(' ')[1:]
                    j = [int(j[0]), int(j[1]), int(j[2]), float(j[3]), float(j[4])]
                    self.R[j[0], j[1], j[2]]  = j[3]
                    self.T[j[0], j[1], j[2]]  = j[4]
                    self.R_d[(j[0], j[1])][j[2]] = j[3]
                    self.T_d[(j[0], j[1])][j[2]] = j[4]

            else:
                self.R = np.zeros((self.nstates, self.nactions, self.nstates))
                for i in lines[4:-2]:
                    j = i.split(' ')[1:]
                    j = [int(j[0]), int(j[1]), int(j[2]), float(j[3]), float(j[4])]
                    self.R[j[0], j[1], j[2]]  = j[3]
                    self.T[j[0], j[1], j[2]]  = j[4]

            self.type = lines[-2].split(' ')[-1]
            self.gamma = float(lines[-1].split(' ')[-1])
            
            self.P = np.zeros((self.nstates), int)
            self.V = np.zeros((self.nstates))
            self.Q = np.zeros((self.nstates))
            
            return

        print('no mdpfile provided')     

class Maze:

    def __init__(self, mazefile):
        if not mazefile is None:

            self.grid = np.loadtxt(mazefile, int)[1:-1, 1:-1]
            self.rows, self.cols = self.grid.shape
            self.start = [i for i in zip(*np.where(self.grid == 2))]
            self.end = [i for i in zip(*np.where(self.grid == 3))]
            
            self.actions = [(0, -1), (-1, 0), (0, 1), (1, 0)]
            self.action_map = {0 : 'W', 1 : 'N', 2 : 'E', 3 : 'S'}
            self.raction_map = {'W' : 0, 'N' : 1, 'E' : 2, 'S' : 3}

            self.nstates = 0
            self.encode = {}
            self.decode = {}

            for i in range(self.rows):
                for j in range(self.cols):
                    if(self.grid[i,j] == 1): continue 
                    self.encode[(i,j)] = self.nstates
                    self.decode[self.nstates] = (i,j)
                    self.nstates += 1
            
            self.en_start = [self.encode[i] for i in self.start]
            self.en_end = [self.encode[i] for i in self.end]
            # print(self.start, self.end)

    def write_mdp(self, good_reward = 10000, bad_reward = -10000, normal_reward = -1, discount = 1):

        end_st = self.en_end[0]

        print('numStates', self.nstates)
        print('numActions', 4)
        
        print('start', ' '.join([str(i) for i in self.en_start]))
        print('end', ' '.join([str(i) for i in self.en_end]))

        transitions = []

        for x in range(self.rows):
            for y in range(self.cols):

                if(self.grid[x,y] == 1): continue #wall
                if(self.grid[x,y] == 3): continue #end state
                
                idx = self.encode[(x,y)]

                for i, a in zip(range(4), self.actions):

                    row = 'transition %d %d '%(idx, i)

                    x1, y1 = x + a[0], y + a[1]

                    if x1<0 or x1>=self.rows or y1<0 or y1>=self.cols: #leads out of the maze area
                        row += '%d %d 1' %(end_st, bad_reward)
                        continue

                    if self.grid[x1,y1] == 1: #leading into a wall
                        row += '%d %d 1' %(end_st, bad_reward)
                        continue
                    
                    idx1 = self.encode[(x1,y1)]
                    row += '%d '%idx1

                    if idx1 in self.en_end: #leading to final state
                        row += '%d 1' %good_reward
                    else: #leading to an empty cell
                        row += '%d 1' %normal_reward

                    transitions.append(row)

        print('\n'.join(transitions))

        print('mdptype episodic')

        print('discount %f' %discount)

    def path_from_policy(self, value_policy):

        policy = np.loadtxt(value_policy, float)
        policy = policy[:, 1].astype(int)
        path = np.empty(self.nstates, dtype = str)
        vis = np.zeros(self.nstates , dtype = bool)
        nxt = [self.start[0]]

        while nxt:

            v = nxt.pop()
            ev = self.encode[v]
            vis[ev] = True
            
            if ev in self.en_end: break
            a = self.actions[policy[ev]]
            x, y = v[0] + a[0], v[1] + a[1]
            ch = self.encode[(x,y)]
            
            if x < 0 or x >= self.rows or y < 0 or y >= self.cols: continue
            if self.grid[x, y] == 1: continue
            if vis[ch]: continue
            
            path[ch] = self.action_map[policy[ev]]
            nxt.append((x,y))

        found = False
        for e in self.en_end:
            if vis[e]: ans, found = e, True
        if not found: return 'invalid policys'
        # if not vis[self.en_end]: return 'invlaid poicy'
        
        ret = []
        ev = ans
        v = self.decode[ev]

        while not ev in self.en_start:
            ret.append(path[ev])
            a = self.actions[self.raction_map[path[ev]]]
            v = (v[0] - a[0], v[1] - a[1])
            ev = self.encode[v]
        
        return ' '.join(ret)[::-1]