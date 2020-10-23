from classes import Maze

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--grid', type = str)
    parser.add_argument('--value_policy', type = str)

    args = parser.parse_args()

    maze = Maze(args.grid)
    answer = maze.path_from_policy(args.value_policy)

    print(answer)

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