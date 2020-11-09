import numpy as np
import matplotlib.pyplot as plt
from environment import WindyGridworld

class Agent:
    def __init__(self, epsilon = 0.1, alpha = 0.5, algo = 'sarsa', environ = None, nactions = 4, seed = 0):
        """
        t                   to track time steps
        results             for the plot
        state            tuple for current state
        Q(s,a)              3D numpy mat for Q table
        max_episodes        for max number of episodes to run for
        epsilon             parameter
        alpha               parameter
        algo                algorithm to use
        learn               function that starts exploring
        """
        self.t = 0
        if environ is None:
            print('set the environment before proceeding')
            return
        else:
            self.environ = environ

        # Initialize for learning
        self.nactions = nactions
        self.actions = self.environ.dirs[:nactions]
        self.Q = np.zeros((environ.rows, environ.cols, len(self.actions)))
        self.epsilon = epsilon
        self.alpha =  alpha
        self.t = 0

        algos = {
            'sarsa' : self.sarsa,
            'e-sarsa' : self.e_sarsa,
            'q-learn' : self.q_learn
        }
        self.target = algos[algo]
        self.rand = np.random.default_rng(seed)
    
    def learn(self, max_episodes = 150):
        self.results = []
        st = self.environ.start[0]
        episodes = 0
        while episodes < max_episodes:                
            
            if self.rand.random() <= self.epsilon:
                act = self.rand.choice(self.nactions)
            else: act = np.argmax(self.Q[st[0], st[1]])
            if self.rand.random() <= self.epsilon:
                nact = self.rand.choice(self.nactions)
            else: nact = np.argmax(self.Q[st[0], st[1]])
            rew, nst = self.environ.act(st, act)
            self.Q[st[0], st[1], act] += self.alpha * ( rew + self.target(nst, nact) - self.Q[st[0], st[1], act])
            self.t += 1
            st = nst
            if rew == 1:
                episodes += 1
                st = self.environ.start[0]
                print(episodes)
                self.results.append((self.t, episodes))
            # if self.t % 100 == 0:
                # print(st, self.t, self.actions[act])
        return self.results

    def sarsa(self, nst, nact):
        return self.environ.gamma * self.Q[nst[0], nst[1], nact] 

    def e_sarsa(self):
        pass

    def q_learn(self, nst, nact):
        return self.environ.gamma * np.max(self.Q[nst[0], nst[1]])

if __name__ == '__main__':
    import  argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--algo', type = str, default = 'sarsa')
    parser.add_argument('--nactions', type = int, default = 4)
    parser.add_argument('--stochastic', type = bool, default = False)
    args = parser.parse_args()
    grid = WindyGridworld(7, 10, (3,0), (3, 7), winds = [0, 0, 0, 1, 1, 1, 2, 2, 1, 0], stochastic=args.stochastic)
    grid.visualize()
    # exit()
    agent = Agent(environ = grid, nactions = args.nactions, algo = args.algo)
    results = agent.learn(500)
    y = [j for i,j in results]
    x = [i for i,j in results]

    plt.plot(x, y)
    plt.show()