import matplotlib.pyplot as plt
import numpy as np
from environment import WindyGridworld
from agent import Agent


if __name__ == '__main__':
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('--max_episodes', type=int, default=150)
    parser.add_argument('--nactions', type=int, default=4)
    parser.add_argument('--seed', type=int, default=7)
    my_type = lambda x : True if (x == 'True') else False
    parser.add_argument('--stochastic', type=my_type, default=False)
    args = parser.parse_args()

    algos = ['sarsa', 'e-sarsa', 'q-learn']

    algos = ['sarsa']

    rand = np.random.default_rng(11)

    y = np.zeros((args.max_episodes, ))
    x = np.zeros((args.max_episodes, ))
    for algo in algos:
        for _ in range(10):
            seed = rand.integers(10000)
            grid = WindyGridworld(7, 10, (3,0), (3, 7), winds = [0, 0, 0, 1, 1, 1, 2, 2, 1, 0], stochastic=args.stochastic, seed=seed)
            agent = Agent(environ = grid, nactions=args.nactions, algo=algo, seed= seed)
            results = agent.learn(args.max_episodes)
            _y = [j for i, j in results]
            _x = [i for i, j in results]
            x = x + _x
            y = _y 
        x = x / 10
        plt.plot(x, y)
    plt.legend(algos)
    plt.savefig('baselinek.png') 
    plt.show()