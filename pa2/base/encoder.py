from classes import Maze

if __name__ == '__main__':

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--grid', type = str)

    args = parser.parse_args()

    maze = Maze(args.grid)

    maze.write_mdp(100000, -100000, -1, discount = 0.9)