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