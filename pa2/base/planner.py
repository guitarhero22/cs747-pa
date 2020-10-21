from classes import Planner

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()

    parser.add_argument('--mdp', type = str)
    parser.add_argument('--algorithm', type = str)

    args = parser.parse_args()
    planner = Planner(args.mdp, args.algorithm)
    out = planner.plan()

    for i,j in zip(out[1], out[2]):
        print(i,j)
