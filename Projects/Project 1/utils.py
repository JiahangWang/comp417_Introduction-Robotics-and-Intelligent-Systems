import argparse
def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', type=str, default="shot.png")
    parser.add_argument('--step_size', type=int, default=10)
    parser.add_argument('--rrt_sampling_policy', type=str, default="uniform")

    parser.add_argument('--start_pos_x', type=int, default=100)
    parser.add_argument('--start_pos_y', type=int, default=630)
    parser.add_argument('--start_pos_theta', type=float, default=0)
    parser.add_argument('--target_pos_x', type=int, default=800)
    parser.add_argument('--target_pos_y', type=int, default=150)
    parser.add_argument('--target_pos_theta', type=float, default=0)

    parser.add_argument('--robot_length', type=int, default=25)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--visualize', type=int, default=1)

    args = parser.parse_args()
    return args
