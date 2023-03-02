from __future__ import print_function

import os
import sys
cwd = os.getcwd()
sys.path.append(cwd)

import numpy as np
import argparse
import random

from motion.motion_planners.meta import solve
from motion.motion_planners.trajectory.linear import solve_multi_linear
from motion.motion_planners.trajectory.discretize import distance_discretize_curve, V_MAX, A_MAX
from scripts.samplers import get_sample_fn, get_collision_fn, get_extend_fn, get_distance_fn, wrap_collision_fn, wrap_sample_fn
from motion.motion_planners.primitives import get_difference_fn, get_duration_fn
from motion.motion_planners.trajectory.smooth import smooth_curve, get_curve_collision_fn
from motion.motion_planners.trajectory.limits import analyze_continuity
from motion.motion_planners.utils import user_input, profiler, INF, compute_path_cost, remove_redundant, \
    waypoints_from_path

from motion.motion_planners.smoothing import smooth_path

from simulator.render import CV2Renderer
from simulator.envs import NavigationEnv

def retime_path(path, collision_fn=lambda q: False, smooth=False, **kwargs):
    # d = len(path[0])
    # v_max = 5.*np.ones(d)
    # a_max = v_max / 1.
    v_max, a_max = V_MAX, A_MAX
    print('Max vel: {} | Max accel: {}'.format(v_max, a_max))

    waypoints = remove_redundant(path)
    waypoints = waypoints_from_path(waypoints)
    positions_curve = solve_multi_linear(waypoints, v_max, a_max)
    if not smooth:
        return positions_curve

    print('Position: t={:.3f}, error={:.3E}'.format(*analyze_continuity(positions_curve)))
    print('Velocity: t={:.3f}, error={:.3E}'.format(*analyze_continuity(positions_curve.derivative(nu=1))))
    print('Acceleration: t={:.3f}, error={:.3E}'.format(*analyze_continuity(positions_curve.derivative(nu=2))))

    curve_collision_fn = get_curve_collision_fn(collision_fn, max_velocities=v_max, max_accelerations=a_max)
    positions_curve = smooth_curve(positions_curve,
                                   #v_max=None, a_max=None,
                                   v_max=v_max,
                                   a_max=a_max,
                                   curve_collision_fn=curve_collision_fn, **kwargs)
    print('Position: t={:.3f}, error={:.3E}'.format(*analyze_continuity(positions_curve)))
    print('Velocity: t={:.3f}, error={:.3E}'.format(*analyze_continuity(positions_curve.derivative(nu=1))))
    print('Acceleration: t={:.3f}, error={:.3E}'.format(*analyze_continuity(positions_curve.derivative(nu=2))))

    return positions_curve


OBJ_INFO = {
    'obstacle':{
        'box_1': {'shape': 'box', 'pose': (3.5, 7.5, 0.5, 0, 0, 0, 1), 'size': (0.3, 0.3, 0.5), 'rgba': [0.8, 0.8, 0.8, 1]},
        'box_2': {'shape': 'box', 'pose': (7.5, 3.5, 0.5, 0, 0, 0, 1), 'size': (0.2, 0.2, 0.5), 'rgba': [0.8, 0.8, 0.8, 1]},
        'box_3': {'shape': 'box', 'pose': (5.0,  5.0,  0.5, 0, 0, 0, 1), 'size': (0.3, 0.3, 0.5), 'rgba': [0.8, 0.8, 0.8, 1]},
        'cylinder_1': {'shape': 'cylinder', 'pose': (2.5, 2.5, 0.5, 0, 0, 0, 1), 'size': (0.2, 0.5), 'rgba': [0.8, 0.8, 0.8, 1]},
        },
    'goal':{
        'box_goal': {'shape': 'box', 'pose': (10*0.8, 10*0.8, 0.1, 0, 0, 0, 1), 'size': (5*0.1, 5*0.1, 0.1), 'rgba': [0.8, 0.0, 0.0, 0.1]},
        },
    'start':{
        'box_start': {'shape': 'box', 'pose': (10*0.1, 10*0.1, 0.1, 0, 0, 0, 1), 'size': (5*0.1, 5*0.1, 0.1), 'rgba': [0.0, 0.8, 0.0, 0.1]},
        }
}

##################################################

def main(draw=True):
    """
    Creates and solves the 2D motion planning problem.
    """
    # https://github.com/caelan/pddlstream/blob/master/examples/motion/run.py
    # TODO: 3D workspace and CSpace
    # TODO: visualize just the tool frame of an end effector

    np.set_printoptions(precision=3)
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--algorithm', default='rrt_connect',
                        help='The algorithm to use.')
    parser.add_argument('--anytime', action='store_true',
                        help='Run the planner in an anytime mode.')
    parser.add_argument('-d', '--draw', action='store_true',
                        help='When enabled, draws the roadmap')
    parser.add_argument('-r', '--restarts', default=3, type=int,
                        help='The number of restarts.')
    parser.add_argument('-s', '--smooth', action='store_true', default=1,
                        help='When enabled, smooths paths.')
    parser.add_argument('-t', '--time', default=1., type=float,
                        help='The maximum runtime.')
    parser.add_argument('--seed', default=364, type=int,
                        help='The random seed to use.')
    args = parser.parse_args()
    print(args)

    seed = args.seed
    if seed is None:
        #seed = random.randint(0, sys.maxsize)
        seed = random.randint(0, 10**3-1)
    print('Seed:', seed)
    random.seed(seed)
    np.random.seed(seed)

    #########################
    start = np.array([1. , 1.])
    goal = np.array([8., 8.])

    env = NavigationEnv(OBJ_INFO)
    renderer = CV2Renderer(device_id=-1, sim=env.sim, cam_name='birdview')
    recorder = None
    env.set_renderer(renderer)
    env.set_recorder(recorder)
    env.reset()

    env.step(start)

    conf_ids = env.conf_ids
    region = env.conf_region
    geom_info = env.geom_info

    #########################
    weights = np.reciprocal(V_MAX)
    distance_fn = get_distance_fn(weights=[1, 1]) # distance_fn
    min_distance = distance_fn(start, goal)
    print('Distance: {:.3f}'.format(min_distance))

    with profiler(field='tottime'): # cumtime | tottime
        # TODO: cost bound & best cost
        collision_fn, colliding, cfree = wrap_collision_fn(get_collision_fn(conf_region=region, conf_ids=conf_ids, geom_info=geom_info, sim=env.sim))
        sample_fn, samples = wrap_sample_fn(get_sample_fn(conf_region=region, conf_ids=conf_ids, geom_info=geom_info, sim=env.sim, use_halton=True)) # obstacles

        circular = {}
        extend_fn, roadmap = get_extend_fn(circular=circular), []

        cost_fn = get_duration_fn(difference_fn=get_difference_fn(circular=circular), v_max=V_MAX, a_max=A_MAX)
        path = solve(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                        cost_fn=cost_fn, weights=weights, circular=circular,
                        max_time=args.time, max_iterations=INF, num_samples=100,
                        success_cost=0 if args.anytime else INF,
                        restarts=2, smooth=0, algorithm=args.algorithm, verbose=True)

    cost = compute_path_cost(path, cost_fn)
    print('Length: {} | Cost: {:.3f} | Ratio: {:.3f}'.format(len(path), cost, cost/min_distance))
    path = waypoints_from_path(path)

    #curve = interpolate_path(path) # , collision_fn=collision_fn)
    curve = retime_path(path, collision_fn=collision_fn, smooth=args.smooth,
                        max_time=args.time) # , smooth=True)

    times, path = distance_discretize_curve(curve)
    times = [np.linalg.norm(curve(t, nu=1), ord=INF) for t in times]

    env.reset()
    for point in path:
        env.step(point)
        # time.sleep(0.01)


    if args.smooth:
        smoothed = smooth_path(path, extend_fn, collision_fn,
                                cost_fn=cost_fn, sample_fn=sample_fn,
                                max_iterations=INF, max_time=args.time,
                                converge_time=INF, verbose=True)
        print('Smoothed distance_fn: {:.3f}'.format(compute_path_cost(smoothed, distance_fn)))
        env.reset()
        for point in smoothed:
            env.step(point)
            # time.sleep(0.01)

    user_input('Finish?')

if __name__ == '__main__':
    main()
