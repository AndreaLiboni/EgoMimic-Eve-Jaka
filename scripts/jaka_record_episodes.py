#!/usr/bin/env python3
import threading
import argparse
import os
import time
import cv2
import h5py
import numpy as np
import rclpy
from tqdm import tqdm

from eve.jaka_real_env import RealEnvJaka
from eve.constants import (
    FPS,
    TASK_CONFIGS,
)
from eve.jaka_utils import ControllerSubscriber
from eve.jaka import JAKA


def capture_one_episode(
    max_timesteps,
    camera_names,
    dataset_dir,
    dataset_name,
    overwrite,
    env: RealEnvJaka,
    controller: ControllerSubscriber,
):
    print(f'Dataset name: {dataset_name}')

    # Saving dataset
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)
    dataset_path = os.path.join(dataset_dir, dataset_name)
    if os.path.isfile(dataset_path) and not overwrite:
        print(f'Dataset already exists at \n{dataset_path}\nHint: set overwrite to True.')
        exit()

    # Data collection
    # ts = env.reset()
    timesteps = []
    actions = []
    actual_dt_history = []
    time0 = time.time()
    DT = 1 / FPS
    # while controller.record:
    #     t0 = time.time()
    #     action = get_action(env.robot)
    #     t1 = time.time()
    #     ts = env.step(action)
    #     t2 = time.time()
    #     timesteps.append(ts)
    #     actions.append(action)
    #     actual_dt_history.append([t0, t1, t2])
    #     time.sleep(max(0, DT - (time.time() - t0)))
    #     max_timesteps += 1
    for t in tqdm(range(max_timesteps)):
        if not controller.record:
            max_timesteps = t
            break
        t0 = time.time()
        ts = env.step(
            action=None,
            get_obs=True,
        )
        t1 = time.time()
        action = ts.observation['qpos']
        t2 = time.time()
        timesteps.append(ts)
        actions.append(action)
        actual_dt_history.append([t0, t1, t2])
        # print(max(0, DT - (time.time() - t0)))
        time.sleep(max(0, DT - (time.time() - t0)))
    
    print(f'Avg fps: {max_timesteps / (time.time() - time0)}')

    freq_mean = print_dt_diagnosis(actual_dt_history)
    if freq_mean < 30:
        print(f'\n\nfreq_mean is {freq_mean}, lower than 30, re-collecting... \n\n\n\n')
        # return False

    """
    For each timestep:
    observations
    - images
        - cam_high          (480, 640, 3) 'uint8'
        - cam_low           (480, 640, 3) 'uint8'
        - cam_left_wrist    (480, 640, 3) 'uint8'
        - cam_right_wrist   (480, 640, 3) 'uint8'
    - qpos                  (14,)         'float64'
    - qvel                  (14,)         'float64'

    action                  (14,)         'float64'
    base_action             (2,)          'float64'
    """

    data_dict = {
        '/observations/qpos': [],
        '/action': [],
    }

    for cam_name in camera_names:
        data_dict[f'/observations/images/{cam_name}'] = []

    # len(action): max_timesteps, len(time_steps): max_timesteps + 1
    while actions:
        action = actions.pop(0)
        ts = timesteps.pop(0)
        data_dict['/observations/qpos'].append(ts.observation['qpos'])
        data_dict['/action'].append(action)
        
        for cam_name in camera_names:
            data_dict[f'/observations/images/{cam_name}'].append(
                ts.observation['images'][cam_name]
            )

    COMPRESS = True

    # Save to HDF5
    t0 = time.time()
    with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024**2*2) as root:
        root.attrs['sim'] = False
        root.attrs['compress'] = COMPRESS
        obs = root.create_group('observations')
        image = obs.create_group('images')
        for cam_name in camera_names:
            if COMPRESS:
                image.create_dataset(cam_name, (max_timesteps, 480, 640, 3), dtype='uint8',
                                          chunks=(1, 480, 640, 3), compression="gzip",  compression_opts=4)
            else:
                image.create_dataset(cam_name, (max_timesteps, 480, 640, 3), dtype='uint8',
                                         chunks=(1, 480, 640, 3))
        
        if COMPRESS:
            obs.create_dataset('qpos', (max_timesteps, 7), compression="gzip",  compression_opts=4)
            root.create_dataset('action', (max_timesteps, 7), compression="gzip",  compression_opts=4)
        else:
            obs.create_dataset('qpos', (max_timesteps, 7))
            root.create_dataset('action', (max_timesteps, 7))
        
        # obs.create_dataset('qvel', (max_timesteps, 7))
        # obs.create_dataset('effort', (max_timesteps, 7))

        for name, array in data_dict.items():
            print(name, np.array(array).shape)
            root[name][...] = array

    print(f'Saving: {time.time() - t0:.1f} secs')

    return True


def main(args: dict):
    task_config = TASK_CONFIGS[args['task_name']]
    dataset_dir = task_config['dataset_dir']
    max_timesteps = task_config['episode_len']
    camera_names = task_config['camera_names']
    tcp_id = task_config.get('tcp_id', None)

    # torque_base = args.get('enable_base_torque', False)

    if args['episode_idx'] is not None:
        episode_idx = args['episode_idx']
    else:
        episode_idx = get_auto_index(dataset_dir)
    overwrite = True

    dataset_name = f'episode_{episode_idx}'
    print(dataset_name + '\n')

    robot = JAKA(args['ip'])
    robot.frame_id = 0
    if tcp_id is not None:
        print(f'Setting TCP ID to {tcp_id}')
        robot.tool_id = tcp_id
    robot.setup_robot()
    # robot.move_to_start(block=True)
    robot.servo_mode()

    rclpy.init()
    env = RealEnvJaka(robot)

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    controller = ControllerSubscriber(robot)
    executor.add_node(controller)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    print("Robot control ready")
    while True:
        is_healthy = capture_one_episode(
            max_timesteps,
            camera_names,
            dataset_dir,
            dataset_name,
            overwrite,
            env,
            controller
        )
        if is_healthy:
            break
    
    executor.shutdown()
    rclpy.shutdown()


def get_auto_index(dataset_dir, dataset_name_prefix='', data_suffix='hdf5'):
    max_idx = 1000
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)
    for i in range(max_idx+1):
        if not os.path.isfile(
            os.path.join(dataset_dir, f'{dataset_name_prefix}episode_{i}.{data_suffix}')
        ):
            return i
    raise Exception(f'Error getting auto index, or more than {max_idx} episodes')


def print_dt_diagnosis(actual_dt_history):
    actual_dt_history = np.array(actual_dt_history)
    get_action_time = actual_dt_history[:, 1] - actual_dt_history[:, 0]
    step_env_time = actual_dt_history[:, 2] - actual_dt_history[:, 1]
    total_time = actual_dt_history[:, 2] - actual_dt_history[:, 0]

    dt_mean = np.mean(total_time)
    # dt_std = np.std(total_time)
    freq_mean = 1 / dt_mean
    print((
        f'Avg freq: {freq_mean:.2f} Get action: {np.mean(get_action_time):.3f} '
        f'Step env: {np.mean(step_env_time):.3f}')
    )
    return freq_mean


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--task_name',
        action='store',
        type=str,
        help='Task name.',
        required=True,
    )
    parser.add_argument(
        '--episode_idx',
        action='store',
        type=int,
        help='Episode index.',
        default=None,
        required=False,
    )
    parser.add_argument(
        '--ip',
        action='store',
        type=str,
        help='JAKA Robot IP address.',
        required=True,
    )
    main(vars(parser.parse_args()))
