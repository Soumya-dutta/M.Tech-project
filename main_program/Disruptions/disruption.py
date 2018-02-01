import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sample_time = 0.1
tot_distance = 1000
max_vel, min_vel = 50, 0
max_acc, max_dec = 10, -10
leader_pos, leader_vel, time = [], [], []
leader_acc, agent_inds, time_completed = [], [], []
K1, K2, K3, time_val = 1, 1, 4.06, 0.0
D, ball_rad = 249, 0.15
fpout, fvout = open('Position.csv', 'w'), open('Velocity.csv', 'w')
faout = open('Input.csv', 'w')
total_system_pos, total_system_vel = {}, {}
total_system_acc = {}
number_of_trips, agent_tag = {}, {}
dist_to_switch = 600
N = 6
law1 = 0
leader_dict = {}

def main():
    disrupt_file = pd.read_csv('disruption.csv')
    disrupt_dist = disrupt_file['Disruption distance']
    disrupt_dur = disrupt_file['Duration']
    num_agents = int(np.ceil(tot_distance/dist_to_switch)*2)
    agent_inds = [str(x) for x in range(1, num_agents+1)]
    agent_leaders = agent_inds[-1:] + agent_inds[:-1]
    for x in range(len(agent_inds)):
        leader_dict[agent_inds[x]] = agent_leaders[x]
        number_of_trips[agent_inds[x]] = 0
    print(leader_dict)



main()
