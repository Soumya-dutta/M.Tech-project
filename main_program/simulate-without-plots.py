import numpy as np
import sys
params = sys.argv[1].split(',')
sample_time = 0.1
tot_distance = float(params[0])
max_vel, min_vel = float(params[6]), 0
max_acc, max_dec = float(params[7]), -1*float(params[7])
leader_pos, leader_vel, time = [], [], []
leader_acc, agent_inds, time_completed = [], [], []
K1, K2, K3, time_val = float(params[3]), float(params[4]), float(params[5]), 0.0
D, ball_rad = float(params[2]), float(params[8])
total_system_pos, total_system_vel = {}, {}
total_system_acc = {}
number_of_trips, agent_tag = {}, {}
dist_to_switch = int(params[1])
N = 4
law1 = 0
leader_dict = {}

def nextval(v, d, acc):
    next_v = v + sample_time*acc
    if next_v > max_vel:
        next_v = max_vel
    if acc != 0:
        next_dist = d + ((next_v)**2-v**2)/(2*acc)
        return next_v, next_dist
    else:
        next_dist = d + v*sample_time
        return v, next_dist


def leader():
    global time_val
    init_vel, distance = 0.0, 0.0
    leader_pos.append(0), leader_vel.append(0)
    leader_acc.append(0)
    while leader_pos[-1] < dist_to_switch:
        a = calc_acc(tot_distance, 0, leader_pos[-1], leader_vel[-1])
        vel, dist = nextval(leader_vel[-1], leader_pos[-1], a)
        time_val += sample_time
        leader_pos.append(dist), leader_vel.append(vel)
        leader_acc.append(a)
        time.append(round(time_val, 3))

def calc_acc(l_p, l_v, own_d, own_v):
    global law1
    global time
    rel_dist, rel_vel = l_p - own_d, l_v - own_v
    dist_fdbck = K1*(rel_dist-D)
    vel_fdbck = K2*(rel_vel)
    neg_fdbck = K3*own_v

    l1 = dist_fdbck+vel_fdbck-neg_fdbck
    x = K1*(tot_distance-own_d) + K2*(rel_vel) - K3*own_v

    start_sgn = np.sign(np.ceil((l_p-dist_to_switch)/tot_distance))
    l_change_sgn = np.sign(np.ceil((tot_distance+0.001-l_p)/tot_distance))
    vel_sat_sgn = np.sign(np.ceil(np.sign(max_vel-own_v)-0.1*np.sign(l1)))
    vel_sat_sgn_x = np.sign(np.ceil(np.sign(max_vel-own_v)-0.1*np.sign(x)))

    law1 = l1*start_sgn*l_change_sgn*vel_sat_sgn
    law2 = x*start_sgn*(1-l_change_sgn)*vel_sat_sgn_x

    fdbck = law1+law2
    if fdbck > max_acc:
        return max_acc
    elif fdbck <= max_dec:
        return max_dec
    else:
        return fdbck

def complete(a_ind, num):
    global time_val
    agents_not_started = [x for x in a_ind if x != '1' and x != str(num//2+1)]
    for a in agents_not_started:
        agent_d, agent_v, agent_acc, t = [0], [0], [0], sample_time
        leader_for_agent = leader_dict[a]
        while t < time_val:
            time_index = time.index(t)
            leader_d = total_system_pos[leader_for_agent][time_index]
            leader_v = total_system_vel[leader_for_agent][time_index]
            acc = calc_acc(leader_d, leader_v, agent_d[-1], agent_v[-1])
            v, d = nextval(agent_v[-1], agent_d[-1], acc)
            agent_d.append(d), agent_v.append(v)
            agent_acc.append(acc)
            t += sample_time
            t = round(t, 3)
        total_system_pos[a], total_system_vel[a] = agent_d, agent_v
        total_system_acc[a] = agent_acc
    while((number_of_trips['2']) < N):
        time_val += sample_time
        time_val = round(time_val, 3)
        time.append(time_val)
        for a in a_ind[:num//2]:
            leader_for_agent = a_ind[:num//2][a_ind[:num//2].index(a)-1]
            leader_d = total_system_pos[leader_for_agent][-1]
            leader_v = total_system_vel[leader_for_agent][-1]
            agent_d, agent_v = total_system_pos[a][-1], total_system_vel[a][-1]
            if agent_d == tot_distance:
                agent_v, agent_d = 0, 0
            if leader_d == 0 and agent_d != 0:
                leader_d, leader_v = tot_distance, 0
            if leader_d > 0 and agent_d > leader_d:
                leader_d, leader_v = tot_distance + 0.002, 0
            acc = calc_acc(leader_d, leader_v, agent_d, agent_v)
            v, d = nextval(agent_v, agent_d, acc)
            if d > tot_distance-ball_rad:
                number_of_trips[a] += 1
                d, v = tot_distance, 0
            total_system_pos[a].append(d), total_system_vel[a].append(v)
            total_system_acc[a].append(acc)

def plot_times(n):
    time_req, time_wait = {}, {}
    start_dist = (max_acc*sample_time)**2/(2*max_acc)
    for x in range(1, n+1):
        agent_times = total_system_pos[str(x)]
        time_req_l = []
        time_wait_l = []
        start_times = [y for y in range(len(agent_times)) if agent_times[y] == start_dist]
        #print(start_times)
        end_times = [y for y in range(len(agent_times)) if agent_times[y] == tot_distance]
        #print(end_times)
        l = min(len(end_times), len(start_times))
        for z in range(l):
            time_req_l.append((end_times[z]+1)*sample_time - (start_times[z]+1)*sample_time)
        time_req[str(x)] = time_req_l
        for z in range(1, len(start_times)):
            time_wait_l.append((start_times[z]+1)*sample_time - (end_times[z-1]+1)*sample_time)
        time_wait[str(x)] = time_wait_l
    #print(time_req)
    #print(time_wait)
    time_travel_second, time_wait_second = time_req['2'][-2:], time_wait['2'][-2:]
    t_travel_init, t_wait_init = time_travel_second[0], time_wait_second[0]
    for t in time_travel_second[1:]:
        if np.around(t, decimals = 1) != np.around(t_travel_init, decimals = 1):
            t_travel_init = "Not converged"
            break
    for t in time_wait_second[1:]:
        if np.around(t, decimals = 1) != np.around(t_wait_init, decimals = 1):
            t_wait_init = "Not converged"
            break
    #print(np.around(t_travel_init, decimals = 1))
    #print(np.around(t_wait_init, decimals = 1))
    if t_travel_init != "Not converged" and t_wait_init != "Not converged":
        results.write(str(np.around(t_travel_init, decimals = 1)) + "," + str(np.around(t_wait_init, decimals = 1))+"\n")
    else:
        results.write("Not converged,Not converged\n")

def main():
    num_agents = int(np.ceil(tot_distance/dist_to_switch)*2)
    agent_inds = [str(x) for x in range(1, num_agents+1)]
    agent_leaders = agent_inds[-1:] + agent_inds[:-1]
    for x in range(len(agent_inds)):
        leader_dict[agent_inds[x]] = agent_leaders[x]
        number_of_trips[agent_inds[x]] = 0
    leader()
    #print(leader_dict)
    total_system_pos[agent_inds[0]] = leader_pos
    total_system_vel[agent_inds[0]] = leader_vel
    total_system_acc[agent_inds[0]] = leader_acc
    total_system_pos[agent_inds[num_agents//2]] = leader_pos
    total_system_vel[agent_inds[num_agents//2]] = leader_vel
    total_system_acc[agent_inds[num_agents//2]] = leader_acc
    number_of_trips[agent_inds[0]] += 1
    number_of_trips[agent_inds[num_agents//2]] += 1
    complete(agent_inds, num_agents)
    plot_times(num_agents//2)

results = open('results.csv', 'a')
main()
results.close()
