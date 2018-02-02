import matplotlib.pyplot as plt
import numpy as np
sample_time = 0.1
tot_distance = 10000
max_vel, min_vel = 50, 0
max_acc, max_dec = 10, -10
leader_pos, leader_vel, time = [], [], []
leader_disrupt_pos, leader_disrupt_vel, leader_disrupt_acc = [], [], []
leader_acc, agent_inds, time_completed = [], [], []
K1, K2, K3, time_val = 1, 1, 4.06, 0.0
D, ball_rad, wait_time_val = 1000, 0.15, 30
fpout, fvout = open('Position.csv', 'w'), open('Velocity.csv', 'w')
faout = open('Input.csv', 'w')
total_system_pos, total_system_vel = {}, {}
total_system_acc = {}
number_of_trips, agent_tag = {}, {}
dist_to_switch = 4000
N = 15
law1 = 0
leader_dict, wait_time = {}, {}
disruption_dist, disruption_dur, disruption_cnt = 8000, 1200, 0


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

def calc_acc(l_p, l_v, own_d, own_v, f):
    global law1
    global time
    rel_dist, rel_vel = l_p - own_d, l_v - own_v
    dist_fdbck = K1*(rel_dist-(1-f)*D)
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


def leader():
    global time_val
    init_vel, distance = 0.0, 0.0
    leader_pos.append(0), leader_vel.append(0), leader_disrupt_pos.append(0)
    leader_acc.append(0), leader_disrupt_vel.append(0)
    leader_disrupt_acc.append(0)
    while leader_pos[-1] <= dist_to_switch or leader_disrupt_pos [-1]<= dist_to_switch:
        a = calc_acc(tot_distance, 0, leader_pos[-1], leader_vel[-1], 0)
        a_dis = calc_acc(disruption_dist, 0, leader_disrupt_pos[-1], leader_disrupt_vel[-1], 1)
        vel, dist = nextval(leader_vel[-1], leader_pos[-1], a)
        vel_dis, pos_dis = nextval(leader_disrupt_vel[-1], leader_disrupt_pos[-1], a_dis)
        leader_pos.append(dist), leader_vel.append(vel)
        leader_acc.append(a)
        leader_disrupt_pos.append(pos_dis), leader_disrupt_vel.append(vel_dis)
        leader_disrupt_acc.append(a_dis)
        time.append(round(time_val, 3))
        time_val += sample_time
        round(time_val, 3)
        #print(leader_pos[-1])
        #print(time_val)



def complete(a_ind, num, leaders):
    global time_val
    global disruption_cnt
    #print(time_val)
    count, count_dis, flag = 0, 0, 0
    for a in a_ind:
        wait_time[a] = 0
    agents_not_started = [x for x in a_ind if x != '1' and x != str(num//2+1)]
    #print(agents_not_started)
    for a in agents_not_started:
        agent_d, agent_v, agent_acc, t = [0], [0], [0], 0.0
        leader_for_agent = leader_dict[a]
        while t <= round(time_val,3):
            time_index = time.index(t)
            leader_d = total_system_pos[leader_for_agent][time_index]
            leader_v = total_system_vel[leader_for_agent][time_index]
            acc = calc_acc(leader_d, leader_v, agent_d[-1], agent_v[-1], 0)
            #print(a + " " + str(acc) + " " + str(leader_d) + " " + str(t))
            v, d = nextval(agent_v[-1], agent_d[-1], acc)
            agent_d.append(d), agent_v.append(v)
            agent_acc.append(acc)
            t += sample_time
            t = round(t, 3)
        total_system_pos[a], total_system_vel[a] = agent_d, agent_v
        total_system_acc[a] = agent_acc
    #print(time_val)
    while((number_of_trips['2']) < N):
        #print(total_system_pos['4'])
        time_val += sample_time
        time_val = round(time_val, 3)
        time.append(time_val)
        #print(a_ind[:num//2])
        for a in a_ind:
            leader_for_agent = leaders[a]
            #print(a + " " +str(number_of_trips[a]))
            #print(number_of_trips[leader_for_agent])
            if a == '1' and number_of_trips[a] == 1 and number_of_trips[leader_for_agent] == 1:
                leader_d, leader_v = tot_distance + 0.002, 0
            else:
                leader_d = total_system_pos[leader_for_agent][-1]
                leader_v = total_system_vel[leader_for_agent][-1]
            agent_d, agent_v = total_system_pos[a][-1], total_system_vel[a][-1]
            if a == '1' and agent_d > disruption_dist - ball_rad and disruption_cnt == 0:
                if count_dis <= disruption_dur:
                    flag = 1
                    count_dis += 1
                else:
                    disruption_cnt = 1
            if agent_d == tot_distance:
                #print(a)
                agent_v, agent_d = 0, 0
            if leader_d == 0 and agent_d != 0:
                leader_d, leader_v = tot_distance, 0
            if leader_d > 0 and agent_d > leader_d:
                leader_d, leader_v = tot_distance + 0.002, 0
            if a == '1' and number_of_trips[a] == 0 and number_of_trips[leader_for_agent] == 1:
                leader_d, leader_v = tot_distance + 0.002, 0
            if a == '1' and agent_d < disruption_dist - ball_rad and disruption_cnt == 0:
                acc = calc_acc(disruption_dist, 0, agent_d, agent_v, 1)
            else:
                acc = calc_acc(leader_d, leader_v, agent_d, agent_v, 0)
            if agent_d == 0 and wait_time[a] <= wait_time_val:
                acc, wait_time[a] = 0, wait_time[a] + 1
            if wait_time[a] > wait_time_val and acc > 0:
                wait_time[a] = 0
                #acc = calc_acc(leader_d, leader_v, agent_d, agent_v, 0)

            if flag == 1 and a == '1':
                flag = 0
                agent_d, agent_v = disruption_dist, 0
                acc = 0
            v, d = nextval(agent_v, agent_d, acc)

            #print(d)
            if d > tot_distance-ball_rad:
                #print(a)
                number_of_trips[a] += 1
                d, v = tot_distance, 0
            #print(a)
            #print(d)
            total_system_pos[a].append(d), total_system_vel[a].append(v)
            total_system_acc[a].append(acc)


def write_to_file(n):
    s_p_total, s_v_total, s_a_total = "Time", "Time", "Time"
    for ind in range(1, n+1):
        s = ",Agent " + str(ind)
        s_p_total += s
        s_v_total += s
        s_a_total += s
    s_p_total += "\n"
    s_v_total += "\n"
    s_a_total += "\n"
    for ind in range(len(time)):
        s_p, s_v, s_a = str(time[ind]), str(time[ind]), str(time[ind])
        for k in total_system_vel.keys():
            s_p += "," + str(total_system_pos[k][ind])
            s_v += "," + str(total_system_vel[k][ind])
            s_a += "," + str(total_system_acc[k][ind])
        s_p_total += s_p + "\n"
        s_v_total += s_v + "\n"
        s_a_total += s_a + "\n"
    fpout.write(s_p_total)
    fvout.write(s_v_total)
    faout.write(s_a_total)

def plot_times(n):
    time_req = {}
    start_dist = (max_acc*sample_time)**2/(2*max_acc)
    for x in range(1, n+1):
        agent_times = total_system_pos[str(x)]
        #print(agent_times.index(start_dist))
        time_req_l = []
        start_times = [y for y in range(len(agent_times)) if agent_times[y] == start_dist]
        #start_times = [y for y in range(len(agent_times)) if agent_times[y] > 0 and agent_times[y-1] == 0]
        print(start_times)
        end_times = [y for y in range(len(agent_times)) if agent_times[y] == tot_distance]
        print(end_times)
        l = min(len(end_times), len(start_times))
        #print(x)
        #print(l)
        for z in range(l):
            #print(time[end_times[z]])
            time_req_l.append((end_times[z]+1)*sample_time - (start_times[z]+1)*sample_time)
        time_req[str(x)] = time_req_l
        #print(start_times)
        #print(end_times)
    print(time_req)
    fig = plt.figure()
    ax1 = fig.add_subplot(111)
    labels = []
    for x in range(1, n+1):
        labels.append("Agent" + str(x))
    for k in time_req.keys():
        times = time_req[k]
        trips = len(times)
        num_trips = [x+1 for x in range(trips)]
        #m = {'1':"o", '2':"s"}
        ax1.scatter(num_trips, times, label = labels[int(k)-1])
        #plt.title("Time required by agent " + str(k))
        plt.xlabel("Trip number")
        plt.ylabel("Time required in seconds")
        #plt.savefig("Time required by agent " + str(k))
        #plt.clf()
    plt.title("Time required by different agents in different trips")
    plt.legend()
    plt.savefig("Time required by Agents.eps", format = 'eps', dpi = 1000)
    plt.clf()
    print(total_system_pos.keys())
    for k, v in total_system_pos.items():
        l, l1 = len(total_system_pos[k]), len(time)
        le = min(l,l1)
        plt.plot(time[:le], total_system_pos[k][:le], label = "Agent"+k)
    plt.xlabel('Time in seconds')
    plt.ylabel('Position in meters')
    plt.title('Position of different agents vs Time')
    plt.legend()
    plt.savefig('Position vs time for different agents.eps', format = 'eps', dpi = 1000)
    plt.clf()
    for k, v in total_system_vel.items():
        l, l1 = len(total_system_vel[k]), len(time)
        le = min(l,l1)
        plt.plot(time[:le], total_system_vel[k][:le], label = "Agent"+k)
    plt.xlabel('Time in seconds')
    plt.ylabel('Velocity in m/s')
    plt.title('Velocity of different agents vs Time')
    plt.legend()
    plt.savefig('Velocity vs time for different agents.eps', format = 'eps', dpi = 1000)
    plt.clf()
    for k, v in total_system_acc.items():
        l, l1 = len(total_system_acc[k]), len(time)
        le = min(l,l1)
        plt.plot(time[:le], total_system_acc[k][:le], label = "Agent"+k)
    plt.xlabel('Time in seconds')
    plt.ylabel('Acceleration in m/s^2')
    plt.title('Acceleration of different agents vs Time')
    plt.legend()
    plt.savefig('Acc vs time for different agents.eps', format = 'eps', dpi = 1000)



def main():
    num_agents = int(np.ceil(tot_distance/dist_to_switch)*2)
    agent_inds = [str(x) for x in range(1, num_agents+1)]
    agent_leaders = agent_inds[-1:] + agent_inds[:-1]
    for x in range(len(agent_inds)):
        leader_dict[agent_inds[x]] = agent_leaders[x]
        number_of_trips[agent_inds[x]] = 0
    leader()
    print(leader_dict)
    total_system_pos[agent_inds[0]] = leader_disrupt_pos
    total_system_vel[agent_inds[0]] = leader_disrupt_vel
    total_system_acc[agent_inds[0]] = leader_disrupt_acc
    total_system_pos[agent_inds[num_agents//2]] = leader_pos
    total_system_vel[agent_inds[num_agents//2]] = leader_vel
    total_system_acc[agent_inds[num_agents//2]] = leader_acc
    #number_of_trips[agent_inds[0]] += 1
    #number_of_trips[agent_inds[num_agents//2]] += 1
    print(leader_disrupt_pos[-1])
    print(round(time_val, 3))
    time.append(round(time_val, 3))
    complete(agent_inds, num_agents, leader_dict)
    '''
    for x in range(num_agents//2):
        total_system_pos[agent_inds[x+num_agents//2]] = total_system_pos[agent_inds[x]]
        total_system_vel[agent_inds[x+num_agents//2]] = total_system_vel[agent_inds[x]]
        total_system_acc[agent_inds[x+num_agents//2]] = total_system_acc[agent_inds[x]]
    '''
    write_to_file(num_agents)
    #print(total_system_pos['3'])
    #print(time)
    plot_times(num_agents//2)

main()
