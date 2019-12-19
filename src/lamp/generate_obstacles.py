#!/usr/bin/env python

from utility import euclidean_distance
import random
import os
import math

'''
B config (some more correlation but still pretty random)
barrier_prob = 0.2
i_prob = 0.2
ii_prob = 0.2 
iii_prob = 0.2
vi_prob = 0.2
vii_prob = 0.2
viii_prob = 0.2
'''

'''
D config
bgi_prob = 0.5
a_prob = 0.1
ce_prob = 0.5
dfh_prob = 0.4
i_prob = 0
ii_prob = 0 
iii_prob = 0.9
vi_prob = 0
vii_prob = 0
viii_prob = 0
'''
'''
E config
    bgi_prob = 0.5
    a_prob = 0
    ce_prob = 0.5
    dfh_prob = 0.4
    i_prob = 0.3
    ii_prob = 0 
    iii_prob = 0.8
    vi_prob = 0
    vii_prob = 0
    viii_prob = 0
'''

def spawn_obstacles():
    # obstacle probabilities
    bgi_prob = 0.5
    a_prob = 0.1
    ce_prob = 0.5
    dfh_prob = 0.4
    i_prob = 0
    ii_prob = 0 
    iii_prob = 0.9
    vi_prob = 0
    vii_prob = 0
    viii_prob = 0

    random.seed()
    cmd = ''
    del_cmd = ''
    obstacles = []
    v_spawn = False #only spawn dumpster_v if dumpster_vi or dumpster_vii present
    avoid_set = [(18.25,17.521),(16.681,3.661),(16.681,6.725),(14.182,8.204),(14.294,17.532),(10.169,8.204),(10.168,17.558),(5.787,3.507),(2.841,6.814)] #list of doorway locations to prevent spawning debris in doorways 

    suff = random.randint(0,100) # random suffix to all model names
    barriers = []

    if random.random() < bgi_prob:
        barriers += ['B', 'G', 'I']

    if random.random() < a_prob:
        barriers += ['A']

    if random.random() < ce_prob:
        barriers += ['C', 'E']
        # ii_prob = 1 # correlate w/ dumpsters b/w (2,7)

    if random.random() < dfh_prob:
        barriers += ['D', 'F', 'H']

    # add barriers to cmd
    for letter in barriers:
        temp_cmd, temp_del, model_name = barrier(letter, suff)
        cmd += temp_cmd
        del_cmd += temp_del
        obstacles += [model_name]

    if random.random() < i_prob: 
        x_pos1 = 18.424
        x_pos2 = 17.198
        y_pos = random.random()*6.0 + 9.0
        model_name1 = 'block_i_1_{}'.format(suff)
        model_name2 = 'block_i_2_{}'.format(suff)
        cmd += ("rosrun gazebo_ros spawn_model " + 
                "-database drc_practice_orange_jersey_barrier " + 
                "-gazebo -model " + model_name1 + " -x " + str(x_pos1) + 
                " -y " + str(y_pos) + " -Y 1.571\n")
        cmd += ("rosrun gazebo_ros spawn_model " + 
                "-database drc_practice_blue_cylinder " +
                "-gazebo -model " + model_name2 + " -x " + str(x_pos2) +
                " -y " + str(y_pos) + "\n")
        del_cmd += ('rosservice call gazebo/delete_model block_i_1\n' + 
                'rosservice call gazebo/delete_model block_i_2\n')
        obstacles += [model_name1, model_name2]
        avoid_set.extend([(x_pos1,y_pos), (x_pos2,y_pos)])

    if random.random() < ii_prob: 
        x_pos1 = 15.067
        x_pos2 = 12.918
        y_pos = random.random()*5.0 + 10.0
        model_name1 = "dumpster_ii_1_{}".format(suff)
        model_name2 = "dumpster_ii_2_{}".format(suff)
        cmd += ("rosrun gazebo_ros spawn_model -database dumpster" + 
                " -gazebo -model " + model_name1 + " -x " + str(x_pos1) + 
                ' -y ' + str(y_pos) + " -Y 0\n" + 
                "rosrun gazebo_ros spawn_model -database dumpster" + 
                " -gazebo -model " + model_name2 + " -x " + str(x_pos2) + 
                ' -y ' + str(y_pos) + ' -Y 1.570796\n')
        del_cmd += 'rosservice call gazebo/delete_model dumpster_ii_1\n\
            rosservice call gazebo/delete_model dumpster_ii_2\n'
        obstacles += [model_name1, model_name2]
        avoid_set.append((x_pos1, y_pos))
        avoid_set.append((x_pos2, y_pos))

    if random.random() < iii_prob: 
        x_pos = random.random()*4.0 + 6.6
        y_pos1 = 11.42
        y_pos2 = 9.5
        model_name1 = "dumpster_iii_1_{}".format(suff)
        model_name2 = "dumpster_iii_2_{}".format(suff)
        cmd += ('rosrun gazebo_ros spawn_model -database dumpster\
                 -gazebo -model ' + model_name1 + 
                ' -x ' + str(x_pos) + ' -y ' + str(y_pos1) + ' -Y 0.0\n\
                rosrun gazebo_ros spawn_model -database dumpster\
                 -gazebo -model ' + model_name2 + 
                ' -x ' + str(x_pos) + ' -y ' + str(y_pos2) + ' -Y 0.0\n')
        del_cmd = del_cmd + 'rosservice call gazebo/delete_model dumpster_iii_1\n\
            rosservice call gazebo/delete_model dumpster_iii_2\n'
        obstacles += [model_name1, model_name2]
        avoid_set.append((x_pos,y_pos1))
        avoid_set.append((x_pos,y_pos2))

    if random.random() < vi_prob: 
        x_pos1 = 12.774
        y_pos1 = 6.486
        v_spawn = True
        model_name = "dumpster_vi_{}".format(suff)
        cmd += ('rosrun gazebo_ros spawn_model -database dumpster\
                 -gazebo -model ' + model_name +  
                ' -x ' + str(x_pos1) + ' -y ' + str(y_pos1) + ' -Y -1.160\n')
        del_cmd += 'rosservice call gazebo/delete_model dumpster_vi\n'
        obstacles += [model_name]
        avoid_set.append((x_pos1,y_pos1))

    if random.random() < vii_prob: 
        x_pos = 13.127 
        y_pos = 3.735
        v_spawn = True
        model_name = "dumpster_vii_{}".format(suff)
        cmd += ('rosrun gazebo_ros spawn_model -database dumpster\
                 -gazebo -model ' + model_name + 
                ' -x ' + str(x_pos) + ' -y ' + str(y_pos) + ' -Y -1.567\n')
        del_cmd += 'rosservice call gazebo/delete_model dumpster_vii\n'
        obstacles += [model_name]
        avoid_set.append((x_pos,y_pos))

    if v_spawn:
        x_pos = 15.198
        y_pos = 4.968 
        model_name = "dumpster_v_{}".format(suff)
        cmd += ('rosrun gazebo_ros spawn_model -database dumpster -gazebo -model ' + 
                model_name + ' -x ' + str(x_pos) + ' -y ' + str(y_pos) + ' -Y 0.0\n')
        del_cmd += 'rosservice call gazebo/delete_model dumpster_v\n'
        obstacles += [model_name]
        avoid_set.append((x_pos,y_pos))

    if random.random() < viii_prob: 
        x_pos = 4.3266
        y_pos = 5.1462
        model_name = "dumpster_viii_{}".format(suff)
        cmd += ('rosrun gazebo_ros spawn_model -database dumpster\
                 -gazebo -model ' + model_name +  
                ' -x ' + str(x_pos) + ' -y ' + str(y_pos) + ' -Y 1.092\n')
        del_cmd += 'rosservice call gazebo/delete_model dumpster_viii\n'
        obstacles += [model_name]
        avoid_set.append((x_pos,y_pos))

    num_debris = random.randint(0,11)
    num_spawned = 0
    nloops = 0 
    while num_spawned < num_debris:
        # prevent infinite loops
        nloops += 1
        if nloops > 1000: break

        loc = random.random()
        if loc < 0.1: #a
            x_pos = 18.8
            ymin = 6
            ymax = 16.75
            y_pos = (ymax-ymin)*random.random()+ymin
        elif loc < 0.2: #b
            x_pos = 17.4
            ymin = 6.0
            ymax = 16.75
            y_pos = (ymax-ymin)*random.random()+ymin		
        elif loc < 0.4: #c
            xmin = 12.585
            xmax = 15.95
            x_pos = (xmax-xmin)*random.random()+xmin
            ymin = 8.91
            ymax = 16.75
            y_pos = (ymax-ymin)*random.random()+ymin
        elif loc < 0.6: #d
            xmin = 6.55
            xmax = 15.95
            x_pos = (xmax-xmin)*random.random()+xmin
            ymin = 3.0
            ymax = 7.45
            y_pos = (ymax-ymin)*random.random()+ymin
        elif loc < 0.7: #e
            xmin = 3.58
            xmax = 5.0
            x_pos = (xmax-xmin)*random.random()+xmin
            ymin = 3.0
            ymax = 7.48
            y_pos = (ymax-ymin)*random.random()+ymin
        elif loc < 0.85: #f1
            xmin = 3.58
            xmax = 11.11
            x_pos = (xmax-xmin)*random.random()+xmin
            ymin = 8.93
            ymax = 11.94
            y_pos = (ymax-ymin)*random.random()+ymin
        else: #f2
            xmin = 3.58
            xmax = 11.11
            x_pos = (xmax-xmin)*random.random()+xmin
            ymin = 13.4
            ymax = 16.8
            y_pos = (ymax-ymin)*random.random()+ymin
        if all(euclidean_distance(coord,(x_pos,y_pos))>3.0 for coord in avoid_set):
            model_name = "debris{}_{}".format(num_spawned, suff)
            cmd += ('rosrun gazebo_ros spawn_model -database drc_practice_blue_cylinder\
                    -gazebo -model ' + model_name + ' -x ' + str(x_pos) + ' -y ' + str(y_pos) + ' -Y 0.0\n')
            del_cmd = del_cmd + 'rosservice call gazebo/delete_model debris' + str(num_spawned) + '\n'
            obstacles += [model_name]
            avoid_set.append((x_pos, y_pos))
            num_spawned = num_spawned + 1
    os.system(cmd)
    return obstacles
    # return del_cmd

def delete_obstacles(del_cmd):
	os.system(del_cmd)

def barrier(letter, suff):
    cmd = "rosrun gazebo_ros spawn_model -database drc_practice_white_jersey_barrier\
            -gazebo -model barrier_{}_{} ".format(letter, suff)

    if letter == 'A':
        cmd += '-x 18.25 -y 17.521 -Y 1.571\n'
    elif letter == 'B':
        cmd += '-x 16.681 -y 3.661 -Y 0.0\n'
    elif letter == 'C':
        cmd += '-x 16.681 -y 6.725 -Y 0.0\n'
    elif letter == 'D':
        cmd += '-x 14.182 -y 8.204 -Y 1.571\n'
    elif letter == 'E':
        cmd += '-x 14.294 -y 17.532 -Y 1.571\n'
    elif letter == 'F':
        cmd += '-x 10.169 -y 8.204 -Y 1.571\n'
    elif letter == 'G':
        cmd += '-x 10.168 -y 17.558 -Y 1.571\n'
    elif letter == 'H':
        cmd += '-x 5.787 -y 3.507 -Y 0.0\n'
    elif letter == 'I':
        cmd += '-x 2.841 -y 6.814 -Y 0.0\n'
    else:
        print("That barrier doesn't exist!")
        return '', '' 

    del_cmd = "rosservice call gazebo/delete_model barrier_{}\n".format(letter)
    model_name = "barrier_{}_{}".format(letter, suff)

    return cmd, del_cmd, model_name

def add_number(number):
    # add task number to gazebo
    random.seed()
    str_num = str(number)
    digits = range(len(str_num))
    digits.reverse()
    model_names = []
    cmd = ''

    for i in digits:
        suff = random.randint(0,100) # random suffix to all model names
        n = str_num[i]
        name = "n_{}_{}".format(i, suff)
        x = -1.5 - 1.75*(len(str_num) - 1 - i)
        cmd += "rosrun gazebo_ros spawn_model\
                 -file /home/f4tsang/.gazebo/models/number{}/model.sdf\
                 -gazebo -model {} -x {} -z 0 -Y -1.571\n".format(n, name, x)

        model_names.append(name)

    os.system(cmd)
    return model_names
