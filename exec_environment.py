"""
    This code communicates with the coppeliaSim software and simulates shaking a container to mix objects of different color 

    Install dependencies:
    https://www.coppeliarobotics.com/helpFiles/en/zmqRemoteApiOverview.htm
    
    MacOS: coppeliaSim.app/Contents/MacOS/coppeliaSim -GzmqRemoteApi.rpcPort=23004 ~/path/to/file/mix_Intro_to_AI.ttt
    Ubuntu: ./coppeliaSim.sh -GzmqRemoteApi.rpcPort=23004 ~/path/to/file/mix_Intro_to_AI.ttt
"""
import pickle
import sys
# Change to the path of your ZMQ python API
sys.path.append('/app/zmq/')
import numpy as np
from itertools import product
from zmqRemoteApi import RemoteAPIClient
from numpy import random

class Simulation():
    def __init__(self, sim_port = 23004):
        self.sim_port = sim_port
        self.directions = ['Up','Down','Left','Right']
        self.initializeSim()

    def initializeSim(self):
        self.client = RemoteAPIClient('localhost',port=self.sim_port)
        self.client.setStepping(True)
        self.sim = self.client.getObject('sim')
        
        # When simulation is not running, ZMQ message handling could be a bit
        # slow, since the idle loop runs at 8 Hz by default. So let's make
        # sure that the idle loop runs at full speed for this program:
        self.defaultIdleFps = self.sim.getInt32Param(self.sim.intparam_idle_fps)
        self.sim.setInt32Param(self.sim.intparam_idle_fps, 0)  
        
        self.getObjectHandles()
        self.sim.startSimulation()
        self.dropObjects()
        self.getObjectsInBoxHandles()
    
    def getObjectHandles(self):
        self.tableHandle=self.sim.getObject('/Table')
        self.boxHandle=self.sim.getObject('/Table/Box')
    
    def dropObjects(self):
        self.blocks = 18
        frictionCube=0.06
        frictionCup=0.8
        blockLength=0.016
        massOfBlock=14.375e-03
        
        self.scriptHandle = self.sim.getScript(self.sim.scripttype_childscript,self.tableHandle)
        self.client.step()
        retInts,retFloats,retStrings=self.sim.callScriptFunction('setNumberOfBlocks',self.scriptHandle,[self.blocks],[massOfBlock,blockLength,frictionCube,frictionCup],['cylinder'])
        
        print('Wait until blocks finish dropping')
        while True:
            self.client.step()
            signalValue=self.sim.getFloatSignal('toPython')
            if signalValue == 99:
                loop = 20
                while loop > 0:
                    self.client.step()
                    loop -= 1
                break
    
    def getObjectsInBoxHandles(self):
        self.object_shapes_handles=[]
        self.obj_type = "Cylinder"
        for obj_idx in range(self.blocks):
            obj_handle = self.sim.getObjectHandle(f'{self.obj_type}{obj_idx}')
            self.object_shapes_handles.append(obj_handle)

    def getObjectsPositions(self):
        pos_step = []
        box_position = self.sim.getObjectPosition(self.boxHandle,self.sim.handle_world)
        for obj_handle in self.object_shapes_handles:
            # get the starting position of source
            obj_position = self.sim.getObjectPosition(obj_handle,self.sim.handle_world)
            obj_position = np.array(obj_position) - np.array(box_position)
            pos_step.append(list(obj_position[:2]))
        return pos_step
    
    def action(self,direction=None):
        if direction not in self.directions:
            print(f'Direction: {direction} invalid, please choose one from {self.directions}')
            return
        box_position = self.sim.getObjectPosition(self.boxHandle,self.sim.handle_world)
        _box_position = box_position
        span = 0.02
        steps = 5
        if direction == 'Up':
            idx = 1
            dirs = [1, -1]
        elif direction == 'Down':
            idx = 1
            dirs = [-1, 1]
        elif direction == 'Right':
            idx = 0
            dirs = [1, -1]
        elif direction == 'Left':
            idx = 0
            dirs = [-1, 1]

        for _dir in dirs:
            for _ in range(steps):
                _box_position[idx] += _dir*span / steps
                self.sim.setObjectPosition(self.boxHandle, self.sim.handle_world, _box_position)
                self.stepSim()

    def getStates(self):
        value = 0
        possible_states = {}
        digits = '0123456789'
        all_objects = [''.join(obj) for obj in product(digits, repeat=4) if sum(map(int, obj)) == 9]

        for blue_obj in all_objects:
            for red_obj in all_objects:
                istate = blue_obj + red_obj
                value = value+1
                possible_states[istate] = value

        return possible_states

    def getReward(self, blue_objs, red_objs):
        gridAO_b_count = 0
        gridBO_b_count = 0
        gridCO_b_count = 0
        gridDO_b_count = 0
        gridAO_r_count = 0
        gridBO_r_count = 0
        gridCO_r_count = 0
        gridDO_r_count = 0
        for b_pos in blue_objs:
            if -0.05 <= b_pos[0] <= 0 and 0 <= b_pos[1] <= 0.05:
                gridAO_b_count = gridAO_b_count+1
            if 0 <= b_pos[0] <= 0.05 and 0 <= b_pos[1] <= 0.05:
                gridBO_b_count = gridBO_b_count+1
            if 0 <= b_pos[0] <= 0.05 and -0.05 <= b_pos[1] <= 0:
                gridCO_b_count = gridCO_b_count+1
            if -0.05 <= b_pos[0] <= 0 and -0.05 <= b_pos[1] <= 0:
                gridDO_b_count = gridDO_b_count+1

        for r_pos in red_objs:
            if -0.05 <= r_pos[0] <= 0 and 0 <= r_pos[1] <= 0.05:
                gridAO_r_count = gridAO_r_count+1
            if 0 <= r_pos[0] <= 0.5 and 0 <= r_pos[1] <= 0.05:
                gridBO_r_count = gridBO_r_count+1
            if 0 <= r_pos[0] <= 0.05 and -0.05 <= r_pos[1] <= 0:
                gridCO_r_count = gridCO_r_count+1
            if -0.05 <= r_pos[0] <= 0 and -0.05 <= r_pos[1] <= 0:
                gridDO_r_count = gridDO_r_count+1

        state = str(gridAO_b_count) + str(gridBO_b_count) + str(gridCO_b_count) + str(gridDO_b_count) + str(gridAO_r_count) + str(gridBO_r_count) + str(gridCO_r_count) + str(gridDO_r_count)
        reward = self.calculateReward(state)
        return state, reward

    def calculateReward(self, state):
        positions_array = [int(position) for position in state]
        rewards_dict = {
            1: 30,
            2: 50,
            3: 40,
            4: 20,
            5: 10,
            6: -10,
            7: -20,
            8: -30,
            0: -60
        }
        max_count = max(positions_array)
        min_count = min(positions_array)

        if positions_array.count(2) == 6 and positions_array.count(3) == 2:
            reward = 100
        elif positions_array.count(9) == 2:
            reward = -100
        else:
            reward = rewards_dict.get(max_count) + rewards_dict.get(min_count)

        return reward


    def stepSim(self):
        self.client.step()

    def stopSim(self):
        self.sim.stopSimulation()


def main():
    episodes = 10
    steps = 50
    Q_table = np.zeros((48400,4))
    exploration_probability = 0.1
    gamma = 0.99
    lr = 0.1
    f = open('reward.txt', 'w')

    for episode in range(episodes):
        print(f'Running episode: {episode + 1}')
        env = Simulation()
        initial_positions = env.getObjectsPositions()
        blue_objs = initial_positions[:9]
        red_objs = initial_positions[9:]
        state, reward = env.getReward(blue_objs, red_objs)
        state_array = list(map(int, state))
        while sum(state_array[:4]) != 9 or sum(state_array[4:]) != 9:
            env.stopSim()
            env = Simulation()
            initial_positions = env.getObjectsPositions()
            blue_objs = initial_positions[:9]
            red_objs = initial_positions[9:]
            state, reward = env.getReward(blue_objs, red_objs)
            state_array = list(map(int, state))
        total_reward = reward

        for _ in range(steps):
            possible_states = env.getStates()
            if random.uniform(0, 1) < exploration_probability:
                action = random.randint(0, len(env.directions))
            else:
                action = np.argmax(Q_table[possible_states[state], :])
            env.action(direction = env.directions[action])
            positions = env.getObjectsPositions()
            blue_objs = positions[:9]
            red_objs = positions[9:]
            current_state, current_reward = env.getReward(blue_objs, red_objs)
            current_state_array = list(map(int, current_state))
            if sum(current_state_array[:4]) != 9:
                break
            if sum(current_state_array[4:]) != 9:
                break
            possible_states = env.getStates()
            Q_table[possible_states[state], action] = (1-lr)*Q_table[possible_states[state], action] + lr*(current_reward+gamma*(Q_table[possible_states[current_state], np.argmax(Q_table[possible_states[current_state],:])]))
            total_reward = total_reward + current_reward
        f.write(f"\n Total Reward for episode {episode + 1} : {total_reward}")

        env.stopSim()

    with open('Q_table.pkl', 'wb') as file:
        pickle.dump(Q_table, file)


if __name__ == '__main__':
    
    main()
