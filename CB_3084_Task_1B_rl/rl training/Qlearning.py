'''
*
*   ===================================================
*       CropDrop Bot (CB) Theme [eYRC 2025-26]
*   ===================================================
*
*  This script is intended to be an Boilerplate for 
*  Task 1B of CropDrop Bot (CB) Theme [eYRC 2025-26].
*
*  Filename:		Qlearning.py
*  Created:		    24/08/2025
*  Last Modified:	24/08/2025
*  Author:		    e-Yantra Team
*  Team ID:		    [ CB_2202 ]
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*  
*  e-Yantra - An MHRD project under National Mission on Education using ICT (NMEICT)
*
*****************************************************************************************
'''
'''You can Modify the this file,add more functions According to your usage.
   You are not allowed to add any external packges,Beside the included Packages.You can use Built-in Python modules.
'''
import numpy as np
import random
import pickle
import os

class QLearningController:
    def __init__(self, n_states=0, n_actions=0, filename="q_table.pkl"): 
        self.n_states = n_states
        self.n_actions = n_actions

        self.lr = 0.2  # Learning rate: how much new information overrides old
        self.epsilon = 0.0  # Exploration rate: chance of choosing a random action

        self.filename = filename

        # some inbetween required parameters
        self.prev_error = 0.0
        self.turn_dir = 0
        self.prev_turn_dir = 0
        self.in_curve = False
        self.curve_timer = 0

        # Initialize Q-table with zeros; dimensions: [states x actions]
        self.q_table = np.zeros((n_states, n_actions))

        self.action_list = [0, 1, 2, 3, 4]  # Example: 0 = left, 1 = forward, 2 = right

        self.actions = {}


    def Get_state(self, sensor_data): 

        def discretize(value, bins=3):
            if value < 0.3:
                return 0  # off line
            elif value < 0.7:
                return 1  # edge of line
            else:
                return 2  # on line 

        state = (
            discretize(sensor_data['left_corner']),
            discretize(sensor_data['left']),
            discretize(sensor_data['middle']),
            discretize(sensor_data['right']),
            discretize(sensor_data['right_corner'])
        )

        # print(sensor_data)
        state_index = 0
        for v in state:
            state_index = state_index * 3 + v
        # print(state_index)
        return state_index
    

    def Calculate_reward(self, state, last_action):
        # --- decode state back to sensor values ---
        sensors = []
        temp = state
        for _ in range(5):
            sensors.append(temp % 3)
            temp //= 3
        sensors.reverse()

        lcorner, left, middle, right, rcorner = sensors

        weights = [-2, -1, 0, 1, 2]
        values  = [lcorner, left, middle, right, rcorner]

        reward = 0

        # --- LOST LINE ---
        den = sum(values)

        # # detect dashed turn 
        # post_turn_gap = (
        #     den == 0 and
        #     self.prev_error is not None and
        #     abs(self.prev_error) > 0.8   # just came out of a sharp turn
        # )

        # print(den)
        if den == 0:
            self.in_curve = False
            return -60

        # --- weighted centroid error ---
        error = sum(w * v for w, v in zip(weights, values)) / den
        abs_error = abs(error)

        # --- curve detection (robust) ---
        # print(f"ERROR: {abs_error} middle: {middle} L: {left} R: {right}")
        is_curve = abs_error > 0.6 or (middle <= 1 and (left == 2 or right == 2))

        
        # ===== STRAIGHT LINE MODE ======
        if not is_curve:
            # alignment reward
            if abs_error == 0:
                reward += 25
            elif abs_error <= 0.35:
                reward += 15
            elif abs_error <= 0.75:
                reward += 5
            else:
                reward -= 10

            # action shaping
            if last_action == 2:  # forward
                if abs_error <= 0.35:
                    reward += 12
                else:
                    reward -= 6
            elif last_action in [1, 3]:
                if abs_error <= 0.35:
                    reward -= 4   # unnecessary turning
                else:
                    reward += 2
            elif last_action in [0, 4]:
                reward -= 6
        
        # ========= CURVE MODE ==========
        else:
            # STRONG curve entry incentive
            if abs_error > 0.8:
                if error < 0 and last_action == 1:   # left turn correct
                    reward += 20
                elif error > 0 and last_action == 3: # right turn correct
                    reward += 20
                else:
                    reward -= 10

            # correct turn direction
            if error < -0.4:   # line on left → turn left
                if last_action == 1:
                    reward += 15
                elif last_action == 3:
                    reward -= 6
            elif error > 0.4:  # line on right → turn right
                if last_action == 3:
                    reward += 15
                elif last_action == 1:
                    reward -= 6

            # forward only if already aligned
            if last_action == 2:
                if abs_error <= 0.35:
                    reward += 6
                elif abs_error <= 0.8:
                    reward -= 4
                else:
                    reward -= 6

            # discourage spinning
            if last_action in [0, 4]:
                reward -= 8

            # small side-sensor stability bonus
            if left == 1 or right == 1:
                reward += 1
            if lcorner == 2 or rcorner == 2:
                reward += 2

        # --- time penalty ---
        reward -= 0.05
        return reward
      
    
    def update_q_table(self, state, action, reward, next_state):
       
        # Find index of action in action list
        a_idx = self.action_list.index(action)

        old_value = self.q_table[state][a_idx]
        next_max = np.max(self.q_table[next_state])


        update = old_value + self.lr * ((reward + 0.9 * next_max) - old_value)

        self.q_table[state][a_idx] = update  


    def choose_action(self, state):    
        if random.uniform(0,1) < self.epsilon: 
            index = random.randint(0, len(self.action_list) - 1)
        else:
           index = np.argmax(self.q_table[state])

        # print("action: " ,self.action_list[index])
        return self.action_list[index]  


    def perform_action(self, action):
        forward = 2.5
        slow = 1.6
        stop = 0.0
        in_place = 1.8

        if action == 0:     
            return -in_place,in_place
        elif action == 1:   
            return slow,forward
        elif action == 2:   
            return forward,forward
        elif action == 3:            
            return forward,slow
        elif action == 4:
            return in_place,-in_place
        else:
            return stop,stop



    def save_q_table(self):
        """
        Save the current Q-table and parameters to a file.

        Useful for keeping learned behavior between runs.

        === INSTRUCTIONS: You may Save Additional Thing while saving but do not Remove the the following Parameters ===
        """
        with open(self.filename, 'wb') as f:
            pickle.dump({
                'q_table': self.q_table,
                'epsilon': self.epsilon,
                'n_action': self.n_actions,
                'n_states': self.n_states
                # Add any additional data you want to save
            }, f)

    def load_q_table(self):
        """
        Load the Q-table and parameters from file, if it exists.

        Returns:
        - True if data was loaded successfully, False otherwise.
        """
        if os.path.exists(self.filename):
            with open(self.filename, 'rb') as f:
                data = pickle.load(f)
            self.q_table = data.get('q_table', self.q_table)
            self.epsilon = data.get('epsilon', self.epsilon)
            self.n_actions = data.get('n_action', self.n_actions)
            self.n_states = data.get('n_states', self.n_states)
            # Load other data here if needed
            return True
        return False
