from __future__ import annotations
from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt
from .terrain import generate_reference_and_limits
from .control import PDController

import pandas as pd

class Submarine:
    def __init__(self):

        self.mass = 1
        self.drag = 0.1
        self.actuator_gain = 1 #affects how much controller impacts movement

        self.dt = 1 # Time step for discrete time simulation

        #inital positions and velocity
        self.pos_x = 0
        self.pos_y = 0
        self.vel_x = 1 # Constant velocity in x direction
        self.vel_y = 0

    
        #transition method updates the positon and velocity of submarine after each time step, dt based on the controller action and disturbances
        
    def transition(self, action: float, disturbance: float):
        self.pos_x += self.vel_x * self.dt
        self.pos_y += self.vel_y * self.dt

        force_y = -self.drag * self.vel_y + self.actuator_gain * (action + disturbance) #opposing drag force, scaled controller action, scaled disturbance
        acc_y = force_y / self.mass
        self.vel_y += acc_y * self.dt


    def get_depth(self) -> float:
        return self.pos_y
    
    def get_position(self) -> tuple:
        return self.pos_x, self.pos_y
    
    def reset_state(self):
        self.pos_x = 0
        self.pos_y = 0
        self.vel_x = 1
        self.vel_y = 0
    
class Trajectory:
    def __init__(self, position: np.ndarray): 
        #position- numpy array which contains x and y positions over time (index corresponds to subsequent time steps)
        self.position = position  



#plot method plots the trajectory of the submarine over time

    def plot(self):
        plt.plot(self.position[:, 0], self.position[:, 1])
        plt.show()



#plot_completed_mission method plots completed trajectory of sumbarine along with reference depth (from mission class) and cave limits

    def plot_completed_mission(self, mission: Mission):
        x_values = np.arange(len(mission.reference)) #represent time steps corresponding to length of mission NOT x position

        #Find min and max height (cave limits) at each time stamp from mission class
        min_depth = np.min(mission.cave_depth)
        max_height = np.max(mission.cave_height)

        plt.fill_between(x_values, mission.cave_height, mission.cave_depth, color='blue', alpha=0.3) #Fills water within cave itself blue
        
        #Fills cave itself (i.e. the rock) brown
        plt.fill_between(x_values, mission.cave_depth, min_depth*np.ones(len(x_values)), 
                         color='saddlebrown', alpha=0.3)
        plt.fill_between(x_values, max_height*np.ones(len(x_values)), mission.cave_height, 
                         color='saddlebrown', alpha=0.3)
                       
        plt.plot(self.position[:, 0], self.position[:, 1], label='Trajectory')
        plt.plot(mission.reference, 'r', linestyle='--', label='Reference')
        plt.legend(loc='upper right')
        plt.show()

@dataclass
class Mission:
    reference: np.ndarray #initalising???
    cave_height: np.ndarray
    cave_depth: np.ndarray


#random_mission mehtod generates a random mission with specified duration and scale.
#Returns random mission with target depth reference, cave height and depth limits via 3 numpy arrays

    @classmethod
    def random_mission(cls, duration: int, scale: float): #cls refers to class itself in class methods
        (reference, cave_height, cave_depth) = generate_reference_and_limits(duration, scale)
        return cls(reference, cave_height, cave_depth)

    @classmethod
    def from_csv(cls, file_name: str):
        # You are required to implement this method
        
        df = pd.read_csv(file_name) #define data frame

        num_rows = df.shape[0]
        print('number of rows ', num_rows)
        
        #Extract columns from .csv file
        reference = df['reference'].to_numpy()
        cave_height = df['cave_height'].to_numpy()
        cave_depth = df['cave_depth'].to_numpy() 

        # Step 3: Return a new instance of the Mission class
        return cls(reference=reference, cave_height=cave_height, cave_depth=cave_depth)


        


class ClosedLoop:
    def __init__(self, plant: Submarine, controller):
        self.plant = plant #i.e the submarine
        self.controller = controller

    def simulate(self,  mission: Mission, disturbances: np.ndarray) -> Trajectory:

        T = len(mission.reference)
        print("T is ", T)
        if len(disturbances) < T:
            raise ValueError("Disturbances must be at least as long as mission duration")
        
        positions = np.zeros((T, 2))
        actions = np.zeros(T)
        self.plant.reset_state()

        for t in range(T):
            positions[t] = self.plant.get_position()
            current_depth[t] = self.plant.get_depth()
            reference_depth[t] = mission.reference[t] #get desired reference depth from the mission

            # Call your controller here
            actions[t] = self.controller.compute_action(current_depth, reference_depth)

            self.plant.transition(actions[t], disturbances[t])

        return Trajectory(positions)
        
    def simulate_with_random_disturbances(self, mission: Mission, variance: float = 0.5) -> Trajectory:
        disturbances = np.random.normal(0, variance, len(mission.reference))
        return self.simulate(mission, disturbances)

# Test calling the from_csv method
if __name__ == "__main__":
    # Use the absolute path to the mission.csv file
    mission_file = "/Users/paras/Library/CloudStorage/OneDrive-Nexus365/Oxford/Y3/B1/Scientific Coding/Scientific Coding Practical/B1 Coding Practical MT24/data/mission.csv"
    
    # Call the from_csv method to load the mission data
    mission = Mission.from_csv(mission_file)
    
    # Print out the loaded data to verify
    print("Reference Depths:", mission.reference)
    print("Cave Heights:", mission.cave_height)
    print("Cave Depths:", mission.cave_depth)