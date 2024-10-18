"""
Creating a simple PD controller which computes the control based on the error signal
"""

class PDController:
    def __init__(self, kp: float, kd: float):
    
        self.kp = kp  # Proportional gain
        self.kd = kd  # Derivative gain
        self.previous_error = 0.0  # Store the error from the previous time step
    
    def compute_action(self, current_depth: float, reference_depth: float) -> float:
        """
        Compute the controller action
        """
        # Calculate the error 
        error = reference_depth - current_depth
        
        # Calculate the discrete time derivative (change in error)
        derivative = error - self.previous_error
        
        # Control action is proportional to the error and derivative of the error
        action = self.kp * error + self.kd * derivative
        
        # Update previous error for the next time step
        self.previous_error = error
        
        return action