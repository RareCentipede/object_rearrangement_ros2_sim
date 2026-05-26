import random
import uuid
import numpy as np

def generate_random_tamp_configs(num_objects: int, 
                                 workspace_bounds: tuple = ((-6.0, 6.0), (-6.0, 6.0)), 
                                 min_dist: float = 0.6) -> tuple:
    """
    Generates random initialization and goal configuration dictionaries.
    
    Args:
        num_objects: Number of dynamic blocks to generate.
        workspace_bounds: ((min_x, max_x), (min_y, max_y)) for random placement.
        min_dist: Minimum Euclidean distance between object centers to avoid overlapping.
        
    Returns:
        tuple: (init_dict, goal_dict)
    """
    (min_x, max_x), (min_y, max_y) = workspace_bounds
    
    # --- 1. Helper function to sample non-overlapping coordinates ---
    def sample_positions(count):
        positions = []
        attempts = 0
        max_attempts = count * 200  # Prevent infinite loops if workspace is too small
        
        while len(positions) < count and attempts < max_attempts:
            attempts += 1
            # Generating flat on the ground (Z=0.0)
            candidate = np.array([random.uniform(min_x, max_x), 
                                  random.uniform(min_y, max_y), 
                                  0.0])
            
            # Check distance against all previously accepted positions
            if all(np.linalg.norm(candidate - p) >= min_dist for p in positions):
                positions.append(candidate)
                
        if len(positions) < count:
            raise ValueError(f"Workspace too crowded! Could only place {len(positions)}/{count} objects. "
                             f"Increase workspace bounds or decrease min_dist.")
        return [p.tolist() for p in positions]

    # Sample all positions needed for Init state (Objects + 1 Static Block + 1 Robot)
    init_positions = sample_positions(num_objects + 2)
    # Sample all positions needed for Goal state (Objects)
    goal_positions = sample_positions(num_objects)
    
    init_dict = {}
    goal_dict = {}
    
    # --- 2. Populate Initialization Configurations ---
    # Add one fixed static anchor block
    init_dict["block1"] = {
        "type": "static",
        "position": init_positions[0],
        "orientation": [0.0, 0.0, 0.0],
        "color": [0.0, 0.0, 0.0],
        "size": 0.3
    }
    
    # Add dynamic blocks
    for i in range(2, num_objects + 2):
        block_name = f"block{i}"
        init_dict[block_name] = {
            "type": "dynamic",
            "position": init_positions[i-1],
            "orientation": [0.0, 0.0, 0.0],
            # Random RGB color
            "color": [round(random.random(), 2) for _ in range(3)],
            "size": 0.3
        }
        
    # Add Robot
    init_dict["robot"] = {
        "type": "robot",
        "position": init_positions[-1],
        "orientation": [0.0, 0.0, 0.0]
    }
    
    # --- 3. Populate Goal Configurations ---
    # Static block stays where it is
    goal_dict["block1"] = {
        "position": init_dict["block1"]["position"],
        "orientation": [0.0, 0.0, 0.0]
    }
    
    # Dynamic blocks get scrambled to new target locations
    for i in range(2, num_objects + 2):
        block_name = f"block{i}"
        goal_dict[block_name] = {
            "position": goal_positions[i-2],
            "orientation": [0.0, 0.0, 0.0]
        }
        
    return init_dict, goal_dict

# --- Example Usage ---
if __name__ == "__main__":
    import pprint
    
    # Generate random configuration for 4 blocks (plus 1 static anchor)
    init_cfg, goal_cfg = generate_random_tamp_configs(num_objects=4)
    
    print("--- GENERATED INITIAL DICTIONARY ---")
    pprint.pprint(init_cfg, sort_dicts=False)
    
    print("\n--- GENERATED GOAL DICTIONARY ---")
    pprint.pprint(goal_cfg, sort_dicts=False)