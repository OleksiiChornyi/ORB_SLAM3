import numpy as np

np.set_printoptions(suppress=True, precision=8)

def calculate_rotation_difference(R1, R2):
    # Calculate the inverse of R1
    R1_inv = np.linalg.inv(R1)
    
    # Calculate the difference: R_diff = R1_inv * R2
    R_diff = np.dot(R1_inv, R2)
    
    return R_diff

def calculate_R2(R1, R_diff):
    # Calculate R2 = R1 * R_diff
    R2 = np.dot(R1, R_diff)
    return R2

def calculate_t_diff(R1, R2, t1, t2):
    # Calculate the inverse of R1
    R1_inv = np.linalg.inv(R1)
    
    # Calculate the translation difference: t_diff = R1_inv * (t2 - t1)
    t_diff = np.dot(R1_inv, (t2 - t1))
    
    return t_diff
    
# Example: Define rotation matrices for both cameras
R_c1 = np.array([
    [ 0.9972638 ,  0.03654628,  0.06425942],
    [-0.0369175 ,  0.99930774,  0.00459865],
    [-0.06404688, -0.00695837,  0.99792263]
])

R_c2 = np.array([
    [ 0.99790448, -0.01275559,  0.06343453],
    [ 0.01312238,  0.99989948, -0.00536886],
    [-0.06335967,  0.00619002,  0.99797156]
])

t1 = np.array([0.0, 0.0, 0.0])
t2 = np.array([-0.234, 0.0, 0.0])

# Calculate the relative rotation matrix R_c1_c2
R_diff = calculate_rotation_difference(R_c1, R_c2)
print("Relative Rotation Matrix R_diff:")
print(R_diff)

print("")
print("calculate_t_diff T_diff:")
print(calculate_t_diff(R_c1, R_c2, t1, t2))