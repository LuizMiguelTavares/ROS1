import numpy as np

def controller_bebop(pose, desired, gains, max_variables):
    g = 9.8 # gravity acceleration constant

    ##################################################################################################################
    '''
                                            ERROR DEALING
    '''              
    ##################################################################################################################                             
    if len(pose) != 18:
        raise ValueError(f"Pose size is incorrect. Expected size: 18. Got: {len(pose)}")
    
    if len(desired) != 18:
        raise ValueError(f"Desired Pose size is incorrect. Expected size: 18. Got: {len(desired)}")

    if len(gains) != 4:
        raise ValueError(f"Gains size is incorrect. Expected size: 4. Got: {len(gains)}")
    
    if len(max_variables) != 4:
        raise ValueError(f"Max Variables size is incorrect. Expected size: 4. Got: {len(max_variables)}")


    ##################################################################################################################
    '''
                                            VARIABLES  INPUT
    '''              
    ##################################################################################################################  
    # [x, y, z, phi, theta, psi, x', y', z', phi', theta', psi']
    x_drone, y_drone, z_drone, \
    phi_drone, theta_drone, psi_drone, \
    x_dot_drone, y_dot_drone, z_dot_drone, \
    phi_dot_drone, theta_dot_drone, psi_dot_drone, \
    x_ddot_drone, y_ddot_drone, z_ddot_drone, \
    phi_ddot_drone, theta_ddot_drone, psi_ddot_drone = pose

    x_desired, y_desired, z_desired, \
    phi_desired, theta_desired, psi_desired, \
    x_dot_desired, y_dot_desired, z_dot_desired, \
    phi_dot_desired, theta_dot_desired, psi_dot_desired, \
    x_ddot_desired, y_ddot_desired, z_ddot_desired, \
    phi_ddot_desired, theta_ddot_desired, psi_ddot_desired = desired
    
    kp = np.array([gains[0],     0     ],
                  [   0    ,   gains[1]])
    
    kd = np.array([gains[2],     0     ],
                  [   0    ,   gains[3]])
    
    # Max variables [theta_max, phi_max, z_dot_max, psi_dot_max]
    theta_max, phi_max, z_dot_max, psi_dot_max = max_variables


    ##################################################################################################################
    '''
                                            THETA AND PHI CONTROLLER
    '''
    ##################################################################################################################
    X_til     = np.array([x_desired - x_drone, y_desired - y_drone]).T
    X_dot_til = np.array([x_dot_desired - x_dot_drone, y_dot_desired - y_dot_drone]).T
    
    x_ddot_ref, y_ddot_ref = np.array([x_ddot_desired, y_ddot_desired]).T +  np.dot(kd, X_dot_til) + np.dot(kp, X_til)

    X_ddot_ref = np.array([x_ddot_ref, y_ddot_ref]).T

    normalization_theta_phi = np.array([[1/(g*theta_max),        0       ], 
                                        [       0       , 1/(g*phi_max)]])
    
    rotation_matrix_theta_phi = np.array([[np.cos(psi_drone) , np.sin(psi_drone)], 
                                          [-np.sin(psi_drone), np.cos(psi_drone)]])
    
    u_theta, u_phi = np.dot(normalization_theta_phi, np.dot(rotation_matrix_theta_phi, X_ddot_ref))


    ##################################################################################################################
    '''
                                            Z' AND PSI' CONTROLLER
    '''
    ##################################################################################################################    
    Z_PSI_til = np.array([z_desired - z_drone, psi_desired - psi_drone]).T

    z_dot_ref, psi_dot_ref = np.array([z_dot_desired, y_dot_desired]).T + np.dot(kp, Z_PSI_til)

    u_z_dot   = z_dot_ref/z_dot_max
    u_psi_dot = psi_dot_ref/psi_dot_max

    return u_theta, u_phi, u_z_dot, u_psi_dot