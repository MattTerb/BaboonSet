import sys
import numpy as np
import sympy as sp


def get_markers():
    return ['snout', 'left_eye', 'right_eye', 'neck_base', 
            'spine_1','spine_2','spine_3','spine_4', 'tail_base', 
            'tail_1','tail_2','tail_end',
            'right_shoulder', 'right_elbow', 'right_wrist',
            'left_shoulder', 'left_elbow', 'left_wrist',
            'right_hip', 'right_knee', 'right_ankle',
            'left_hip', 'left_knee', 'left_ankle'
           ] 



def get_skeleton():
    return [['snout', 'left_eye'], ['snout', 'right_eye'], ['snout', 'neck_base'], 
            ['left_eye', 'neck_base'], ['right_eye', 'neck_base'], 
            ['neck_base', 'spine_1'],['spine_1', 'spine_2'],['spine_2', 'spine_3'],
            ['spine_3', 'spine_4'], ['spine_4', 'tail_base'], ['tail_base', 'tail_1'], 
            ['tail_1', 'tail_2'],['tail_2', 'tail_end'],['neck_base','right_shoulder'], 
            ['right_shoulder', 'right_elbow'], ['right_elbow', 'right_wrist'],
            ['neck_base', 'left_shoulder'], ['left_shoulder', 'left_elbow'], 
            ['left_elbow', 'left_wrist'], 
            ['tail_base', 'right_hip'], ['right_hip', 'right_knee'],
            ['right_knee','right_ankle'], 
            ['tail_base', 'left_hip'], ['left_hip', 'left_knee'],
            ['left_knee','left_ankle'],['left_hip', 'right_hip'],
            ['left_shoulder','right_shoulder']
           ] 
 



def get_pose_params():
    states = ['x_0', 'y_0', 'z_0',         # head position in inertial
              'phi_0', 'theta_0', 'psi_0', # head rotation in inertial
              'phi_1', 'theta_1', 'psi_1', # neck
              'theta_2',                   # front torso
              'theta_3', 'psi_3',          # spine_1
              'theta_4', 'psi_4',          # spine_2
              'theta_5', 'psi_5',          # spine_3
              'theta_6', 'psi_6',          # back torso
              'theta_7', 'psi_7',          # tail_base
              'theta_8', 'psi_8',          # tail_1
              'theta_9', 'psi_9',          # tail_2
              'theta_10', 'theta_11',      # left_shoulder, left_elbow
              'theta_12', 'theta_13',       # right_shoulder, right_elbow
              'theta_14', 'theta_15',      # left_hip, left_knee
              'theta_16', 'theta_17'      # right_hip, right_knee
             ] 
    return dict(zip(states ,range(len(states))))


def get_3d_marker_coords(x):
    """Returns either a numpy array or a sympy Matrix of the 3D marker coordinates (shape Nx3) for a given state vector x.
    """
    func = sp.Matrix if isinstance(x[0], sp.Expr) else np.array
    idx = get_pose_params()
    
    # rotations
    RI_0 = rot_z(x[idx['psi_0']]) @ rot_x(x[idx['phi_0']]) @ rot_y(x[idx['theta_0']]) # head
    R0_I = RI_0.T
    RI_1 = rot_z(x[idx['psi_1']]) @ rot_x(x[idx['phi_1']]) @ rot_y(x[idx['theta_1']]) @ RI_0 # neck
    R1_I = RI_1.T
    RI_2 = rot_y(x[idx['theta_2']]) @ RI_1 # front torso
    R2_I = RI_2.T
    RI_3 = rot_z(x[idx['psi_3']])@ rot_y(x[idx['theta_3']]) @ RI_2 # spine 1
    R3_I = RI_3.T
    RI_4 = rot_z(x[idx['psi_4']]) @ rot_y(x[idx['theta_4']]) @ RI_3 # spine 2
    R4_I = RI_4.T
    RI_5 = rot_z(x[idx['psi_5']]) @ rot_y(x[idx['theta_5']]) @ RI_4 # spine 3
    R5_I = RI_5.T
    RI_6 = rot_z(x[idx['psi_6']]) @ rot_y(x[idx['theta_6']]) @ RI_5 # back torso
    R6_I = RI_6.T
    RI_7 = rot_z(x[idx['psi_7']]) @ rot_y(x[idx['theta_7']]) @ RI_6 # tail base
    R7_I = RI_7.T
    RI_8 = rot_z(x[idx['psi_8']]) @ rot_y(x[idx['theta_8']]) @ RI_7 # tail 1
    R8_I = RI_8.T
    RI_9 = rot_z(x[idx['psi_9']]) @ rot_y(x[idx['theta_9']]) @ RI_8 # tail 2
    R9_I = RI_9.T
    RI_10 = rot_y(x[idx['theta_10']]) @ RI_2 # left shoulder
    R10_I = RI_10.T
    RI_11 = rot_y(x[idx['theta_11']]) @ RI_10 # left elbow
    R11_I = RI_11.T
    RI_12 = rot_y(x[idx['theta_12']]) @ RI_2 # right shoulder
    R12_I = RI_12.T
    RI_13 = rot_y(x[idx['theta_13']]) @ RI_12 # right elbow
    R13_I = RI_13.T
    RI_14 = rot_y(x[idx['theta_14']]) @ RI_6 # left hip
    R14_I = RI_14.T
    RI_15 = rot_y(x[idx['theta_15']]) @ RI_14 # left knee
    R15_I = RI_15.T
    RI_16 = rot_y(x[idx['theta_16']]) @ RI_6 # right hip
    R16_I = RI_16.T
    RI_17 = rot_y(x[idx['theta_17']]) @ RI_16 # right knee
    R17_I = RI_17.T

    # positions
    p_head          = func([x[idx['x_0']], x[idx['y_0']],x[idx['z_0']]])

    p_left_eye      = p_head         + R0_I  @ func([0, 0.021, 0])
    p_right_eye     = p_head         + R0_I  @ func([0, -0.021, 0])
    p_snout         = p_head         + R0_I  @ func([0.002, 0, -0.069])

    p_neck_base     = p_head         + R1_I  @ func([-0.129, 0, -0.024])
    p_spine_1       = p_neck_base    + R2_I  @ func([-0.05, 0, 0.05])
    p_spine_2       = p_spine_1      + R3_I  @ func([-0.096, 0, -0.05])
    p_spine_3       = p_spine_2      + R4_I  @ func([-0.102, 0, -0.009])
    p_spine_4       = p_spine_3      + R5_I  @ func([-0.104, 0, -0.037])

    p_tail_base     = p_spine_4      + R6_I  @ func([-0.057, 0, -0.019])
    p_tail_1        = p_tail_base    + R7_I  @ func([-0.067, 0, 0.038])
    p_tail_2        = p_tail_1       + R8_I  @ func([-0.024, 0, -0.104])
    p_tail_end      = p_tail_2       + R9_I  @ func([0.021, 0, -0.135])

    p_left_shoulder = p_neck_base    + R2_I  @ func([-0.018, 0.009, -0.115])
    p_left_elbow    = p_left_shoulder+ R10_I @ func([-0.01, 0, -0.122])
    p_left_wrist    = p_left_elbow   + R11_I @ func([0.051, 0, -0.176])

    p_right_shoulder= p_neck_base    + R2_I  @ func([-0.018, -0.009, -0.115])
    p_right_elbow   = p_right_shoulder+ R12_I@ func([-0.01, 0, -0.122])
    p_right_wrist   = p_right_elbow   + R13_I@ func([0.051, 0, -0.176])

    p_left_hip      = p_spine_4      + R6_I  @ func([-0.038, 0.008, -0.087])
    p_left_knee     = p_left_hip     + R14_I @ func([0.054, 0, -0.115])
    p_left_ankle    = p_left_knee    + R15_I @ func([-0.113, 0, -0.124])

    p_right_hip     = p_spine_4      + R6_I  @ func([-0.038, -0.008, -0.087])
    p_right_knee    = p_right_hip    + R16_I @ func([0.054, 0, -0.115])
    p_right_ankle   = p_right_knee   + R17_I @ func([-0.113, 0, -0.124])

    return func([p_left_eye.T, p_right_eye.T, p_snout.T,
                 p_neck_base.T, p_spine_1.T, p_spine_2.T,
                 p_spine_3.T, p_spine_4.T, p_tail_base.T,
                 p_tail_1.T, p_tail_2.T, p_tail_end.T,
                 p_left_shoulder.T, p_left_elbow.T, p_left_wrist.T,
                 p_right_shoulder.T, p_right_elbow.T, p_right_wrist.T,
                 p_left_hip.T, p_left_knee.T, p_left_ankle.T,
                 p_right_hip.T, p_right_knee.T, p_right_ankle.T
                ])


def redescending_loss(err, a, b, c):
    # outlier rejecting cost function
    def func_step(start, x):
        return 1/(1+np.e**(-1*(x - start)))

    def func_piece(start, end, x):
        return func_step(start, x) - func_step(end, x)
    
    e = abs(err)
    cost = 0.0
    cost += (1 - func_step(a, e))/2*e**2
    cost += func_piece(a, b, e)*(a*e - (a**2)/2)
    cost += func_piece(b, c, e)*(a*b - (a**2)/2 + (a*(c-b)/2)*(1-((c-e)/(c-b))**2))
    cost += func_step(c, e)*(a*b - (a**2)/2 + (a*(c-b)/2))
    return cost


def global_positions(R_arr, t_arr):
    "Returns a vector of camera position vectors in the world frame"
    R_arr = np.array(R_arr).reshape((-1, 3, 3))
    t_arr = np.array(t_arr).reshape((-1, 3, 1))
    
    positions = []
    assert R_arr.shape[0]==t_arr.shape[0], 'Number of cams in R_arr do not match t_arr'
    for r, t in zip(R_arr, t_arr):
        pos = -r.T @ t
        positions.append(pos)
        
    return np.array(positions, dtype=np.float32)


def rotation_matrix_from_vectors(u,v):
    """ Find the rotation matrix that aligns u to v
    :param u: A 3D "source" vector
    :param v: A 3D "destination" vector
    :return mat: A transform matrix (3x3) which when applied to u, aligns it with v.
    """
    # https://stackoverflow.com/questions/36409140/create-a-rotation-matrix-from-2-normals
    # Suppose you want to write the rotation that maps a vector u to a vector v.
    # if U and V are their unit vectors then W = U^V (cross product) is the axis of rotation and is an invariant
    # Let M be the associated matrix.
    # We have finally: (V,W,V^W) = M.(U,W,U^W)

    U = (u/np.linalg.norm(u)).reshape(3)
    V = (v/np.linalg.norm(v)).reshape(3)
    
    W = np.cross(U, V)
    A = np.array([U, W, np.cross(U, W)]).T
    B = np.array([V, W, np.cross(V, W)]).T
    return np.dot(B, np.linalg.inv(A))


def rot_x(x):
    if isinstance(x, sp.Expr):
        c = sp.cos(x)
        s = sp.sin(x)
        func = sp.Matrix
    else:
        c = np.cos(x)
        s = np.sin(x)
        func = np.array
    return func([[1, 0, 0],
                 [0, c, s],
                 [0, -s, c]])


def rot_y(y):
    if isinstance(y, sp.Expr):
        c = sp.cos(y)
        s = sp.sin(y)
        func = sp.Matrix
    else:
        c = np.cos(y)
        s = np.sin(y)
        func = np.array
    return func([[c, 0, -s],
                 [0, 1, 0],
                 [s, 0, c]])


def rot_z(z):
    if isinstance(z, sp.Expr):
        c = sp.cos(z)
        s = sp.sin(z)
        func = sp.Matrix
    else:
        c = np.cos(z)
        s = np.sin(z)
        func = np.array
    return func([[c, s, 0],
                 [-s, c, 0],
                 [0, 0, 1]])


# https://stackoverflow.com/questions/14906764/how-to-redirect-stdout-to-both-file-and-console-with-scripting
class Logger:

    def __init__(self, out_fpath):
        self.terminal = sys.stdout
        self.logfile = open(out_fpath, 'w', buffering=1)

    def write(self, message):
        self.terminal.write(message)
        self.logfile.write(message)

    def flush(self):
        # this flush method is needed for python 3 compatibility.
        # this handles the flush command by doing nothing.
        # you might want to specify some extra behavior here.
        pass
