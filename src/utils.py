import numpy as np

def rotz_matrix(angle):
    """
    Rotation matrix around Z axis
    @param angle in radians
    @return 4x4 numpy array
    """
    c = np.cos(angle)
    s = np.sin(angle)
    rz = np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    return rz

def rotx_matrix(angle):
    """
    Rotation matrix around X axis
    @param angle: in radians
    @return 4x4 numpy array
    """
    c = np.cos(angle)
    s = np.sin(angle)
    rx = np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])
    return rx

def roty_matrix(angle):
    """
    Rotation matrix around Y axis
    @param angle: in radians
    @return 4x4 numpy array
    """
    c = np.cos(angle)
    s = np.sin(angle)
    rz = np.array([[c, 0, s, 0], [0, 1, 0, 0], [0, -s, c, 0], [0, 0, 0, 1]])
    return rz

def trans_matrix(displacement):
    """
    Traslation matrix using the 3D displacement vector
    @param displacement: 3D vector indicating the translation in each axis
    @return 4x4 numpy array
    """
    x = displacement[0]
    y = displacement[1]
    z = displacement[2]
    t = np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])
    return t
