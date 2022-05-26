# Copied from https://github.com/PX4/ecl/commit/264c8c4e8681704e4719d0a03b848df8617c0863
# and modified for ArduPilot
from sys import pycache_prefix
from sympy import *
from code_gen import *
import numpy as np

# q: quaternion describing rotation from frame 1 to frame 2
# returns a rotation matrix derived form q which describes the same
# rotation
def quat2Rot(q):
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    # This form is the one normally used in flight dynamics and inertial navigation texts, eg
    # Aircraft Control and Simulation, Stevens,B.L, Lewis,F.L, Johnson,E.N, Third Edition, eqn 1.8-18
    # It does produce second order terms in the covariance prediction that can be problematic
    # with single precision processing.
    # It requires the quternion to be unit length.
    # Rot = Matrix([[q0**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
    #               [2*(q1*q2 + q0*q3), q0**2 - q1**2 + q2**2 - q3**2, 2*(q2*q3 - q0*q1)],
    #                [2*(q1*q3-q0*q2), 2*(q2*q3 + q0*q1), q0**2 - q1**2 - q2**2 + q3**2]])

    # This form removes q1 from the 0,0, q2 from the 1,1 and q3 from the 2,2 entry and results
    # in a covariance prediction that is better conditioned.
    # It requires the quaternion to be unit length and is mathematically identical
    # to the alternate form when q0**2 + q1**2 + q2**2 + q3**2 = 1
    # See https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
    Rot = Matrix([[1 - 2*(q2**2 + q3**2), 2*(q1*q2 - q0*q3)    , 2*(q1*q3 + q0*q2)    ],
                 [2*(q1*q2 + q0*q3)     , 1 - 2*(q1**2 + q3**2), 2*(q2*q3 - q0*q1)    ],
                 [2*(q1*q3-q0*q2)       , 2*(q2*q3 + q0*q1)    , 1 - 2*(q1**2 + q2**2)]])

    return Rot

def create_cov_matrix(i, j):
    if j >= i:
        # return Symbol("P(" + str(i) + "," + str(j) + ")", real=True)
        # legacy array format
        return Symbol("P[" + str(i) + "][" + str(j) + "]", real=True)
    else:
        return 0

def quat_mult(p,q):
    r = Matrix([p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3],
                p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2],
                p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1],
                p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0]])

    return r

def create_symmetric_cov_matrix(n):
    # define a symbolic covariance matrix
    P = Matrix(n,n,create_cov_matrix)

    for index in range(n):
        for j in range(n):
            if index > j:
                P[index,j] = P[j,index]

    return P

def generate_code():
    print('Starting code generation:')
    print('Creating symbolic variables ...')

    dt = symbols("dt", real=True)  # dt
    vx, vy = symbols("vn ve", real=True)
    v = Matrix([vx,vy])
    px, py, pz = symbols("pn pe pd", real=True)
    p = Matrix([px,py,pz])
    state = Matrix([v, p])
    v_new = v
    p_new = Matrix([px + vx * dt, py + vy * dt, pz])
    state_new = Matrix([v_new, p_new])

    print('Computing state propagation jacobian ...')
    A = state_new.jacobian(state)

    P = create_symmetric_cov_matrix(5)

    print('Computing covariance propagation ...')
    P_new = A * P * A.T

    for index in range(5):
        for j in range(5):
            if index > j:
                P_new[index,j] = 0

    print('Simplifying covariance propagation ...')
    P_new_simple = cse(P_new, symbols("PS0:400"), optimizations='basic')

    print('Writing covariance propagation to file ...')
    cov_code_generator = CodeGenerator("./generated/nav_terrain_covariance_prediction.cpp")
    cov_code_generator.print_string("Equations for covariance matrix prediction, without process noise!")
    cov_code_generator.write_subexpressions(P_new_simple[0])
    cov_code_generator.write_matrix(Matrix(P_new_simple[1]), "nextP", True, "[", "]")

    cov_code_generator.close()

    print('Code generation finished!')


if __name__ == "__main__":
    generate_code()
