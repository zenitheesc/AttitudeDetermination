#!/usr/bin/python3

from scipy.spatial.transform import Rotation as R
import numpy as np


def rotSensor(xyz, v):
    rot = R.from_euler('xyz', [-xyz[0], -xyz[1], -xyz[2]], degrees=True)
    M = np.matrix(rot.as_matrix())
    return np.array(np.matmul(M, v))


angles = [0.,  0., 180.]
print("Expected:  ", R.from_euler('xyz', angles, degrees=True).as_quat(), '\n')


v_1i = np.array([0.925417, -0.163176, -0.342020])
v_2i = np.array([-0.378522, -0.440970, -0.813798])

v_1i = np.array([1., 0., 0.])
v_2i = np.array([0., 0., -1.])
v_1b = np.array([0.925417, -0.163176, -0.342020])#rotSensor(angles, v_1i)
v_2b = np.array([-0.37852, -0.440970, -0.813798])#rotSensor(angles, v_2i)

W = [.5, .5]

quats = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
dety = [0, 0, 0, 0]

rotations = ['x', 'y', 'z', '0']
for RT in rotations:
    B = np.matrix(W[0]*(np.outer(v_1b, v_1i)) + W[1]*(np.outer(v_2b, v_2i)))
    if (RT == 'x'):
        print("Rotating on the x axis")
        for i in range(3):
            B[i, 1] = -B[i, 1]
            B[i, 2] = -B[i, 2]

    if (RT == 'y'):
        print("Rotating on the y axis")
        for i in range(3):
            B[i, 0] = -B[i, 0]
            B[i, 2] = -B[i, 2]

    if (RT == 'z'):
        print("Rotating on the z axis")
        for i in range(3):
            B[i, 0] = -B[i, 0]
            B[i, 1] = -B[i, 1]

    S = B + B.transpose()
    S = np.matrix(S)
    sigma = float(B.trace())

    Z = np.matrix([(B[1, 2]-B[2, 1]), (B[2, 0]-B[0, 2]), (B[0, 1]-B[1, 0])])
    lambda_ = sum(W)
    k = float((np.linalg.inv(S).T * np.linalg.det(S)).transpose().trace())
    delta = np.linalg.det(S)
    a = float(sigma ** 2 - k)
    b = float(sigma ** 2 + Z * Z.transpose())
    c = float(delta + Z * S*Z.transpose())
    d = float(Z * (S ** 2)*Z.transpose())

    def f(x): return x ** 4 - (a + b) * x**2 - c*x + (a*b + c*sigma - d)

    def df(x): return 4*x ** 3 - 2*(a + b)*x - c

    lambda_ = lambda_ - f(lambda_)/df(lambda_)
    # lambda_ = lambda_ - f(lambda_)/df(lambda_)

    __ = float(lambda_+sigma) * np.eye(3)
    Y = __ - S
    _ = np.linalg.inv(Y)
    crp = np.matmul(_, Z.transpose())
    crp = np.array(crp.transpose()[0])

    g = np.append(crp, np.linalg.norm(crp)**2)
    g = np.array(g)

    d = 1/np.sqrt(np.linalg.norm(crp)**2)
    # print("det(Y) = ", np.linalg.det(Y), '\t', "d =", d)

    q = np.array(list(map(lambda x: x*d, crp)))
    q = np.append(q, d)
    q = np.array(q)
    n = np.sqrt(sum(map(lambda x: x**2, q)))
    q = np.array(list(map(lambda x: x/n, q)))
    print("Still with rotation: ", q)

    if (RT == 'x'):
        print("Undoing rotation on the x axis")
        q = np.array([q[3], -q[2], q[1], -q[0]])
        quats[0] = q
        dety[0] = np.linalg.det(Y)

    if (RT == 'y'):
        print("Undoing rotation on the y axis")
        q = np.array([q[2], q[3], -q[0], -q[1]])
        quats[1] = q
        dety[1] = np.linalg.det(Y)

    if (RT == 'z'):
        print("Undoing rotation on the z axis")
        q = np.array([-q[1], q[0], q[3], -q[2]])
        quats[2] = q
        dety[2] = np.linalg.det(Y)

    if (RT == '0'):
        quats[3] = q
        dety[3] = np.linalg.det(Y)

    n = np.sqrt(sum(map(lambda x: x**2, q)))
    q = np.array(list(map(lambda x: x/n, q)))

    def euler(q):
        phi = 180.0/np.pi * \
            np.arctan2(2 * (q[3]*q[0] - q[1]*q[2]),
                       1 - 2 * (q[0]*q[0] + q[1]*q[1]))
        tetha = 180.0/np.pi * np.arcsin(2 * (q[3]*q[1] + q[2] * q[0]))
        psi = 180.0/np.pi * \
            np.arctan2(2 * (q[3]*q[2] - q[0]*q[1]),
                       1 - 2 * (q[1]*q[1] + q[2]*q[2]))

        return [round(phi, 5), round(tetha, 5), round(psi, 5)]

    r = R.from_quat(q)
    print(r.as_euler('XYZ', degrees=True))
    R.from_quat(q).as_euler('XYZ', degrees=True)
    print("After undoing: ", q)
    print(euler(q))
    print("")

m = 0
for i in range(len(dety)):
    if(dety[i] > dety[m]):
        m = i
print("Calculated:", quats[m])
print(euler(quats[m]))
