from sympy import symbols, cos, sin, pi, sqrt, atan2, Matrix, simplify

def pose(theta, alpha, a, d):
    T = Matrix([
        [cos(theta), -sin(theta), 0, a],
        [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha)],
        [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha)],
        [0, 0, 0, 1]
    ])
    return simplify(T)

def forward_kin(q1, q2, q3, q4, q5, q6):
    d90 = pi/2
    T0g = pose(q1, 0, 0, 0.75) * pose(q2 - d90, -d90, 0.35, 0) * pose(q3, 0, 1.25, 0) \
          * pose(q4, -d90, -0.054, 1.5) * pose(q5, d90, 0, 0) * pose(q6, -d90, 0, 0) * pose(0, 0, 0, 0.303)
    px, py, pz = T0g[0, 3], T0g[1, 3], T0g[2, 3]
    return float(px), float(py), float(pz)

def get_angles(px, py, pz, roll, pitch, yaw):
    q1 = atan2(py, px)
    d = sqrt(px**2 + py**2)
    q2 = atan2(pz - 0.75, d - 0.35) - pi/4
    q3 = -atan2(pz - 0.75, d - 0.35) / 2
    q4, q5, q6 = roll, pitch, yaw
    return q1.evalf(), q2.evalf(), q3.evalf(), q4, q5, q6