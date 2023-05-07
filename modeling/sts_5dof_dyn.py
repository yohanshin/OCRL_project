# name: sts_5dof_dyn.py
# description: Dynamics of the 5-DOF STS model
# author: Vu Phan
# date: 2023/04/14


from sympy import *
from sympy.physics.mechanics import *
from sympy.physics.vector import *


# Parameters
L1, L2, L3 = symbols('L1, L2, L3') # segment length
r1, r2, r3 = symbols('r1, r2, r3') # segment CoM length
m1, m2, m3 = symbols('m1, m2, m3') # segment mass
I1, I2, I3 = symbols('I1, I2, I3') # segment moment of inertia
g = symbols('g') # gravitational acceleration

# Reference frames
N = ReferenceFrame('N') # global
A = ReferenceFrame('A') # trunk
B = ReferenceFrame('B') # thigh
C = ReferenceFrame('C') # shank

# Dynamic variables
Q1, Q2, Q3, Q4, Q5 = dynamicsymbols('Q1, Q2, Q3, Q4, Q5') # generalized coordinates
V1, V2, V3, V4, V5 = dynamicsymbols('V1, V2, V3, V4, V5') # generalized speeds
tau1, tau2 = dynamicsymbols('tau1, tau2') # joint moments
Fx1, Fy1   = dynamicsymbols('Fx1, Fy1')   # GRFs
Fx2, Fy2   = dynamicsymbols('Fx2, Fy2')   # hip seating forces 

# Origin of the global reference
No = Point('No')

# CoM
Ao = Point('Ao') # trunk CoM
Bo = Point('Bo') # thigh CoM
Co = Point('Co') # shank CoM

# Joints
P1 = Point('P1') # head
P2 = Point('P2') # hip
P3 = Point('P3') # knee
P4 = Point('P4') # ankle

# Frame orientation
A.orient(N, 'Axis', (Q3, N.z))
B.orient(A, 'Axis', (Q4, A.z))
C.orient(B, 'Axis', (Q5, B.z))

# Joint location
P2.set_pos(No, Q1*N.x + Q2*N.y)
P1.set_pos(P2, +L1*A.x)
P3.set_pos(P2, -L2*B.x)
P4.set_pos(P3, -L3*C.x)

# CoM location
Ao.set_pos(P2, +r1*A.x)
Bo.set_pos(P3, +r2*B.x)
Co.set_pos(P4, +r3*C.x)

Cq = [V1 - Q1.diff(), 
      V2 - Q2.diff(), 
      V3 - Q3.diff(), 
      V4 - Q4.diff(), 
      V5 - Q5.diff()]

# Angular velocity
A.set_ang_vel(N, V3*N.z)
B.set_ang_vel(A, V4*N.z)
C.set_ang_vel(B, V5*N.z)

# Linear velocity
No.set_vel(N, 0)
P2.set_vel(N, V1*N.x + V2*N.y) # fix hip so no moving
Ao.v2pt_theory(P2, N, A)
P1.v2pt_theory(P2, N, A) # knee
Bo.v2pt_theory(P2, N, B)
P3.v2pt_theory(Bo, N, B) # hip
Co.v2pt_theory(P3, N, C)
P4.v2pt_theory(Co, N, C) # top-of-head

# Define moments of inertia about mass centers
ID1 = inertia(A, 0, 0, I1)
ID2 = inertia(B, 0, 0, I2)
ID3 = inertia(C, 0, 0, I3)
CI1 = (ID1, Ao)
CI2 = (ID2, Bo)
CI3 = (ID3, Co)

# Define rigid bodies
Arb = RigidBody('Arb', Ao, A, m1, CI1)
Brb = RigidBody('Brb', Bo, B, m2, CI2)
Crb = RigidBody('Crb', Co, C, m3, CI3)

# Gravitational forces
Amg = (Ao, -m1*g*N.y)
Bmg = (Bo, -m2*g*N.y)
Cmg = (Co, -m3*g*N.y)

# Joint moments (only on knee and hip)
Atau = (A, tau1*N.z)
Btau = (B, -tau1*N.z + tau2*N.z)
Ctau = (C, tau2*N.z)

# GRFs
GRFx = (P4, Fx1*N.x)
GRFy = (P4, Fy1*N.y)

# Seat forces
SFx = (P2, Fx2*N.x)
SFy = (P2, Fy2*N.y)

Q = [Q1, Q2, Q3, Q4, Q5]
V = [V1, V2, V3, V4, V5]
kane = KanesMethod(N, Q, V, Cq)

forces = [Amg, Bmg, Cmg, Atau, Btau, Ctau, GRFx, GRFy, SFx, SFy]
bodies = [Arb, Brb, Crb]
fr, frstar = kane.kanes_equations(bodies, forces)

# print('%Mass matrix')
# print(trigsimp(kane.mass_matrix))

# print()

# print('%Force matrix')
# print(trigsimp(kane.forcing))


print('% Contact element kinematics:')
Pc1 = P4.pos_from(No).express(N).simplify()
Vc1 = P4.vel(N).express(N).simplify()
Pc2 = P2.pos_from(No).express(N).simplify()
Vc2 = P2.vel(N).express(N).simplify()
print('Pc1x =',dot(Pc1,N.x))
print('Pc1y =',dot(Pc1,N.y))
print('Pc2x =',dot(Pc2,N.x))
print('Pc2y =',dot(Pc2,N.y))
print('Vc1x =',dot(Vc1,N.x))
print('Vc1y =',dot(Vc1,N.y))
print('Vc2x =',dot(Vc2,N.x))
print('Vc2y =',dot(Vc2,N.y))
print()
print('% Points of interest:')
print('P1x =',dot(P1.pos_from(No).express(N).simplify(),N.x))
print('P1y =',dot(P1.pos_from(No).express(N).simplify(),N.y))
print('P2x =',dot(P2.pos_from(No).express(N).simplify(),N.x))
print('P2y =',dot(P2.pos_from(No).express(N).simplify(),N.y))
print('P3x =',dot(P3.pos_from(No).express(N).simplify(),N.x))
print('P3y =',dot(P3.pos_from(No).express(N).simplify(),N.y))
print('P4x =',dot(P4.pos_from(No).express(N).simplify(),N.x))
print('P4y =',dot(P4.pos_from(No).express(N).simplify(),N.y))
print('Ao_x =',dot(Ao.pos_from(No).express(N).simplify(),N.x))
print('Ao_y =',dot(Ao.pos_from(No).express(N).simplify(),N.y))
print('Bo_x =',dot(Bo.pos_from(No).express(N).simplify(),N.x))
print('Bo_y =',dot(Bo.pos_from(No).express(N).simplify(),N.y))
print('Co_x =',dot(Co.pos_from(No).express(N).simplify(),N.x))
print('Co_y =',dot(Co.pos_from(No).express(N).simplify(),N.y))

