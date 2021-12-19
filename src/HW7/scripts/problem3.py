import sympy as sp

m1,m2,m3 = sp.symbols("m_1 m_2 m_3")

a1,a1c,a2,a2c,a3c = sp.symbols("a_1 a_{1c} a_{2} a_{2c} a_{3c}")

q1,q2,q3 = sp.symbols("q_1 q_2 q_3")
dq1,dq2,dq3 = sp.symbols("\Dot{q}_1 \Dot{q}_2 \Dot{q}_3")
I1,I2,I3 = sp.symbols("I_1 I_2 I_3")

Jv1 = sp.Matrix([[-a1c*sp.sin(q1),0,0],[a1c*sp.cos(q1),0,0],[0,0,0]])
Jv2 = sp.Matrix([[-(a1+q2+a2c)*sp.sin(q1),sp.cos(q1),0],
                 [(a1+q2+a2c)*sp.cos(q1),sp.sin(q1),0],
                 [0,0,0]])
Jv3 = sp.Matrix([[-(a1+q2+a2)*sp.sin(q1)-a3c*sp.sin(q1+q3),sp.cos(q1),-a3c*sp.sin(q1+q3)],
                 [(a1+q2+a2)*sp.cos(q1)+a3c*sp.cos(q1+q3),sp.sin(q1),a3c*sp.cos(q1+q3)],
                 [0,0,0]])

l1 = sp.Matrix([[0,0,0],[0,0,0],[0,0,I1]])
l2 = sp.Matrix([[0,0,0],[0,0,0],[0,0,I2]])
l3 = sp.Matrix([[0,0,0],[0,0,0],[0,0,I3]])

Jw1 = sp.Matrix([[0,0,0],[0,0,0],[1,0,0]])
Jw2 = sp.Matrix([[0,0,0],[0,0,0],[1,0,0]])
Jw3 = sp.Matrix([[0,0,0],[0,0,0],[1,0,1]])

Dq = m1*Jv1.T *Jv1 + m2*Jv2.T *Jv2 + m3*Jv2.T *Jv3  \
    + Jw1.T * l1 * Jw1 + Jw2.T * l2 * Jw2 + Jw3.T * l3 * Jw3

q = [q1,q2,q3]
dq = [dq1,dq2,dq3]
#print(sp.latex(sp.simplify(Dq)))
C = sp.Matrix([[0,0,0],[0,0,0],[0,0,0]])
for k in range(3):
    for j in range(3):
        expr = 0
        for i in range(3):
            expr += (1/2) * (sp.diff(Dq[k,j],q[i]) + sp.diff(Dq[k,i],q[j]) - sp.diff(Dq[i,j],q[k])) * dq[i]
        C[k,j] = expr

print(sp.latex(sp.simplify(C[2,2])))