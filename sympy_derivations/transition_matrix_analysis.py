from sympy import *
from common_ops import *

# Symbols for standard process(F) matrix derivations

skew_gyro=Symbol('w_x')
R=Symbol('R')
I=Symbol('I')
acc=Symbol('a')
acc_skew=Symbol('a_x')
tau=Symbol('tau')

# Equation for phi is
# phi_dot=I + Integration(F*phi)

phi=MatrixSymbol('phi',6,6)
phi=Matrix(phi)

initial_phi=Matrix(eye(6))
initial_phi=initial_phi*I



F=Matrix(zeros(6))
F[0,0]=skew_gyro
F[0,1]=-I
F[2,0]=R*acc_skew
F[2,3]=-R
F[4,2]=I

#Closed form equation
#The transition matrix phi=e^(A*tau)*phi(0,0)
#phi(0,0)=Identity


# Dumb answer anything not Zero or Identity in this equation is what we have to analyze
exp=simplify(exp(F*tau)*initial_phi)
pprint(exp)

analyze=initial_phi+F*phi

#Set all the values to 0 that are zero in the exponent
zero_set=[]
for i in range(0,6):
    for j in range(0,6):
        if exp[i,j]==0:
            zero_set.append((phi[i, j], 0))
        #if exp[i,j]==I:
         #   zero_set.append((phi[i, j], I))
analyze=analyze.subs(zero_set)


pprint(simplify(analyze))

#ans=F*phi
#ans=simplify(ans)


# zero_set=[]
# for i in range(0,6):
#     for j in range(0,6):
#         # if i==j:
#         #     continue
#         # zero_set.append((phi[i,j],0))
#         if ans[i,j]==0:
#             zero_set.append((phi[i, j], 0))
#
#
# ans=ans.subs(zero_set)
# zero_set.clear()
#
# for i in range(0,6):
#     for j in range(0,6):
#         if i==j:
#             continue
#         zero_set.append((phi[i,j],0))
#
#
# ans=ans.subs(zero_set)
# pprint(simplify(exp(F*tau)*initial_phi))

