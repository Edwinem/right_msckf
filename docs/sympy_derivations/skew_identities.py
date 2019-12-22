from sympy import MatrixSymbol, Matrix,symbols,init_printing,pprint,simplify


def skew_matrix(o):
    return Matrix([[0, -o[2], o[1]],
                         [o[2], 0, -o[0]],
                         [-o[1], o[0], 0]])


init_printing()

R = MatrixSymbol('R', 3, 3)
R=Matrix(R)


a1, a2, a3 = symbols('a1 a2 a3')
a = Matrix([[a1, a2, a3]])
a=a.transpose()

sola=R*skew_matrix(a)

bloesch=skew_matrix(R*a)

bloesch_expanded=R*skew_matrix(a)*R.transpose()

#pprint(simplify(bloesch_expanded))

pprint(simplify(bloesch-bloesch_expanded))
