import sympy as sp


def skew_matrix(o):
    return sp.Matrix([[0, -o[2], o[1]],
                         [o[2], 0, -o[0]],
                         [-o[1], o[0], 0]])


