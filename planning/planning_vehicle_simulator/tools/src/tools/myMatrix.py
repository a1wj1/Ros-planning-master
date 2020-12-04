#!/usr/bin/env python 
# -*- coding: utf-8 -*-
# @File     : myMatrix.py
# @Project  : Sleipnir
# @Software : PyCharm
# @Author   : why
# @Email    : weihaoyuan2@126.com
# @Time     : 2020/7/14 上午9:31

import numpy as np
import cvxopt as cvx
def matrixPower( mat, N):
    if mat.shape[0] != mat.shape[1]:
        raise Exception("Inconsistent dimension of matrix!")
    else:
        mat_out = np.eye(mat.shape[0])
        for i in range(N):
            mat_out = np.dot(mat_out, mat)
        return mat_out


def quadprog( H, f, A_cons, b_cons, a1, a2, lb, ub):
    n = H.shape[1]
    P = H
    q = f
    G = np.vstack([-np.eye(n - 1), np.eye(n - 1)])
    G = np.hstack((G, np.zeros(((n - 1) * 2, 1))))
    # G=A_cons
    h = np.array(b_cons)
    sol = cvx.solvers.qp(cvx.matrix(P), cvx.matrix(q.T), cvx.matrix(G), cvx.matrix(h))
    x = sol['x']
    return x


def matMulti(mat1, mat2, mat3=None, parameter=None):
    mat11 = np.mat(mat1)
    mat22 = np.mat(mat2)
    mat = None
    if mat3 is not None:
        mat33 = np.mat(mat3)
        if parameter is None:
            mat = np.dot(np.dot(mat11, mat22), mat33)
        elif parameter == 'PTAP':
            mat = np.dot(np.dot(mat11.T, mat22), mat33)
        elif parameter == 'PAPT':
            mat = np.dot(np.dot(mat11, mat22), mat33.T)
    else:
        if parameter is None:
            mat = np.dot(mat11, mat22)
        elif parameter == 'ATA':
            mat = np.dot(mat11.T, mat22)
        elif parameter == 'AAT':
            mat = np.dot(mat11, mat22.T)
        elif parameter == 'ATAT':
            mat = np.dot(mat11.T, mat22.T)
    if mat.shape == (1, 1):
        return np.array(mat)[0][0]
    else:
        return np.array(mat)


def cell2mat(cell):
    mat_dim1 = cell.shape[0]
    mat_dim2 = cell.shape[1]
    h_mat = []
    for i in range(mat_dim1):
        h_cell = cell[i, 0]
        for j in range(mat_dim2 - 1):
            h_cell = np.hstack((h_cell, cell[i, j + 1]))
        h_mat.append(h_cell)
    mat = h_mat[0]
    for i in range(len(h_mat) - 1):
        mat = np.vstack((mat, h_mat[i + 1]))
    return mat