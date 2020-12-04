#!/usr/bin/env python 
# -*- coding: utf-8 -*-
# @File     : mpc_cvx.py
# @Project  : Sleipnir
# @Software : PyCharm
# @Author   : why
# @Email    : weihaoyuan2@126.com
# @Time     : 2020/7/21 下午5:05

import numpy as np
import pandas as pd
import math
from scipy import sparse
import cvxopt as cvx
from matplotlib import pyplot as plt
import osqp
import copy

class ModelPredictiveControl():
    def __init__(self):
        self.Np=60   #预测时域
        self.Nc=30   #控制时域
        self.Nx=5    #状态量个数
        self.Nu=2    #控制量个数
        self.Ny=3    #输出量个数
        self.L=2.6   #车辆前后轴距
        self.T=0.2   #采样间隔
        self.C=np.array([[1,0,0,0,0],
                         [0,1,0,0,0],
                         [0,0,1,0,0]])#C矩阵
        self.q=np.diag([70,70,10])                #状态量权值系数，分别对应x,y,heading
        self.Q=np.kron(np.eye((self.Np)),self.q)  #状态量权重矩阵
        self.r=np.diag([100,1])                   #控制量权值系数，分别对应速度增量和前轮转角
        self.R=np.kron(np.eye((self.Nc+1)),self.r)#控制量权重矩阵
        self.U=None
        self.traj=[] #参考轨迹
        self.degreesToRadiansScaleFactor = 180.0 / math.pi #转角度制系数

        #各项约束
        #minimize     1/2 x' * P * x + q' * x
        #subject to   l <= A * x <= u
        self.Umin=[-10.5,-0.436]                        #控制量约束下限
        self.Umax=[10,0.436]                            #控制量约束上限
        self.delta_Umin = np.array([[-0.1],[ -0.0082]])#控制增量约束下限
        self.delta_Umax=np.array([[0.1],[0.0082]])     #控制增量约束上限
        self.A_cons_cell=np.zeros((self.Nc+1,self.Nc+1,self.Nu,self.Nu))
        for i in range(self.Nc+1):
            for j in range(self.Nc+1):
                if not j>i:
                    self.A_cons_cell[i,j]=np.eye((self.Nu))
        self.A_cons=self.cell2mat(self.A_cons_cell)
        self.A_cons=np.vstack((self.A_cons,-self.A_cons,np.eye(((self.Nc+1)*self.Nu)),-np.eye(((self.Nc+1)*self.Nu))))
        #self.A_cons=np.vstack((self.A_cons,np.eye(((self.Nc+1)*self.Nu))))  #对应于约束方程的A
        self.delta_min_cons=np.kron(np.ones((self.Nc+1,1)),self.delta_Umin) #控制增量最小值矩阵
        self.delta_max_cons=np.kron(np.ones((self.Nc+1,1)),self.delta_Umax,)#控制增量最大值矩阵
        self.prob=osqp.OSQP()  #OSPQ求解器
        self.alreadySteup=False

        self._carsimExport=[]
        self._carsimImport=[]

        #参数绘图
        self.yaw_list=[]

        #测试用
        self.last_xi=True

    @property
    def carsimImport(self):
        return self._carsimImport.copy()

    @carsimImport.setter
    def carsimImport(self,value):
        self._carsimImport=[value[0]*3.6,value[1]*self.degreesToRadiansScaleFactor,value[1]*self.degreesToRadiansScaleFactor,0,0]

    @property
    def carsimExport(self):
        return self._carsimExport.copy()

    @carsimExport.setter
    def carsimExport(self,value):
        val=value.copy()
        val[2]=(val[2]+270)/self.degreesToRadiansScaleFactor
        val[3]=val[3]/3.6
        val[4]=val[4]/self.degreesToRadiansScaleFactor
        self._carsimExport=val

    def loadTraj(self,trajPath):
        traj=pd.read_csv(trajPath)
        for index,rows in traj.iterrows():
            self.traj.append({'x':rows['x'],'y':rows['y'],'velocity':rows['velocity'],
                              'heading':rows['heading'],'steer':rows['steer']})

    def matMulti(self,mat1,mat2,mat3=None,parameter=None):
        mat11=np.mat(mat1)
        mat22=np.mat(mat2)
        mat=None
        if mat3 is not None:
            mat33=np.mat(mat3)
            if parameter is None:
                mat=np.dot(np.dot(mat11,mat22),mat33)
            elif parameter == 'PTAP':
                mat=np.dot(np.dot(mat11.T,mat22),mat33)
            elif parameter == 'PAPT':
                mat=np.dot(np.dot(mat11,mat22),mat33.T)
            else:
                raise ValueError('parameter error')
        else:
            if parameter is None:
                mat=np.dot(mat11,mat22)
            elif parameter=='ATA':
                mat=np.dot(mat11.T,mat22)
            elif parameter=='AAT':
                mat=np.dot(mat11,mat22.T)
            elif parameter=='ATAT':
                mat = np.dot(mat11.T, mat22.T)
            else:
                raise ValueError('parameter error')
        if mat.shape == (1,1):
            return np.array(mat)[0][0]
        else:
            return np.array(mat)

    def cell2mat(self,cell):
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

    def matPower(self,mat,N):
        if mat.shape[0]!=mat.shape[1]:
            raise Exception("Inconsistent dimension of matrix!")
        elif N==0:
            return np.eye((mat.shape[0]))
        else:
            mat_out=np.eye(mat.shape[0])
            for i in range(N):
                mat_out=np.dot(mat_out,mat)
            return mat_out

    def predict(self,n):
        exportValue=self.carsimExport
        x=exportValue[0]
        y = exportValue[1]
        heading = exportValue[2]
        v = exportValue[3]
        steer = exportValue[4]
        #目标点的速度
        v_r=self.traj[n]['velocity']
        #目标点的前轮转向
        steer_r=self.traj[n]['steer']
        #目标点的航向角(弧度制)
        heading_r=self.traj[n]['heading']/self.degreesToRadiansScaleFactor
        #目标点横纵坐标
        x_r=self.traj[n]['x']
        y_r=self.traj[n]['y']
        #控制量初始化
        if self.U is None:
            self.U=[v-v_r,steer-steer_r]
        #运动学模型,对应公式4.6的矩阵A,B
        a=np.array([[1,0,-self.T*v_r*math.sin(heading_r)],
                    [0,1,self.T*v_r*math.cos(heading_r)],
                    [0,0,1]])
        b=np.array([[self.T*math.cos(heading_r),0],
                    [self.T*math.sin(heading_r),0],
                    [self.T*math.tan(steer_r)/self.L,self.T*v_r/(self.L*(math.cos(steer_r))**2)]])
        #取xi=[x;u]后新的状态空间表达式系数,对应公式4.10的矩阵A,B
        A=np.eye((self.Nx))
        A[:self.Ny,:self.Ny]=a.copy()
        A[:self.Ny,self.Ny:]=b.copy()
        B=np.zeros((self.Nx,self.Nu))
        B[:self.Ny,:]=b.copy()
        B[self.Ny:,:]=np.eye((self.Nu))
        #状态量,对应公式4.9
        xi=np.array([x-x_r,y-y_r,heading-heading_r,self.U[0],self.U[1]])
        if self.last_xi:
            print(xi-self.last_xi)
        self.yaw_list.append((heading-heading_r)*self.degreesToRadiansScaleFactor)
        #输出表达式系数，对应公式4.12的PSI和THETA
        PSI_cell=np.zeros((self.Np,1,self.Ny,self.Nx))
        THETA_cell=np.zeros((self.Np,self.Nc+1,self.Ny,self.Nu))
        for i in range(self.Np):
            PSI_cell[i][0]=self.matMulti(self.C,self.matPower(A,i+1))
            for j in range(self.Nc+1):
                if not j > i:
                    THETA_cell[i,j]=self.matMulti(self.C,self.matPower(A,i-j),B)
        THETA=self.cell2mat(THETA_cell)
        PSI=self.cell2mat(PSI_cell)
        #对应式3.16的H
        H=(self.matMulti(THETA,self.Q,THETA,parameter='PTAP')+self.R)/2
        #对应式3.14的E
        E=self.matMulti(PSI,xi,parameter='AAT')
        #对应式3.16的g.T
        f=np.ravel(2*self.matMulti(E,self.Q,THETA,parameter='PTAP'))
        #控制量最大值约束
        U_max_cons=np.kron(np.ones((self.Nc+1,1)),np.array([[self.Umax[0]-self.U[0]],[self.Umax[1]-self.U[1]]]))
        #控制量最小值约束
        U_min_cons=np.kron(np.ones((self.Nc+1,1)),np.array([[self.Umin[0]-self.U[0]],[self.Umin[1]-self.U[1]]]))
        #最小值约束矩阵
        min_cons=np.ravel(np.vstack((U_min_cons,self.delta_min_cons)))
        #最大值约束矩阵
        max_cons=np.ravel(np.vstack((U_max_cons,self.delta_max_cons)))
        cons=np.vstack((U_max_cons,-U_min_cons,self.delta_max_cons,-self.delta_min_cons))
        #输入各项参数到求解器中
        sol = cvx.solvers.qp(cvx.matrix(H), cvx.matrix(f.T), cvx.matrix(self.A_cons), cvx.matrix(cons))
        x = sol['x']
        #QP求解
        self.U=[self.U[0]+x[0],self.U[1]+x[1]]
        print('U'*50,self.U)
        Import=[self.U[0]+v_r,self.U[1]+steer_r]
        self.carsimImport=[Import[0],-Import[1]]


if __name__ == '__main__':
    test=ModelPredictiveControl()
    test.loadTraj('newTraj.csv')
    test.predict(0)
    print(test.carsimImport)