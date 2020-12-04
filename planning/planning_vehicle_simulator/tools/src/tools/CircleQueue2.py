#!/usr/bin/env python
# encoding: utf-8
# 如果觉得不错，可以推荐给你的朋友！http://tool.lu/pyc
import multiprocessing as mp

class MyQueue(object):
    
    def __init__(self, size):
        self.size = size
        self.mg = mp.Manager()
        self.queue = self.mg.list()#列表
        self.pointer = mp.Value('i', -1)#定义共享内存pointer

    def inQueue(self, n):
        self.pointer.value = (self.pointer.value + 1) % self.size
        if self.isFull():#如果队列满了
            self.queue[self.pointer.value] = n
        else:
            self.queue.append(n)

    def outQueue(self):
        if self.isEmpty():
            return -1
        return self.queue[self.pointer.value]

    def delete(self, n):
        element = self.queue[n]
        self.queue.remove(element)

    def isEmpty(self):
        if len(self.queue) == 0:
            return True

    def isFull(self):
        if len(self.queue) == self.size:
            return True


