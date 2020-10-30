# -*- coding: utf-8 -*-
"""
Created on Sun Sep 23 20:32:25 2018

@author: robot
"""

import os,sys
AbsolutePath = os.path.abspath(__file__)           
#将相对路径转换成绝对路径
SuperiorCatalogue = os.path.dirname(AbsolutePath)   
#相对路径的上级路径
BaseDir = os.path.dirname(SuperiorCatalogue)        
#在“SuperiorCatalogue”的基础上在脱掉一层路径，得到我们想要的路径。
if BaseDir in sys.path:
#    print('have been added')
    pass
else:
    sys.path.append(BaseDir)
    

import copy 
import numpy as np
from constructMethod.instance import Instance
#import sys 

class Solution(object):
    """
    Class representing a solution to a problem.
    """
    
    def __init__(self, instance : Instance ):
        """Creates a new solution for the given problem."""
        super(Solution, self).__init__()
        self._instance = instance
#        self.encode = np.zeros()
        self.objective = sys.float_info.max
        self.encode = np.zeros((self._instance.robNum,self._instance.taskNum),dtype =int)
        self.encode[:][:] = sys.maxsize
#        self.variables = FixedLengthArray(problem.nvars)
#        self.objectives = FixedLengthArray(problem.nobjs)
#        self.constraints = FixedLengthArray(problem.nconstrs)
#        self.constraint_violation = 0.0
#        self.evaluated = False
    def evaluate(self):
        """Evaluates this solution."""
        makespan = self._instance.evaluate(self.encode)
        self.objective = makespan
        return makespan
        
    def __repr__(self):
        return self.__str__()
        
    def __str__(self):        
        return     "Solution encode =\n " + str(self.encode) + '\n objective  = ' + str(self.objective) + ' \n instance = ' + str(self._instance)
#    "Solution[" + ",".join(list(map(str, self.variables))) + "|" + ",".join(list(map(str, self.objectives))) + "|" + str(self.constraint_violation) + "]"
    
    def __deepcopy__(self, memo):
        """Overridden to avoid cloning the problem definition."""
        result = Solution(self._instance)
        
        memo[id(self)] = result
        
        for k, v in self.__dict__.items():
            if k != "_instance":
                setattr(result, k, copy.deepcopy(v, memo))                
        return result
    def __getitem__(self,index):
        return self.encode[index[0]][index[1]]
    def __setitem__(self,key,value):
        self.encode[key[0]][key[1]]  = value
        return self.encode
    def __eq__(self,other):
#        print((self.encode == other.encode).all())
        if (self.encode == other.encode).all():
            if self._instance == other._instance:
                return True
        return False
    def genNoBackTrackEncode(self):
        self.encode = self._instance.genNoBackTrackEncode(self.encode)
#    def __cmp__(self,other):
#        if self.objective> other.objective:
#            return True
#        else:
#            return False


if __name__ == '__main__':
#    print(Solution.__doc__)
    insName = 's100_5_10_max100_2.5_2.5_2.5_1.2_thre0.1_MPDAins.dat'
    ins = Instance(BaseDir + '//data\\' + insName)
    sol = Solution(ins)
#    print(sol.encode)
#    print(sol)
    sol.encode[0][1] = 100
#    print(sol.encode[(0,1)])
    sol.encode[(0,1)] = -2
#    print(sol.encode[(0,1)])
#    print(sol.encode)    
    sol2 = copy.deepcopy(sol)
    
    print((sol.encode == sol2.encode).all())
#    print(sol.instance == sol2.instance)
#    print(sol.instance.insFileName == sol2.instance.insFileName)
#    print(sol.instance.insFileName)
#    print(sol2.instance.insFileName)
    sol.encode[(0,1)] = -20
    if sol == sol2:
        print('=-00asd-0i ')
    else:
        print('not eq')
#    print(sol2.encode)
#    if sol.encode == sol2.encode:
#        print('0aslhdjk')
    
    