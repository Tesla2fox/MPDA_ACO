# -*- coding: utf-8 -*-
"""
Created on Tue Sep  4 11:18:03 2018

inh is an abbreviation for inherent
@author: robot
"""
import math
import sys
import copy
import numpy
from collections import namedtuple

def getTaskDic():
    dic = dict()
    dic['initState'] = 0
    dic['init_rate'] = 0
# =============================================================================
# current state and rate 
# =============================================================================
    dic['c_state'] = 0
    dic['c_rate'] = 0
    dic['threhold'] = 0
    dic['cmplt'] = False
    return dic


class Task():
    _model = {'exp': '_expModel',
                 'line': '_lineModel'}
    def __init__(self):
        self.initState  = 0
        self.initRate = 0
        self.cState = 0
        self.cRate = 0
        self._changeRateTime = 0
        self.cmplt = False
        self.threhod = 0
        self._cmpltTime = sys.float_info.max

    @property
    def cmpltTime(self):
        return self._cmpltTime
    @cmpltTime.setter
    def cmpltTime(self,cmplt_time):
        if cmplt_time < self._changeRateTime:
            raise  Exception("cmplt_time < changeRateTime Error")
        else:
            self._cmpltTime = cmplt_time
    @property
    def changeRateTime(self):
        return self._changeRateTime


    def calExecuteDur(self):
#        print(self.threhod)
        if math.isclose(self.threhod,self.cState):
            return 0
        e_dur = math.log(self.threhod/self.cState)/self.cRate
#        print(self.threhod)
#        print(self.cState)
        if e_dur <0:
            print('threhod',self.threhod)
            print('cState',self.cState)
            raise Exception('Negative execution during')
            print('Negative execution during')
        return e_dur
# =============================================================================
# valid = false means that  cState is too high can not figure out
#   = true means that cState is right
# =============================================================================
    def calCurrentState(self,time):
        changeDur = time - self._changeRateTime
        incre = changeDur * self.cRate        
        '''
        指数大于70 将会返回错误
        数据爆炸了= =
        '''

        if incre > 700:
            return False
        self.cState = self.cState*math.exp(changeDur*self.cRate)                
        valid =  True
        if(self.cState >sys.float_info.max ):
            valid = False
        self._changeRateTime = time
        return valid

    '''
    主要用于计算行动序列
    '''
    def calRobArrive(self,arrive_time,rob_abi):
        if self.calCurrentState(arrive_time):
            self.cRate = self.cRate - rob_abi
            if self.cRate >= 0:
                leave_time = sys.float_info.max
            else:
                execute_dur = self.calExecuteDur()
                leave_time = arrive_time + execute_dur
            self.cmpltTime = leave_time
        else:
            raise Exception("计算行动序列的时候不应该出错")
        return leave_time

    '''
    predict state and execute period for task 
    '''
    def preCalCurrentState(self,time):
        changeDur = time - self._changeRateTime
        incre = changeDur * self.cRate        
        if incre > 709:
            return False
        cState = self.cState*math.exp(changeDur*self.cRate)                
        valid =  True
        if(cState > sys.float_info.max ):
            valid = False
        return cState,valid
    
    def preCalExecuteDur(self,cState,cRate):
        e_dur = math.log(self.threhod/cState)/cRate
        if e_dur <0:
            print('threhod',self.threhod)
            print('cState',self.cState)
            raise Exception('Bug dur')
            print('bug dur')
        return e_dur
        
    def isCmplt(self):
#        print(self.cState)
#        print(self.threhod)
        bias = abs(self.cState - self.threhod)
        if bias < 0.000001:
            self.cmplt = True
            return True
        else:
            self.cmplt = False
            return False
    def display(self):
        print('initState',self.initState,' initRate',self.initRate,
              ' cState',self.cState,' cRate',self.cRate,
        ' changeRateTime',self._changeRateTime,' cmplt ',self.cmplt,
        ' threhod ',self.threhod)
    def __str__(self):
        return 'initState = '+ str(self.initState)+ ' initRate = '+str(self.initRate)+' cState = '+str(self.cState)\
        +' cRate = '+str(self.cRate)+' changeRateTime = '+str(self._changeRateTime) + ' cmplt  = '+ str(self.cmplt)\
        +' threhod '+ str(self.threhod) +' cmpltTime = ' + str(self._cmpltTime)
    def variableInfo(self):
        return self.cState,self.cRate,self._changeRateTime,self.cmplt,self._cmpltTime
    def recover(self,cState,cRate,changeRateTime,cmplt,cmpltTime):
        self.cState = cState
        self.cRate = cRate
        self._changeRateTime = changeRateTime
        self.cmplt = cmplt
        self._cmpltTime = cmpltTime
    def __deepcopy__(self,memo):
        """Overridden to avoid cloning the problem definition."""
        result = Task()
        memo[id(self)] = result        
        for k, v in self.__dict__.items():
            setattr(result, k, copy.deepcopy(v, memo))                
        return result
    def __eq__(self, other):
        if self.cRate == other.cRate and self.cState == other.cState and self._changeRateTime == other._changeRateTime\
            and  self.cmplt == other.cmplt and self._cmpltTime == other._cmpltTime:
            return True
        else:
            return False








if __name__ == '__main__':
    tsk = Task()
    tsk.display()
    print(tsk.__dict__)
    print(tsk.initState)    
    print(math.log(1.7976931348623157e+308))
    