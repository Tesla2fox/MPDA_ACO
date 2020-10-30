# -*- coding: utf-8 -*-
"""
Created on Fri Sep 21 15:49:17 2018

@author: robot
"""
from readcfg import  *
import numpy as np
from  enum import  Enum

class TaskModelType(Enum):
    ExpModel = 1
    LineModel = 2




class Instance(object):
    def __init__(self, insFileName = 'wtf'):
        self.insFileName = insFileName
        readCfg = Read_Cfg(insFileName)
        self.robNum = int(readCfg.getSingleVal('robNum'))
        self.taskNum = int(readCfg.getSingleVal('taskNum'))
        self.threhold = readCfg.getSingleVal('comp_threhold')
        self.robAbiLst = []
        self.robVelLst = []
        self.taskStateLst = []
        self.taskRateLst = []
        readCfg.get('rob_abi',self.robAbiLst)
        readCfg.get('rob_vel',self.robVelLst)
        readCfg.get('tsk_rat',self.taskRateLst)
        readCfg.get('tsk_state',self.taskStateLst)
        self.rob2taskDisMat = np.zeros((self.robNum,self.taskNum))
        disLst = []
        readCfg.get('rob2tskDis',disLst)
#        print(self.rob2taskDisMat)
        for i in range(self.robNum):
            for j in range(self.taskNum):
#                print(i,j)
#                print(i*self.robNum+j)
#                print(disLst[i*self.robNum+j])
                self.rob2taskDisMat[i][j] = disLst[i*self.taskNum+j]
        self.taskDisMat = np.zeros((self.taskNum,self.taskNum))
        disLst = []
        readCfg.get('tskDis',disLst)
        for i in range(self.taskNum):
            for j in range(self.taskNum):
                self.taskDisMat[i][j] = disLst[i*self.taskNum+j]
        # self.decode = DecodeSS(self.insFileName)
        self.taskModelType = TaskModelType.ExpModel

    def __str__(self):
        return self.insFileName + '\n robNum = '+ str(self.robNum) +' task =' +str(self.taskNum)
    def __eq__(self,other):
        if self.insFileName == other.insFileName:
            return True
        else:
            return False
    def __ne__(self,other):
        if self.__eq__(other):
            return False
        return True
    def evaluate(self,encode):
        self.decode.encode = encode
#        makespan = self.decode.decode()
        try:
            makespan = self.decode.decode()
            pass
        except InvalidStateException:
            makespan = sys.float_info.max
        except RobotStuckException:
            makespan = sys.float_info.max
#        except Exception as e:
#            print(e)
        return makespan
    def genNoBackTrackEncode(self,encode):
        resEncode = np.zeros((self.robNum,self.taskNum),dtype =int)
        resEncode[:][:] = -1
        for i  in range(self.robNum):
            ind = 0
            for j in range(self.taskNum):
                if encode[i][j] != -1:
                    resEncode[i][ind] = encode[i][j]
                    ind += 1
        return resEncode
    def calRob2TaskPeriod(self,robID,taskID):
        dis = self.rob2taskDisMat[robID][taskID]
        dis_time = dis/self.robVelLst[robID]
        return dis_time
    def calTask2TaskPeriod(self,robID,taskID1,taskID2):
        dis = self.taskDisMat[taskID1][taskID2]
        period = dis/self.robVelLst[robID]
        return period


if __name__ =='__main__':

    wtf = Read_Cfg("wf")

    # insName = 's100_5_10_max100_2.5_2.5_2.5_1.2_thre0.1_MPDAins.dat'
    # ins = Instance(BaseDir + '//data\\' + insName)
    # insName = 's100_5_10_max100_2.5_2.5_2.5_1.2_thre0.1_MPDAins.dat'
    # ins2 = Instance(BaseDir + '//data\\' + insName)
    # if ins == ins2:
    #     print('asd')
    # print(ins)
    
    
    