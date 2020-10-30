import random
import numpy as np
import readcfg as rd

from enum import Enum
import matplotlib.pyplot as plt


class TaskModelType(Enum):
    ExpModel = 1
    LineModel = 2

class MPDAInstance(object):
    def __init__(self):
        pass
    def loadCfg(self,fileName : str):
        # print(fileName)
        self._insName = fileName
        readCfg = rd.Read_Cfg(fileName)
        self._robNum = int(readCfg.getSingleVal('robNum'))
        self._taskNum = int(readCfg.getSingleVal('taskNum'))
        self._threhold = readCfg.getSingleVal('comp_threshold')
        self._robAbiLst  = []
        self._robVelLst = []
        self._taskStateLst = []
        self._taskRateLst = []
        readCfg.get('rob_abi',self._robAbiLst)
        readCfg.get('rob_vel',self._robVelLst)
        readCfg.get('task_rate',self._taskRateLst)
        readCfg.get('task_init_demand',self._taskStateLst)
        self._rob2taskDisMat = np.zeros((self._robNum,self._taskNum))
        disLst = []
        readCfg.get('rob2taskDisMat', disLst)
        for i in range(self._robNum):
            for j in range(self._taskNum):
                self._rob2taskDisMat[i][j] = disLst[i * self._taskNum + j]

        self._taskDisMat = np.zeros((self._taskNum, self._taskNum))
        disLst = []
        readCfg.get('taskDisMat', disLst)
        for i in range(self._taskNum):
            for j in range(self._taskNum):
                self._taskDisMat[i][j] = disLst[i * self._taskNum + j]

        # self._taskConLst = []
        # readCfg.get('tsk_con', self._taskConLst)

    def __str__(self):
        return 'robNum = ' + str(self._robNum) + '  taskNum = ' + str(self._taskNum) +'\n'+ self._insName

    def drawIns(self):
        readCfg = rd.Read_Cfg(self._insName)

        rob_x_lst = []
        rob_y_lst = []
        readCfg.get('rob_x',rob_x_lst)
        readCfg.get('rob_y',rob_y_lst)
        plt.plot(rob_x_lst, rob_y_lst, 'ro',label='Rob')
        for i in range(self._robNum):
            plt.text(rob_x_lst[i],rob_y_lst[i],'R' + str(i))


        tsk_x_lst = []
        tsk_y_lst = []
        readCfg.get('tsk_x',tsk_x_lst)
        readCfg.get('tsk_y',tsk_y_lst)

        plt.plot(tsk_x_lst, tsk_y_lst, 'bs',label='Task')
        for i in range(self._taskNum):
            plt.text(tsk_x_lst[i],tsk_y_lst[i],'T' + str(i))
        plt.legend()
        plt.show()

        # plt.title(self._insName)
        # plt.savefig(self.figBaseDir + '_box'+ str(insID))

if __name__ == '__main__':
    # print('test')
    insFileName = './/staticMpdaBenchmarkSet//S_3_15_5.03.txt'
    ins = MPDAInstance()
    ins.loadCfg(fileName= insFileName)
    print(ins)