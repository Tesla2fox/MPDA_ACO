import readcfg as r_d
from mpdaInstance import MPDAInstance
import os, sys
from mpdaDecodeMethod.mpdaRobot import RobotState,Robot
from mpdaDecodeMethod.mpdaTask import  Task
from mpdaDecodeMethod.mpdaDecoderActSeq import ActionSeq,ActionTuple,EventType,MPDADecoderActionSeq
import numpy as np
from enum import Enum
from collections import  namedtuple
import math

RobTaskPair = namedtuple('RobTaskPair',['robID','taskID'])


class CalType(Enum):
    arriveCond = 1
    leaveCond = 2
    endCond = 3
    backCond = 4
    stateInvalidCond = 5

def generateRandEncode(robNum,taskNum):
    encode = np.zeros((robNum, taskNum),dtype =int)
    for i in range(robNum):
        permLst = [x for x in range(taskNum)]
        np.random.shuffle(permLst)
        encode[i][:] = permLst
    return encode


# def generateRandPopEncode(robNum,taskNum):
#     pop = []
#     encode = np.zeros((robNum, taskNum),dtype =int)
#     for i in range(robNum):
#         permLst = [x for x in range(taskNum)]
#         np.random.shuffle(permLst)
#         encode[i][:] = permLst
#     robIndLst = [0 for _ in range(robNum)]
#
#     while len(pop) == (robNum*taskNum):
#         rdRobID = np.random.randint(0,robNum -1)
#         rdTaskID = encode[rdRobID][robIndLst[rdRobID]]
#         pop.append(RobTaskPair(rdRobID,rdTaskID))
#         robIndLst[rdRobID] += 1
#
#     '''
#     there are still some problems to construct a
#     '''






AbsolutePath = os.path.abspath(__file__)
# 将相对路径转换成绝对路径
SuperiorCatalogue = os.path.dirname(AbsolutePath)
# 相对路径的上级路径
BaseDir = os.path.dirname(SuperiorCatalogue)

degBoolean = False

class PopDecoder(object):
    def __init__(self,ins :MPDAInstance):
        self._insName = ins._insName
        # readCfg = rd.Read_Cfg(fileName)
        self._robNum = ins._robNum
        self._taskNum = ins._taskNum
            # int(readCfg.getSingleVal('taskNum'))
        self._threhold = ins._threhold
        self._robAbiLst  = ins._robAbiLst
        self._robVelLst = ins._robVelLst
        self._taskStateLst = ins._taskStateLst
        self._taskRateLst = ins._taskRateLst
        self._rob2taskDisMat = ins._rob2taskDisMat
        self._taskDisMat = ins._taskDisMat
        if degBoolean:
            self._degFile = open(BaseDir+ '/debugData/deg.dat', 'w')


class MPDADecoderCon(object):
    def __init__(self,ins :MPDAInstance):
        self._insName = ins._insName
        # readCfg = rd.Read_Cfg(fileName)
        self._robNum = ins._robNum
        self._taskNum = ins._taskNum
            # int(readCfg.getSingleVal('taskNum'))
        self._threhold = ins._threhold
        self._robAbiLst  = ins._robAbiLst
        self._robVelLst = ins._robVelLst
        self._taskStateLst = ins._taskStateLst
        self._taskRateLst = ins._taskRateLst
        self._rob2taskDisMat = ins._rob2taskDisMat
        self._taskDisMat = ins._taskDisMat
        self._taskConLst = ins._taskConLst
        if degBoolean:
            self._degFile = open(BaseDir+ '/debugData/deg.dat', 'w')

    def decode(self, x):
        self.encode = x
        self._actSeq = ActionSeq()
        self._invalidTaskLst = []
        self.initStates()
        # if self.decodeProcessor():
        validStateBoolean = self.decodeProcessor()
        if degBoolean:
            self._degFile.write(str(self.cmpltLst))
        # if self._invalidTaskLst:
        # print(self._actSeq.convert2MultiPerm(self._robNum))
        return validStateBoolean, self._actSeq, self._invalidTaskLst
    def initStates(self):
        '''
        initialize states of decode method
        '''
        self.taskLst = []
        self.robotLst = []
        self.cmpltLst = [False] * self._taskNum
        for i in range(self._robNum):
            rob = Robot()
            rob._ability = self._robAbiLst[i]
            rob._vel = self._robVelLst[i]
            rob.encodeIndex = 0
            rob.taskID, rob.encodeIndex, stopBool = self.getRobTask(robID=i, encodeIndex=0)
            if not stopBool:
                dis = self._rob2taskDisMat[i][rob.taskID]
                dis_time = dis / rob._vel
                rob.arriveTime = dis_time
            rob.stopBool = stopBool
            rob.stateType = RobotState['onRoad']
            rob.leaveTime = 0
            self.robotLst.append(rob)

        for i in range(self._taskNum):
            task = Task()
            task.cState = math.log(self._taskStateLst[i])
            task._initState = math.log(self._taskStateLst[i])
            task.cRate = self._taskRateLst[i]
            task._initRate = self._taskRateLst[i]
            task._threhod = math.log(self._threhold)
            task.cmpltTime = sys.float_info.max
            self.taskLst.append(task)
        # self.decodeTime = 0
        # self.validStateBool = True
    def decodeProcessor(self):
        while not self.allTaskCmplt():
            cal_type, actionID = self.findActionID()
            if cal_type  == CalType['arriveCond']:
                rob = self.robotLst[actionID]
                arriveTime = rob.arriveTime
                encodeInd = rob.encodeIndex
                taskID = self.encode[actionID][encodeInd]
                self._actSeq.append(ActionTuple(robID =actionID,taskID= taskID, eventType = EventType.arrive,eventTime = arriveTime))

                if self.cmpltLst[taskID]:
                    # =============================================================================
                    #  the task has been cmplt
                    # =============================================================================
                    rob = self.robotLst[actionID]
                    rob.leaveTime = rob.arriveTime
                    rob.taskID = taskID
                    rob.stateType = RobotState['onTask']
                    self._actSeq._arrCmpltTaskLst.append((actionID, taskID))
                else:
# =============================================================================
# the task has not been cmplt
# =============================================================================
                    task = self.taskLst[taskID]
                    rob.taskID = taskID
                    validStateBool = task.calCurrentState(arriveTime)
                    if task.cState > math.exp(self._taskConLst[taskID]):
                        if taskID in self._invalidTaskLst:
                            pass
                        else:
                            self._invalidTaskLst.append(taskID)
                    if not validStateBool:
                        break
                    task.cRate = task.cRate - rob._ability
# can not be cmplted
                    if task.cRate >= 0:
                        leaveTime = sys.float_info.max
# can be completed
                    else:
                        rob.executeDur = task.calExecuteDur()
                        rob.executeBool = False
                        leaveTime = rob.arriveTime + rob.executeDur
                        coordLst = self.findCoordRobot(actionID)
                        for coordID in coordLst:
                            coordRob = self.robotLst[coordID]
                            coordRob.leaveTime = leaveTime
                            coordRob.executeDur = coordRob.leaveTime - coordRob.arriveTime
                    rob.leaveTime = leaveTime
                    rob.stateType = RobotState['onTask']


            if cal_type == CalType['leaveCond']:
                rob = self.robotLst[actionID]
                taskID = rob.taskID
                task = self.taskLst[taskID]
                preTaskID = taskID
                self.cmpltLst[taskID] = True
                self._actSeq.append(ActionTuple(robID =actionID,taskID= taskID, eventType = EventType.leave,eventTime = rob.leaveTime))
                task.cmpltTime = rob.leaveTime

                coordLst = self.findCoordRobot(actionID)
                for coordID in coordLst:
                    self.updateRobLeaveCond(robID = coordID)
                    self._actSeq.append(ActionTuple(robID = coordID, taskID= taskID,eventType = EventType.leave,eventTime = task.cmpltTime))
                self.updateRobLeaveCond(robID = actionID)

                '''
                debug is here
                '''

                if degBoolean:
                    self._degFile.write(str(taskID) + ' have been completed\n')

            if cal_type == CalType.endCond:
                # invalidFitness = True
                validStateBool = False
                break

        if not validStateBool:
            pass
            # print('the state is explosion')
        return  validStateBool
    '''
    some fucntions
    '''

    def allTaskCmplt(self):
        if False in self.cmpltLst:
            return False
        else:
            return True

    def findActionID(self):
        cal_type = CalType['endCond']
        actionID = sys.float_info.max
        minTime = sys.float_info.max
        for i in range(self._robNum):
            rob = self.robotLst[i]
            if rob.stopBool != True:
                if rob.stateType == RobotState['onRoad']:
                    if rob.arriveTime < minTime:
                        minTime = rob.arriveTime
                        cal_type = CalType['arriveCond']
                        actionID = i
                if rob.stateType == RobotState['onTask']:
                    if rob.leaveTime < minTime:
                        minTime = rob.leaveTime
                        cal_type = CalType['leaveCond']
                        actionID = i
        if degBoolean:
            self.saveRobotInfo(degFile= self._degFile)
            self._degFile.write(str(actionID) + ' time = '+ str(minTime)
                              + ' type = ' + str(cal_type) + '\n')
        # self.saveEventInMemory()
        # if minTime < self.decodeTime:
        #     cal_type = CalType['backCond']
        #            print(minTime)
        #            print(self.decodeTime)
        #            taskID = self.robotLst[actionI].taskID
        #        self.saveRobotInfo()

        return cal_type, actionID

    def findCoordRobot(self, robID):
        '''
        find robots which are corrdinated with the robot A
        '''
        coordLst = []
        rob = self.robotLst[robID]
        taskID = rob.taskID
        for i in range(self._robNum):
            if i == robID:
                continue
            #            crob = self.robotLst[i]
            if self.robotLst[i].stateType == RobotState['onRoad']:
                continue
            if self.robotLst[i].stopBool == True:
                continue
            if self.robotLst[i].taskID == taskID:
                coordLst.append(i)
        return coordLst

    def updateRobLeaveCond(self,robID):
        rob = self.robotLst[robID]
        preTaskID = rob.taskID
        while True:
            if rob.encodeIndex == (self._taskNum - 1):
                rob.stopBool = True
                break
            rob.encodeIndex += 1
            taskID = self.encode[robID][rob.encodeIndex]
            if self.cmpltLst[taskID]:
                continue
            else:
                roadDur = self.calRoadDur(preTaskID, taskID, robID)
                arriveTime = rob.leaveTime + roadDur
                if arriveTime > self.taskLst[taskID].cmpltTime:
                    continue
                rob.roadDur = roadDur
                rob.taskID = taskID
                rob.arriveTime = rob.leaveTime + rob.roadDur
                rob.stateType = RobotState['onRoad']
                break

    def getRobTask(self, robID=0, encodeIndex=0):
            '''
            get the robot next task ID
            '''
            stopBool = False
            while True:
                if encodeIndex == self._taskNum:
                    stopBool = True
                    break
                taskID = self.encode[robID][encodeIndex]
                if taskID < 0:
                    encodeIndex += 1
                    continue
                else:
                    break
            return taskID, encodeIndex, stopBool

    def calRoadDur(self,taskID1,taskID2,robID):
        '''
        calculate the time fragment from the time when robID leaves the taskID1 to
        the time when rob arrives the taskID2
        '''
        dis = self._taskDisMat[taskID1][taskID2]
        rob = self.robotLst[robID]
        roadDur = dis/rob._vel
        return roadDur

    def saveRobotInfo(self,degFile):
        '''
        save robot information into the deg files
        '''
        deg = degFile
        deg.write('\n')
        for i in range(self._robNum):
            lst = []
            lst.append(i)
            lst.append('arriveTime')
            lst.append(self.robotLst[i].arriveTime)
            lst.append('leaveTime')
            lst.append(self.robotLst[i].leaveTime)
            lst.append('state')
            lst.append(self.robotLst[i].stateType)
            lst.append('taskID')
            lst.append(self.robotLst[i].taskID)
            str_lst  = [str(x) for x in lst]
            robInfo = '  '
            robInfo = robInfo.join(str_lst)
            deg.write(robInfo+'\n')
        deg.write('\n')
        deg.flush()
import os

if __name__ == '__main__':
    print('test_mpdaDecoder')
    # f_con = open(BaseDir +'//b_conBenchmark.dat','w')
    # print(BaseDir)
    # for root, exsit_dirs, files in os.walk(BaseDir+'//conBenchmark'):
    #     # print(files)
    #     break
    #     for file in files:
    #         print(file)
    #         ins = MPDAInstance()
    #         insFileName = BaseDir + '//conBenchmark//'+file
    #         # insFileName = BaseDir + '//conBenchmark//5_5_CLUSTERED_CLUSTERED_UNITARY_SVSCV_LVSCV_thre0.1MPDAins.dat'
    #         try:
    #             ins.loadCfg(fileName=insFileName)
    #         except:
    #             continue
    #         decoder = MPDADecoderCon(ins)
    #         invalidNum = 0
    #         for _ in range(100):
    #             x = generateRandEncode(robNum=ins._robNum, taskNum=ins._taskNum)
    #             validStateBoolean, actSeq, invalidTask = decoder.decode(x)
    #             actSeqDecoder = MPDADecoderActionSeq(ins)
    #             actSeqDecoder.decode(actSeq)
    #             if len(decoder._invalidTaskLst) !=0:
    #                 invalidNum += 1
    #         print(invalidNum)
    #         if invalidNum > 20:
    #             print(file)
    #             f_con.write(file +'\n')
    #             f_con.flush()
    #         # break
    #     break
    # # exit()

    ins = MPDAInstance()
    insFileName = BaseDir + '//conBenchmark//' + '8_8_CLUSTERED_CLUSTERED_MSVFLV_QUADRANT_MSVFLV_thre0.1MPDAins.dat'
    insFileName = BaseDir + '//conBenchmark//5_5_CLUSTERED_CLUSTERED_MSVFLV_MSVFLV_SVSCV_thre0.1MPDAins.dat'
    insFileName = BaseDir +'//conBenchmark//' + '10_15_RANDOMCLUSTERED_RANDOMCLUSTERED_SVSCV_LVSCV_QUADRANT_thre0.1MPDAins.dat'
    insFileName = BaseDir +'//sBenchmark//' + '14_12_ECCENTRIC_CLUSTERED_QUADRANT_UNITARY_UNITARY_thre0.1MPDAins.dat'
    insFileName = BaseDir +'//anaBenchmark//' + '10_5_RANDOM_RANDOM_SVSCV_SVSCV_LVLCV_thre0.1MPDAins.dat'
    insFileName = BaseDir +'//anaBenchmark//' + '20_10_ECCENTRIC_CLUSTERED_SVSCV_SVSCV_LVLCV_thre0.1MPDAins.dat'
    insFileName = BaseDir +'//anaBenchmark//' + '15_10_ECCENTRIC_CLUSTERED_SSVSCV_SVSCV_LVLCV_thre0.1MPDAins.dat'
    insFileName = BaseDir +'//anaBenchmark//' + '40_10_CENTRAL_RANDOM_MSVFLV_SVSCV_LVLCV_thre0.1MPDAins.dat'


    ins.loadCfg(fileName=insFileName)
    decoder = MPDADecoderCon(ins)

    # np.random.seed(2)
    # x = []
    # for _ in range(10):
    #     x.append(RobTaskPair(robID = np.random.randint(0, ins._robNum -1),taskID = np.random.randint(0, ins._taskNum -1)))
    # print(x)

    x = generateRandEncode(robNum= ins._robNum, taskNum= ins._taskNum)
    # x = [RobTaskPair(robID = 1, taskID = 2),RobTaskPair(robID = 3,taskID = 4)]
    # print(x)
    # exit()
    # x = [[6, 1, 0, 7, 5 ,4, 2, 3],
    #     [6, 4, 7 ,0 ,5 ,3 ,2 ,1],
    #     [3, 0 ,4 ,2 ,6 ,7 ,5 ,1],
    #     [3, 5 ,6 ,0 ,2 ,7 ,1 ,4],
    #     [4, 5 ,1 ,0 ,6 ,7 ,3 ,2],
    #     [6, 3 ,4 ,0 ,7 ,5 ,1 ,2],
    #     [1 ,0 ,3 ,6 ,7 ,4 ,5, 2],
    #     [1, 4, 7, 0, 2, 3, 6, 5]]
    # x =

    validStateBoolean,actSeq,invalidTask = decoder.decode(x)
    actSeqDecoder = MPDADecoderActionSeq(ins)
    actSeqDecoder.decode(actSeq)
    # print(np.array(actSeq.convert2MultiPerm(ins._robNum),dtype = object))
    for perm in actSeq.convert2MultiPerm(ins._robNum):
        print(perm)
    # actSeqDecoder.drawActionSeqGantt()
    # actSeqDecoder.drawTaskScatter()
    # actSeqDecoder.drawTaskDependence()

    print('first decoder is over')
    # np.random.seed(1)

    chrom = [1, 0, 6, 3, 7, 5, 2, 4, 6, 3, 2, 5, 0, 7, 1, 4, 6, 3, 1, 2, 5, 7, 4, 0, 1, 0, 6, 5, 7, 3, 4, 2, 6, 3, 5, 7, 4,
         1, 0, 2, 6, 3, 2, 5, 7, 4, 1, 0, 1, 6, 0, 5, 7, 2, 4, 3, 6, 3, 2, 7, 4, 1, 5, 0]

    chrom = [7, 1, 5, 6, 2, 3, 0, 4, 4, 7, 5, 1, 3, 0, 2, 6, 2, 6, 7, 0, 1, 3, 4, 5, 3, 1, 0, 2, 5, 4, 6, 7, 4, 3, 2, 5, 1, 7, 6, 0, 2, 6, 1, 5, 4, 0, 7, 3, 2, 1, 7, 6, 5, 3, 0, 4, 5, 1, 6, 4, 0, 2, 3, 7]
        #
        # [2, 3, 7, 5, 4, 1, 0, 6, 2, 3, 0, 6, 4, 1, 5, 7, 3, 5, 1, 4,
        #      2, 6, 0, 7, 4, 7, 5, 1, 3, 2, 0, 6, 3, 1, 5, 4, 2, 6, 0, 7, 2, 0, 5, 6, 4, 3, 7, 1, 6, 7, 0, 1, 5, 2, 4, 3, 1, 7, 2, 6, 3, 0, 5, 4]
    chrom = [1, 6, 0, 3, 5, 7, 2, 4, 4, 0, 7, 2, 1, 5, 6, 3, 3, 2, 0, 5, 4, 6, 7, 1, 5, 1, 6, 0, 4, 3, 2, 7, 4, 0, 7, 3, 6, 1, 2, 5, 3, 2, 7, 5, 4, 6, 1, 0, 3, 2, 7, 6, 0, 1, 5, 4, 5, 1, 6, 7, 0, 4, 2, 3]
    chrom = [6, 1, 5, 0, 2, -1, -1, -1, 4, 3, 2, -1, -1, -1, -1, -1, 3, 2, -1, -1, -1, -1, -1, -1, 1, 5, 0, 2, -1, -1, -1, -1, 4, 3, 2, -1, -1, -1, -1, -1, 3, 2, -1, -1, -1, -1, -1, -1, 7, 6, 1, 5, 0, 2, -1, -1, 7, 6, 1, 5, 0, 2, -1, -1]
    # chrom = [1, 6, 0, 3, 5, 7, 2, 4, 4, 0, 7, 2, 1, 5, 6, 3, 3, 2, 0, 5, 4, 6, 7, 1, 5, 1, 6, 0, 4, 3, 2, 7, 4, 0, 7, 3, 6, 1,
    #  2, 5, 3, 2, 7, 5, 4, 6, 1, 0, 3, 2, 7, 6, 0, 1, 5, 4, 5, 1, 6, 7, 0, 4, 2, 3]
    chrom = [7, 6, 1, 5, 0, 2, -1, -1, 4, 5, 0, 2, -1, -1, -1, -1, 3, 0, 2, -1, -1, -1, -1, -1, 6, 1, 5, 2, -1, -1, -1, -1, 4, 0, 2, -1, -1, -1, -1, -1, 3, 5, 0, 2, -1, -1, -1, -1, 3, 0, 2, -1, -1, -1, -1, -1, 7, 6, 1, 5, 0, 2, -1, -1]

    chrom = [3, 7, 2, -1, -1, -1, -1, -1, 4, 7, 2, -1, -1, -1, -1, -1, 3, 7, 2, -1, -1, -1, -1, -1, 5, 1, 6, 7, 2, -1, -1, -1, 4, 7, 2, -1, -1, -1, -1, -1, 3, 2, -1, -1, -1, -1, -1, -1, 7, 2, -1, -1, -1, -1, -1, -1, 5, 1, 6, 0, 2, -1, -1, -1]

    chrom = [0, 4, 1, -1, -1, 2, 0, 1, -1, -1, 2, 0, 1, -1, -1, 3, 4, 1, -1, -1, 2, 0, 4, 1, -1]
    chrom = [8, 3, 6, 5, 15, 16, 2, 12, 0, 1, 13, 10, 19, 9, 14, 11, 4, 17, 18, 7]
    chrom = [1, 8, 11, 0, 10, 7, 6, 5, 9, 2, 4, 3, 5, 6, 1, 11, 2, 7, 0, 4, 8, 10, 9, 3, 7, 6, 5, 0, 11, 2, 10, 4, 8, 3, 1, 9, 7, 6, 2, 0, 8, 5, 11, 10, 3, 4, 9, 1, 8, 10, 2, 7, 5, 0, 11, 4, 9, 3, 1, 6, 5, 3, 10, 8, 11, 9, 7, 2, 1, 4, 6, 0, 1, 0, 7, 10, 11, 4, 5, 6, 2, 9, 3, 8, 5, 1, 9, 7, 0, 10, 3, 6, 8, 2, 4, 11, 5, 6, 7, 1, 2, 4, 11, 8, 0, 9, 3, 10, 0, 11, 4, 2, 6, 5, 8, 10, 7, 9, 1, 3, 5, 3, 7, 6, 1, 0, 8, 4, 9, 11, 10, 2, 9, 11, 6, 8, 7, 1, 10, 0, 2, 4, 3, 5, 7, 4, 6, 11, 3, 0, 1, 2, 5, 10, 9, 8, 5, 3, 11, 6, 0, 2, 1, 4, 8, 9, 10, 7]
    # chrom = [3, 4, 0, 2, 1, 1, 4, 0, 3, 2, 1, 2, 0, 3, 4, 1, 2, 0, 4, 3, 1, 3, 4, 0, 2]
    chrom = [6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 4, 10, 1, -1, -1, -1, -1, -1, -1, -1, -1, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 3, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 2, 10, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 4, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 7, 9, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 11, 10, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1]

    chrom = [11, 1, 14, 3, 8, 9, 10, 2, 12, 7, 13, 5, 4, 0, 6, 5, 14, 8, 12, 0, 4, 3, 13, 10, 1, 7, 11, 2, 9, 6, 1, 14, 7, 8, 6, 0, 3, 9, 2, 11, 12, 10, 4, 5, 13, 9, 0, 12, 14, 3, 13, 2, 6, 5, 7, 11, 1, 8, 4, 10, 9, 14, 7, 12, 0, 4, 1, 2, 5, 8, 11, 3, 10, 6, 13, 1, 8, 6, 9, 5, 2, 4, 11, 7, 13, 14, 10, 3, 0, 12, 1, 8, 9, 6, 0, 2, 12, 7, 3, 10, 4, 14, 11, 13, 5, 11, 9, 8, 12, 3, 5, 6, 7, 2, 0, 14, 1, 13, 4, 10, 2, 12, 6, 5, 14, 4, 9, 3, 7, 1, 0, 8, 10, 13, 11, 10, 0, 13, 5, 3, 4, 6, 7, 8, 14, 9, 1, 12, 2, 11]
    chrom = [4, 2, 0, 3, 1, 2, 0, 1, 4, 3, 1, 3, 4, 0, 2, 1, 4, 0, 3, 2, 4, 2, 1, 0, 3, 4, 2, 1, 0, 3, 1, 3, 0, 2, 4, 4, 2, 0, 3, 1, 3, 4, 2, 0, 1, 3, 2, 1, 4, 0]
    chrom = [1, -1, -1, -1, -1, 2, 0, -1, -1, -1, 4, 0, -1, -1, -1, 1, -1, -1, -1, -1, 4, 2, 0, -1, -1, 2, 0, -1, -1, -1, 3, -1, -1, -1, -1, 4, 0, -1, -1, -1, 3, -1, -1, -1, -1, 4, 0, -1, -1, -1]
    chrom = [1, 8, 11, 0, 10, 7, 6, 5, 9, 2, 4, 3, 5, 6, 1, 11, 2, 7, 0, 4, 8, 10, 9, 3, 7, 6, 5, 0, 11, 2, 10, 4, 8, 3, 1, 9, 7, 6, 2, 0, 8, 5, 11, 10, 3, 4, 9, 1, 8, 10, 2, 7, 5, 0, 11, 4, 9, 3, 1, 6, 5, 3, 10, 8, 11, 9, 7, 2, 1, 4, 6, 0, 1, 0, 7, 10, 11, 4, 5, 6, 2, 9, 3, 8, 5, 1, 9, 7, 0, 10, 3, 6, 8, 2, 4, 11, 5, 6, 7, 1, 2, 4, 11, 8, 0, 9, 3, 10, 0, 11, 4, 2, 6, 5, 8, 10, 7, 9, 1, 3, 5, 3, 7, 6, 1, 0, 8, 4, 9, 11, 10, 2, 9, 11, 6, 8, 7, 1, 10, 0, 2, 4, 3, 5, 7, 4, 6, 11, 3, 0, 1, 2, 5, 10, 9, 8, 5, 3, 11, 6, 0, 2, 1, 4, 8, 9, 10, 7]
    chrom = [9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 3, 6, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, 3, 5, 6, 9, -1, -1, -1, -1, -1, -1, -1, -1, 11, 8, 4, 2, -1, -1, -1, -1, -1, -1, -1, -1, 3, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, 3, 7, 6, 9, -1, -1, -1, -1, -1, -1, -1, -1, 7, 6, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 11, 8, 4, 2, -1, -1, -1, -1, -1, -1, -1, -1]

    chrom = [ 11, 12, 13, 8, 7, 6, 2, 4, 9, 14, 16, 18, 10, 15, 17, 3, 1, 19, 0, 5, 13, 2, 3, 17, 6, 5, 0, 16, 1, 4, 19, 8, 12, 14, 15, 10, 9, 11, 7, 18, 11, 17, 12, 15, 9, 0, 8, 7, 19, 3, 4, 5, 1, 13, 14, 6, 18, 2, 16, 10, 7, 3, 12, 5, 16, 18, 14, 2, 10, 11, 19, 0, 8, 15, 4, 13, 17, 6, 9, 1, 13, 17, 15, 6, 7, 9, 0, 2, 19, 14, 16, 11, 10, 3, 8, 12, 18, 5, 4, 1, 17, 7, 12, 13, 1, 6, 5, 0, 19, 9, 4, 8, 18, 15, 10, 11, 14, 16, 3, 2, 11, 17, 12, 1, 9, 5, 8, 18, 14, 16, 3, 19, 7, 13, 15, 2, 10, 0, 6, 4, 13, 2, 17, 12, 0, 15, 8, 14, 5, 16, 11, 1, 19, 7, 6, 10, 4, 9, 18, 3, 7, 1, 0, 2, 16, 13, 9, 8, 15, 6, 4, 18, 14, 3, 17, 10, 5, 12, 11, 19, 7, 1, 8, 17, 0, 19, 15, 13, 18, 2, 4, 3, 12, 16, 5, 11, 9, 14, 6, 10]
    chrom = [3, 0, 2, 4, 1, 1, 0, 3, 2, 4, 3, 0, 4, 2, 1, 3, 0, 4, 2, 1, 2, 4, 3, 0, 1, 2, 4, 1, 0, 3, 1, 0, 3, 2, 4, 2, 4, 3, 0, 1, 3, 0, 4, 2, 1, 2, 3, 1, 0, 4]

    chrom = [3, 8, 9, 1, 5, 2, 7, 4, 6, 0, 2, 1, 0, 5, 4, 3, 7, 8, 9, 6, 2, 9, 0, 4, 5, 7, 3, 8, 6, 1, 8, 3, 2, 0, 7, 5, 6, 9, 4, 1, 0, 1, 9, 8, 7, 5, 2, 4, 6, 3, 0, 3, 6, 9, 7, 1, 4, 5, 2, 8, 2, 7, 5, 8, 6, 9, 0, 4, 1, 3, 3, 1, 6, 2, 5, 7, 8, 4, 9, 0, 8, 9, 7, 0, 3, 4, 1, 2, 5, 6, 1, 6, 9, 8, 7, 2, 5, 4, 0, 3, 0, 3, 5, 1, 2, 7, 6, 8, 9, 4, 1, 2, 4, 7, 9, 8, 3, 0, 6, 5, 0, 3, 8, 4, 5, 1, 7, 9, 6, 2, 0, 3, 2, 6, 4, 8, 1, 9, 5, 7, 0, 8, 1, 6, 3, 5, 4, 2, 7, 9, 6, 0, 3, 8, 1, 9, 5, 7, 2, 4, 7, 1, 4, 6, 5, 9, 8, 2, 3, 0, 4, 8, 7, 5, 6, 2, 1, 9, 3, 0, 3, 9, 4, 1, 5, 7, 0, 6, 8, 2, 5, 8, 2, 7, 0, 3, 6, 9, 1, 4]
    chrom = [3, 4, 7, -1, -1, -1, -1, -1, -1, -1, 5, 1, -1, -1, -1, -1, -1, -1, -1, -1, 4, 2, -1, -1, -1, -1, -1, -1, -1, -1, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, 3, 8, 2, -1, -1, -1, -1, -1, -1, -1, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, 3, 0, -1, -1, -1, -1, -1, -1, -1, -1, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, 9, 0, -1, -1, -1, -1, -1, -1, -1, -1, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, 3, 2, -1, -1, -1, -1, -1, -1, -1, -1, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, 3, 9, 7, -1, -1, -1, -1, -1, -1, -1, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, 5, 2, -1, -1, -1, -1, -1, -1, -1, -1, 3, 6, -1, -1, -1, -1, -1, -1, -1, -1, 9, 1, 6, -1, -1, -1, -1, -1, -1, -1, 0, 6, -1, -1, -1, -1, -1, -1, -1, -1]
    chrom = [7, 3, 8, 1, 0, 4, 6, 9, 5, 2, 6, 3, 5, 9, 8, 1, 7, 4, 2, 0, 6, 2, 7, 4, 1, 0, 3, 9, 8, 5, 6, 8, 3, 1, 9, 2, 7, 5, 4, 0, 6, 8, 2, 3, 0, 7, 1, 5, 9, 4, 4, 5, 1, 2, 0, 3, 8, 7, 9, 6, 7, 9, 0, 1, 4, 3, 8, 2, 6, 5, 4, 7, 5, 3, 2, 6, 9, 0, 1, 8, 3, 1, 6, 4, 8, 9, 7, 0, 2, 5, 7, 6, 9, 4, 8, 1, 0, 2, 5, 3, 1, 8, 5, 0, 9, 3, 6, 2, 4, 7, 3, 7, 5, 4, 8, 0, 1, 6, 9, 2, 6, 7, 5, 3, 0, 4, 8, 9, 1, 2, 4, 7, 0, 9, 3, 6, 1, 8, 5, 2, 1, 7, 8, 0, 6, 5, 2, 9, 4, 3, 2, 5, 7, 9, 0, 4, 8, 6, 1, 3, 5, 0, 2, 9, 7, 4, 3, 8, 6, 1, 3, 8, 4, 2, 6, 9, 1, 5, 0, 7, 9, 1, 4, 2, 6, 8, 0, 3, 5, 7, 0, 4, 6, 3, 2, 8, 7, 5, 1, 9]
    chrom = [0, 2, 1, 0, 1, 2, 1, 2, 0, 0, 1, 2, 1, 2, 0, 1, 0, 2, 1, 2, 0, 2, 0, 1, 1, 0, 2, 1, 2, 0]
    chrom = [1, 9, 4, 6, 0, 2, 3, 5, 7, 8, 3, 5, 7, 1, 2, 6, 9, 4, 0, 8, 3, 2, 9, 6, 8, 5, 0, 7, 1, 4, 2, 9, 0, 4, 1, 5, 7, 3, 6, 8, 4, 7, 0, 2, 6, 5, 1, 3, 9, 8, 3, 8, 2, 0, 5, 4, 1, 9, 6, 7, 2, 8, 7, 0, 1, 9, 3, 6, 5, 4, 1, 7, 6, 2, 0, 4, 9, 5, 8, 3, 4, 9, 8, 1, 2, 7, 6, 3, 5, 0, 2, 4, 1, 9, 8, 5, 6, 7, 3, 0, 4, 9, 3, 2, 1, 5, 8, 6, 7, 0, 1, 7, 9, 8, 5, 0, 3, 6, 4, 2, 1, 8, 7, 6, 2, 3, 5, 0, 9, 4, 4, 8, 1, 3, 5, 9, 2, 0, 6, 7, 1, 2, 6, 3, 0, 7, 5, 4, 8, 9, 3, 9, 1, 2, 8, 0, 6, 4, 5, 7, 1, 9, 0, 5, 2, 6, 8, 3, 4, 7, 1, 2, 8, 6, 0, 3, 7, 9, 4, 5, 2, 4, 5, 9, 6, 1, 8, 3, 7, 0, 4, 7, 2, 5, 6, 1, 3, 0, 8, 9]
    chrom = [1, 4, -1, -1, -1, -1, -1, -1, -1, -1, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, 5, 8, -1, -1, -1, -1, -1, -1, -1, -1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, 6, 0, -1, -1, -1, -1, -1, -1, -1, -1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, 5, 8, -1, -1, -1, -1, -1, -1, -1, -1, 7, 6, 0, -1, -1, -1, -1, -1, -1, -1, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, 7, 6, 0, -1, -1, -1, -1, -1, -1, -1, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, 2, 3, 9, -1, -1, -1, -1, -1, -1, -1, 3, 9, -1, -1, -1, -1, -1, -1, -1, -1, 3, 9, -1, -1, -1, -1, -1, -1, -1, -1, 2, 0, -1, -1, -1, -1, -1, -1, -1, -1, 1, 4, -1, -1, -1, -1, -1, -1, -1, -1, 5, 8, -1, -1, -1, -1, -1, -1, -1, -1]
    chrom = [4, 0, 3, 2, 1, 5, 6, 9, 8, 7, 6, 1, 8, 5, 4, 9, 2, 3, 0, 7, 0, 5, 8, 6, 7, 2, 4, 3, 9, 1, 6, 8, 9, 3, 1, 0, 2, 7, 5, 4, 6, 7, 0, 3, 2, 9, 4, 1, 5, 8, 4, 6, 3, 2, 0, 5, 8, 9, 1, 7, 9, 5, 1, 4, 6, 0, 7, 3, 2, 8, 2, 7, 3, 6, 5, 1, 0, 4, 8, 9, 8, 6, 7, 2, 0, 4, 9, 1, 3, 5, 2, 7, 3, 8, 5, 0, 1, 9, 4, 6, 9, 8, 4, 7, 2, 6, 1, 0, 3, 5, 9, 3, 4, 2, 6, 8, 7, 1, 5, 0, 0, 1, 7, 5, 8, 9, 6, 2, 4, 3, 0, 8, 1, 4, 6, 3, 7, 5, 9, 2, 9, 4, 1, 8, 6, 5, 2, 0, 7, 3]
    chrom = [2, 3, 9, 4, 0, 7, 5, 6, 1, 8, 1, 2, 9, 6, 8, 3, 5, 7, 0, 4, 6, 2, 5, 8, 7, 4, 0, 9, 3, 1, 6, 3, 1, 0, 5, 4, 9, 7, 2, 8, 2, 6, 5, 1, 0, 4, 3, 7, 8, 9, 6, 7, 8, 4, 0, 1, 3, 5, 2, 9, 6, 9, 0, 3, 1, 2, 8, 4, 7, 5, 1, 2, 9, 8, 4, 5, 0, 3, 6, 7, 1, 2, 8, 0, 4, 3, 6, 9, 5, 7, 3, 9, 0, 6, 8, 1, 5, 2, 4, 7, 3, 5, 2, 7, 9, 1, 4, 0, 6, 8, 0, 5, 2, 8, 4, 3, 6, 1, 7, 9, 3, 5, 2, 0, 9, 8, 1, 4, 6, 7, 5, 3, 6, 7, 4, 0, 2, 1, 9, 8, 9, 4, 7, 0, 3, 8, 2, 1, 6, 5, 9, 8, 3, 1, 5, 2, 4, 0, 6, 7, 6, 8, 4, 0, 9, 7, 1, 5, 2, 3, 9, 2, 4, 7, 6, 5, 0, 3, 8, 1, 1, 4, 9, 0, 8, 6, 5, 3, 2, 7, 6, 5, 0, 8, 9, 4, 3, 7, 1, 2, 0, 8, 7, 9, 6, 2, 4, 1, 5, 3, 3, 5, 2, 6, 4, 7, 0, 9, 8, 1, 9, 6, 8, 0, 2, 7, 4, 3, 1, 5, 4, 6, 7, 8, 1, 0, 3, 9, 5, 2, 8, 3, 4, 0, 5, 7, 6, 2, 1, 9, 5, 3, 2, 0, 4, 8, 7, 9, 1, 6, 5, 9, 2, 1, 3, 6, 0, 7, 8, 4, 6, 8, 9, 2, 5, 7, 3, 1, 0, 4, 1, 3, 2, 9, 7, 6, 5, 0, 4, 8, 6, 0, 9, 4, 3, 1, 7, 5, 8, 2, 6, 7, 5, 3, 8, 1, 9, 2, 0, 4, 6, 2, 8, 1, 9, 3, 0, 7, 4, 5, 1, 6, 5, 4, 2, 8, 7, 3, 9, 0, 4, 5, 7, 1, 3, 8, 9, 6, 0, 2, 6, 7, 8, 5, 0, 3, 2, 9, 1, 4, 2, 4, 0, 6, 7, 1, 5, 3, 8, 9, 6, 3, 5, 1, 9, 8, 0, 7, 2, 4, 6, 0, 8, 1, 4, 7, 9, 5, 3, 2, 8, 1, 0, 5, 9, 7, 4, 2, 3, 6, 7, 3, 1, 6, 2, 9, 4, 8, 0, 5]
    # chrom = [9, 4, 8, -1, -1, -1, -1, -1, -1, -1, 2, 8, -1, -1, -1, -1, -1, -1, -1, -1, 9, 3, -1, -1, -1, -1, -1, -1, -1, -1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1, 6, 7, 3, -1, -1, -1, -1, -1, -1, -1, 2, 1, 5, -1, -1, -1, -1, -1, -1, -1, 7, 3, -1, -1, -1, -1, -1, -1, -1, -1, 0, 8, -1, -1, -1, -1, -1, -1, -1, -1, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, 6, 1, 5, -1, -1, -1, -1, -1, -1, -1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, 2, 1, 4, 8, -1, -1, -1, -1, -1, -1, 6, 0, 4, 8, -1, -1, -1, -1, -1, -1, 0, 4, 8, -1, -1, -1, -1, -1, -1, -1, 4, 8, -1, -1, -1, -1, -1, -1, -1, -1]
    # chrom =  [6, 9, 4, 8, -1, -1, -1, -1, -1, -1, 2, 1, 5, -1, -1, -1, -1, -1, -1, -1, 2, 3, -1, -1, -1, -1, -1, -1, -1, -1, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1, 9, 3, -1, -1, -1, -1, -1, -1, -1, -1, 9, 7, 3, -1, -1, -1, -1, -1, -1, -1, 7, 3, -1, -1, -1, -1, -1, -1, -1, -1, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, 1, 4, 8, -1, -1, -1, -1, -1, -1, -1, 4, 8, -1, -1, -1, -1, -1, -1, -1, -1, 6, 9, 5, -1, -1, -1, -1, -1, -1, -1, 4, 8, -1, -1, -1, -1, -1, -1, -1, -1, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, 2, 8, -1, -1, -1, -1, -1, -1, -1, -1]

    # chrom =  [4, 3, 0, -1, -1, 1, 2, 0, -1, -1, 1, 2, 0, -1, -1, 1, 0, -1, -1, -1, 4, 3, 0, -1, -1]
    '''
    better
    '''
    # chrom = [3, 4, 0, -1, -1, 1, 4, 0, -1, -1, 1, 2, 0, -1, -1, 1, 2, 0, -1, -1, 1, 4, 0, -1, -1]
    '''
    worse
    '''
    encode =  np.zeros((ins._robNum, ins._taskNum), dtype=int)
    i = 0
    for robID in range(ins._robNum):
        for taskID in range(ins._taskNum):
            encode[robID][taskID] = chrom[i]
            i += 1
    # mpda_decode_nb = MPDA_Decode_Discrete_NB()
    print(encode)
 #    encode = [[ 2, -1, -1, -1, -1, -1, -1, -1],
 # [ 7,  2, -1, -1, -1, -1, -1, -1],
 # [ 0, -1, -1, -1, -1, -1, -1, -1],
 # [ 2, -1, -1, -1, -1, -1, -1, -1],
 # [ 6,  0, -1, -1, -1, -1, -1, -1],
 # [ 3, -1, -1, -1, -1, -1, -1, -1],
 # [ 6,  5,  0, -1, -1, -1, -1, -1],
 # [ 4,  1,  6,  5,  7,  2,  0,  3]]
    validBoolean,actSeq,invalidTask = decoder.decode(encode)

    # x = generateRandEncode(robNum= ins._robNum, taskNum= ins._taskNum)
    # print(x)
    # validStateBoolean,actSeq = decoder.decode(x)
    # print(actSeq.convert2MultiPerm(ins._robNum))
    for perm in actSeq.convert2MultiPerm(ins._robNum):
        print(perm)
    # print(actSeq)
    actSeqDecoder = MPDADecoderActionSeq(ins)
    actSeqDecoder.decode(actSeq)
    print('failed task ', invalidTask)

    actSeqDecoder.drawActionSeqGantt()
    actSeqDecoder.drawTaskScatter()
    # actSeqDecoder.drawTaskDependence()