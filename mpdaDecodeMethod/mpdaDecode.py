import readcfg as r_d
from mpdaInstance import MPDAInstance
import os, sys
from mpdaDecodeMethod.mpdaRobot import RobotState,Robot
from mpdaDecodeMethod.mpdaTask import  Task
from mpdaDecodeMethod.mpdaDecoderActSeq import ActionSeq,ActionTuple,EventType,MPDADecoderActionSeq
import numpy as np
from enum import Enum
from collections import  namedtuple

RobTaskPair = namedtuple('RobTaskPair',['robID','taskID'])
import math

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




def generateRandPopEncode(robNum,taskNum):
    pop = []
    encode = np.zeros((robNum, taskNum),dtype =int)
    for i in range(robNum):
        permLst = [x for x in range(taskNum)]
        np.random.shuffle(permLst)
        encode[i][:] = permLst
    robIndLst = [0 for _ in range(robNum)]

    while len(pop) == (robNum*taskNum):
        rdRobID = np.random.randint(0,robNum -1)
        rdTaskID = encode[rdRobID][robIndLst[rdRobID]]
        pop.append(RobTaskPair(rdRobID,rdTaskID))
        robIndLst[rdRobID] += 1

    '''
    there are still some problems to construct a 
    '''






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


class MPDADecoder(object):
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

    def decode(self, x):
        self.encode = x
        self._actSeq = ActionSeq()
        self.initStates()
        # if self.decodeProcessor():
        validStateBoolean = self.decodeProcessor()
        if degBoolean:
            self._degFile.write(str(self.cmpltLst))
        # print(self._actSeq.convert2MultiPerm(self._robNum))
        return validStateBoolean, self._actSeq
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
            task.cState = self._taskStateLst[i]
            task._initState = self._taskStateLst[i]
            task.cRate = self._taskRateLst[i]
            task._initRate = self._taskRateLst[i]
            task._threhod = self._threhold
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
                # if degBoolean:
                self._actSeq.append(ActionTuple(robID =actionID,taskID= taskID, eventType = EventType.arrive,eventTime = arriveTime))

                if self.cmpltLst[taskID]:
                    # =============================================================================
                    #  the task has been cmplt
                    # =============================================================================
                    rob = self.robotLst[actionID]
                    rob.leaveTime = rob.arriveTime
                    rob.taskID = taskID
                    rob.stateType = RobotState['onTask']
                    # if degBoolean:
                    self._actSeq._arrCmpltTaskLst.append((actionID, taskID))
                else:
# =============================================================================
# the task has not been cmplt
# =============================================================================
                    task = self.taskLst[taskID]
                    rob.taskID = taskID
                    validStateBool = task.calCurrentState(arriveTime)
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
                # if degBoolean:
                self._actSeq.append(ActionTuple(robID =actionID,taskID= taskID, eventType = EventType.leave,eventTime = rob.leaveTime))
                task.cmpltTime = rob.leaveTime

                coordLst = self.findCoordRobot(actionID)
                for coordID in coordLst:
                    self.updateRobLeaveCond(robID = coordID)
                    # if degBoolean:
                    self._actSeq.append(ActionTuple(robID = coordID, taskID= taskID,eventType = EventType.leave,eventTime = task.cmpltTime))
                self.updateRobLeaveCond(robID = actionID)

                '''
                debug is here
                '''

                if degBoolean:
                    self._degFile.write(str(taskID) + ' have been completed\n')

            if cal_type == CalType.endCond:
                # print('stuck')
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
            if rob.encodeIndex == (len(self.encode[robID]) - 1):
                rob.stopBool = True
                break
            rob.encodeIndex += 1
            taskID = self.encode[robID][rob.encodeIndex]
            # print(taskID)
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
    def calMakespan(self):
        cmpltTime = []
        for task in self.taskLst:
            cmpltTime.append(task.cmpltTime)
        return max(cmpltTime)
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
if __name__ == '__main__':
    print('test_mpdaDecoder')


    print(BaseDir)
    ins = MPDAInstance()
    insFileName = BaseDir +'//f_anaBenchmark//20_80_ECCENTRIC_CLUSTERED_SVSCV_SVSCV_LVLCV_thre0.1MPDAins.dat'
    ins.loadCfg(fileName =  insFileName)

    decoder = MPDADecoder(ins)
    np.random.seed(2)
    x = []
    for _ in range(10):
        x.append(RobTaskPair(robID = np.random.randint(0, ins._robNum -1),taskID = np.random.randint(0, ins._taskNum -1)))
    print(x)

    x = generateRandEncode(robNum= ins._robNum, taskNum= ins._taskNum)
    # x = [RobTaskPair(robID = 1, taskID = 2),RobTaskPair(robID = 3,taskID = 4)]
    # print(x)
    # exit()
    x = [[6, 1, 0, 7, 5 ,4, 2, 3],
        [6, 4, 7 ,0 ,5 ,3 ,2 ,1],
        [3, 0 ,4 ,2 ,6 ,7 ,5 ,1],
        [3, 5 ,6 ,0 ,2 ,7 ,1 ,4],
        [4, 5 ,1 ,0 ,6 ,7 ,3 ,2],
        [6, 3 ,4 ,0 ,7 ,5 ,1 ,2],
        [1 ,0 ,3 ,6 ,7 ,4 ,5, 2],
        [1, 4, 7, 0, 2, 3, 6, 5]]
    x = generateRandEncode(robNum= ins._robNum, taskNum= ins._taskNum)

    import time
    start_time = time.time()
    validStateBoolean,actSeq = decoder.decode(x)
    end_time = time.time()
    print('time = ',end_time - start_time)

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


    encode =  np.zeros((ins._robNum, ins._taskNum), dtype=int)
    i = 0
    for robID in range(ins._robNum):
        for taskID in range(ins._taskNum):
            encode[robID][taskID] = chrom[i]
            i += 1
    # mpda_decode_nb = MPDA_Decode_Discrete_NB()
    print(encode)
    validBoolean,actSeq = decoder.decode(encode)

    # x = generateRandEncode(robNum= ins._robNum, taskNum= ins._taskNum)
    # print(x)
    # validStateBoolean,actSeq = decoder.decode(x)
    # print(actSeq.convert2MultiPerm(ins._robNum))
    for perm in actSeq.convert2MultiPerm(ins._robNum):
        print(perm)
    # print(actSeq)
    actSeqDecoder = MPDADecoderActionSeq(ins)
    actSeqDecoder.decode(actSeq)

    actSeqDecoder.drawActionSeqGantt()
    actSeqDecoder.drawTaskScatter()
    actSeqDecoder.drawTaskDependence()