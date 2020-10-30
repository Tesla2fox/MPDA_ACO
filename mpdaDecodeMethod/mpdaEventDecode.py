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

def generateRandEventEncode(robNum,taskNum):
    encode = []
    for i in range(robNum):
        for j in range(taskNum):
            encode.append(RobTaskPair(robID=i, taskID=j))
    np.random.shuffle(encode)
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

degBoolean = True



class MPDADecoderEvent(object):
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
        self.eventEncode = x
        for i,unit in enumerate(self.eventEncode):
            self._degFile.write(str(i) + ' ' + str(unit) + '\n')
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
        self.eventEncodeInd = 0
        for i in range(self._robNum):
            rob = Robot()
            rob._ability = self._robAbiLst[i]
            rob._vel = self._robVelLst[i]
            # rob.encodeIndex = 0
            # rob.taskID, rob.encodeIndex, stopBool = self.getRobTask(robID=i, encodeIndex=0)
            # if not stopBool:
            #     dis = self._rob2taskDisMat[i][rob.taskID]
            #     dis_time = dis / rob._vel
            #     rob.arriveTime = dis_time
            rob.stopBool = False
            rob.arriveTime = sys.float_info.max
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
        self.decodeTime = 0
        self.encodeInd = 0
        # self.validStateBool = True
    # self.findActionID()
    # def findActionID(self):


    def decodeProcessor(self):
        while not self.allTaskCmplt():
            cal_type, actionID = self.newFindActionID()
            if cal_type  == CalType['arriveCond']:
                rob = self.robotLst[actionID]
                arriveTime = rob.arriveTime
                taskID = self.eventEncode[self.eventEncodeInd].taskID
                self.eventEncodeInd += 1
                # self.eventEncode[self.eventEncodeInd]
                # encodeInd = rob.encodeIndex
                # taskID = self.encode[actionID][encodeInd]
                self._actSeq.append(ActionTuple(robID =actionID,taskID= taskID, eventType = EventType.arrive,eventTime = arriveTime))

                if self.cmpltLst[taskID]:
                    # =============================================================================
                    #  the task has been cmplt
                    # =============================================================================
                    rob = self.robotLst[actionID]
                    rob.leaveTime = rob.arriveTime
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
    def newFindActionID(self):
        cal_type = CalType['endCond']
        actionID = sys.float_info.max
        minTime = sys.float_info.max
        eventRobID,eventTaskID = self.eventEncode[self.eventEncodeInd]
        print('eventTaskID = ',eventTaskID)
        for i in range(self._robNum):
            rob = self.robotLst[i]
            if rob.stateType == RobotState['onRoad'] and i == eventRobID:
                if rob.leaveTime == 0:
                    dis = self._rob2taskDisMat[i][rob.taskID]
                    dis_time = dis / rob._vel
                    rob.arriveTime = dis_time
                else:
                    raise Exception('xx')
                if rob.arriveTime < self.decodeTime:
                    rob.arriveTime = self.decodeTime
                actionID = eventRobID
                cal_type = CalType['arriveCond']
                self.decodeTime = rob.arriveTime
            if rob.stateType == RobotState['onTask'] and i != eventRobID:
                if rob.leaveTime < minTime:
                    minTime = rob.leaveTime
                    cal_type = CalType['leaveCond']
                    actionID = i

        # print('new find action ID')
        # self._degFile.write()
        self.saveRobotInfo(degFile=self._degFile)
        self._degFile.write('cal_type = '+ str(cal_type) + '\n')
        self._degFile.write('actionID = ' + str(actionID) +'\n')
        self._degFile.write('eventTime = '+ str(self.decodeTime) + '\n')
        self._degFile.write('actSeq = ' + str(self._actSeq) + '\n')
        # print('actionID = ',actionID)
        # print('eventTime = ',self.decodeTime)
        # print('actSeq = ',self._actSeq)
        # print(' __ ')

        return cal_type, actionID


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
    print('test_mpdaEventDecoder')
    ins = MPDAInstance()
    insFileName = BaseDir + '//conbenchmark//8_8_CLUSTERED_CLUSTERED_MSVFLV_QUADRANT_MSVFLV_thre0.1MPDAins.dat'
    ins.loadCfg(fileName=insFileName)
    decoder = MPDADecoderEvent(ins)
    np.random.seed(1)
    x = generateRandEventEncode(robNum=ins._robNum, taskNum=ins._taskNum)
    print('event sol = ', x)
    decoder.decode(x)
    exit()
    f_con = open(BaseDir +'//b_conBenchmark.dat','w')
    print(BaseDir)
    for root, exsit_dirs, files in os.walk(BaseDir+'//conBenchmark'):
        # print(files)
        break
        for file in files:
            print(file)
            # insFileName = BaseDir + '//conBenchmark//5_5_CLUSTERED_CLUSTERED_UNITARY_SVSCV_LVSCV_thre0.1MPDAins.dat'
            try:
                ins.loadCfg(fileName=insFileName)
            except:
                continue
            decoder = MPDADecoderCon(ins)
            invalidNum = 0
            for _ in range(100):
                x = generateRandEncode(robNum=ins._robNum, taskNum=ins._taskNum)
                validStateBoolean, actSeq, invalidTask = decoder.decode(x)
                actSeqDecoder = MPDADecoderActionSeq(ins)
                actSeqDecoder.decode(actSeq)
                if len(decoder._invalidTaskLst) !=0:
                    invalidNum += 1
            print(invalidNum)
            if invalidNum > 20:
                print(file)
                f_con.write(file +'\n')
                f_con.flush()
            # break
        break
    # exit()

    ins = MPDAInstance()
    insFileName = BaseDir + '//conBenchmark//' + '8_8_CLUSTERED_CLUSTERED_MSVFLV_QUADRANT_MSVFLV_thre0.1MPDAins.dat'
    # insFileName = BaseDir + '//conBenchmark//5_5_CLUSTERED_CLUSTERED_UNITARY_SVSCV_LVSCV_thre0.1MPDAins.dat'
    ins.loadCfg(fileName=insFileName)
    decoder = MPDADecoderCon(ins)



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
    encode =  np.zeros((ins._robNum, ins._taskNum), dtype=int)
    i = 0
    for robID in range(ins._robNum):
        for taskID in range(ins._taskNum):
            encode[robID][taskID] = chrom[i]
            i += 1
    # mpda_decode_nb = MPDA_Decode_Discrete_NB()
    print(encode)
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
    actSeqDecoder.drawTaskDependence()