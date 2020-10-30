# -*- coding: utf-8 -*-
"""
Created on Tue Sep  4 10:12:55 2018
this is a base DECODE CLASS for the MPDA problem

@author: robot
"""
from MPDA_decode.instance import Instance
import os,sys
from scipy.optimize import differential_evolution
import  numpy as np
from MPDA_decode.robot import RobotState,Robot
from MPDA_decode.task import Task
import  random as rd
from collections import  namedtuple
import  random
import  copy
from MPDA_decode.action import ActionSeq,ActionTuple,EventType,RobTaskPair,TaskSeq,TaskStatusTuple,InvalidType
from enum import Enum
# import logging
from MPDA_decode.action import ActionSeq,ActionTuple
import math


class InvalidStateException(Exception):
    '''
    Custom exception types about InvalidStateException
    '''
    def __init__(self):
        err = 'InvalidStateException'
        super(InvalidStateException, self).__init__(err)

class RobotStuckException(Exception):
    '''
    Custom exception types about RobotStuckException
    '''

    def __init__(self):
        err = 'RobotStuckException'
        super(RobotStuckException, self).__init__(err)

class CalType(Enum):
    arriveCond = 1
    leaveCond = 2
    endCond = 3
    backCond = 4
    stateInvalidCond = 5

AbsolutePath = os.path.abspath(__file__)
# 将相对路径转换成绝对路径
SuperiorCatalogue = os.path.dirname(AbsolutePath)
# 相对路径的上级路径
BaseDir = os.path.dirname(SuperiorCatalogue)


class MPDA_Decode_Discrete_Base(object):
    _ins = object
    def __init__(self):
        # if type(self._ins) != Instance:
        #     raise TypeError("MPDA_Decode_Discrete_Base._ins must be the Instance class")
        '''
        some paras are const during calculation process
        '''
        self.robNum = MPDA_Decode_Discrete_Base._ins._robNum
        self.taskNum = MPDA_Decode_Discrete_Base._ins._taskNum
        self.threhold = MPDA_Decode_Discrete_Base._ins._threhold
        self.robAbiLst = MPDA_Decode_Discrete_Base._ins._robAbiLst
        self.robVelLst = MPDA_Decode_Discrete_Base._ins._robVelLst
        self.taskStateLst = MPDA_Decode_Discrete_Base._ins._taskStateLst
        self.taskRateLst = MPDA_Decode_Discrete_Base._ins._taskRateLst
        self.rob2taskDisMat = MPDA_Decode_Discrete_Base._ins._rob2taskDisMat
        self.taskDisMat = MPDA_Decode_Discrete_Base._ins._taskDisMat

        self.encode = np.zeros((self.robNum, self.taskNum), dtype=int)

        self.taskLst = []
        self.robotLst = []
        self.cmpltLst = []
        self.decodeTime = 0

        self.robInfoLst = []
        self.taskInfoLst = []
        self.decodeTimeLst = []

        # self._degBool = False
    '''
    generate a random encode 
    '''
    def generateRandEncode(self):
        for i in range(self.robNum):
            permLst = [x for x in range(self.taskNum)]
            random.shuffle(permLst)
            self.encode[i][:] = permLst

    def decode(self):
        '''
        ready to construct
        '''
        pass

    def initStates(self):
        '''
        initialize states of decode method
        '''
        self.taskLst.clear()
        self.robotLst.clear()
        self.cmpltLst.clear()
        self.cmpltLst = [False] * self.taskNum
        self._actionSeq = ActionSeq()

        for i in range(self.robNum):
            rob = Robot()
            rob.ability = self.robAbiLst[i]
            rob.vel = self.robVelLst[i]
            rob.encodeIndex = 0
            rob.taskID, rob.encodeIndex, stopBool = self.getRobTask(robID=i, encodeIndex=0)
            if not stopBool:
                dis = self.rob2taskDisMat[i][rob.taskID]
                dis_time = dis / rob.vel
                rob.arriveTime = dis_time
            rob.stopBool = stopBool
            rob.stateType = RobotState['onRoad']
            rob.leaveTime = 0
            self.robotLst.append(rob)

        for i in range(self.taskNum):
            task = Task()
            task.cState = self.taskStateLst[i]
            task.initState = self.taskStateLst[i]
            task.cRate = self.taskRateLst[i]
            task.initRate = self.taskRateLst[i]
            task.threhod = self.threhold
            task.cmpltTime = sys.float_info.max
            self.taskLst.append(task)

        self.decodeTime = 0
        self.validStateBool = True

    #    @profile
    def decodeProcessor(self):
        invalidFitness = False
        backBool = False
        validStateBool = True
        while not self._calEndCond():
            cal_type, actionID = self.findActionID()
            if cal_type == CalType['arriveCond']:
                # =============================================================================
                # arrive event
                # =============================================================================

                rob = self.robotLst[actionID]
                arriveTime = rob.arriveTime
                encodeInd = rob.encodeIndex
                taskID = self.encode[actionID][encodeInd]

                # try:
                # except Exception as e:
                #     print(e)



                # ActionTuple
                if self.taskCmplt(taskID):
                    # =============================================================================
                    #  the task has been cmplt
                    # =============================================================================
                    self.arriveCmpltTask(actionID, encodeInd)
                else:
                    # =============================================================================
                    # the  task has not been cmplt
                    # =============================================================================
                    task = self.taskLst[taskID]
                    rob.taskID = taskID
                    validStateBool = task.calCurrentState(arriveTime)
                    if not validStateBool:
                        break
                    task.cRate = task.cRate - rob.ability
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
                    self.decodeTime = rob.arriveTime
                    self._actionSeq.append(ActionTuple(robID= actionID,taskID = taskID,
                                                   eventType = EventType.arrive,eventTime=arriveTime))

            # =============================================================================
            #  begin the leave condition for
            # =============================================================================
            if cal_type == CalType['leaveCond']:
                rob = self.robotLst[actionID]
                #                print(actionID)
                taskID = rob.taskID
                #                try:
                #                    if  taskID < 0:
                #                        raise Exception('taskID < 0')
                #                except Exception as e:
                #                    print(e)
                #                print(taskID)
                task = self.taskLst[taskID]
                # self._actionSeq.append(ActionTuple(robID= actionID,taskID = taskID,
                #                                    eventType = EventType.leave,eventTime=arriveTime))
                if self.cmpltLst[taskID] == True:
                    self.leaveCmpltTask(actionID)
                    # raise  Exception('leaveCmpltTask')
                #                    print(self.cmpltLst)
                #                    validStateBool = True
                #                    while True:
                #                        print('bug',taskID)
                #                        return
                else:
                    validStateBool = task.calCurrentState(rob.leaveTime)
                    if not validStateBool:
                        break
                    if (task.isCmplt()):
                        self.cmpltLst[taskID] = True
                        try:
                            self.updateEncode(taskID)
                        except Exception as e:
                            print(e)
                            raise  Exception("updateEnocde Error")
                        coordLst = self.findCoordRobot(actionID)
                        for coordID in coordLst:
                            self.updateRobLeaveCond(coordID)
                            self.robotLst[coordID].cmpltTaskLst.append(taskID)
                    self.updateRobLeaveCond(actionID)
                    self.robotLst[actionID].cmpltTaskLst.append(taskID)
                    self.robotLst[actionID].cmpltTaskID = taskID


                '''
                没有debug信息 将来logging
                '''
                # if self._degBool:
                #     self.deg.write('taskID ' + str(taskID) + ' has been cmplt\n')
                #     #                self.deg.write('leaveChanged\n')
                #     self.saveRobotInfo()

                task.cmpltTime = rob.leaveTime
                self.decodeTime = rob.leaveTime
                if self._actionSeq.actionTime != self.decodeTime:
                    raise  Exception('=d=')
            #                print(taskID,' cmpltTime ', task.cmpltTime)
            if cal_type == CalType['endCond']:
                invalidFitness = True
                #                raise Exception('end-Condition-bug, robots have been stuck')
                # print(self._actionSeq)
                # raise RobotStuckException()
                break
            if cal_type == CalType['backCond']:
                backBool = True
                self.backRobID = actionID
                self.backTaskID = self.robotLst[actionID].taskID
                self.backArriveTime = self.robotLst[actionID].arriveTime
                self.backInfo = self.robotLst[actionID].variableInfo()
                break
            #
            #                    print(task.cRate)
            #                    print(task.cRate)
            # =============================================================================
            #  the state is too big  the decode process is wrong
            # =============================================================================
            if not validStateBool:
                break
        #            print('circleTime = ', circleTime)
        #            print('decodeTime = ', self.decodeTime)
        #            circleTime += 1
        #            if circleTime > 3000:
        #                break
        #            print(self.cmpltLst)

        if not validStateBool:
            cal_type = CalType['stateInvalidCond']
            self.validStateBool = False
            #            raise Exception('stateInvalidCond-bug, the state is too enormous')
            # raise InvalidStateException()
        #        print(cal_type)
        return cal_type

    #            break

    def allTaskCmplt(self):
        if False in self.cmpltLst:
            return False
        else:
            return True

    def _allRobStop(self):
        for rob in self.robotLst:
            if rob.stopBool == False:
                return False
        return True
    def _calEndCond(self):
        pass
    def findActionID(self):
        cal_type = CalType['endCond']
        actionID = -1
        minTime = sys.float_info.max
        for i in range(self.robNum):
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

        self.saveEventInMemory()

        if math.isclose(minTime,self.decodeTime) or minTime >self.decodeTime:
            pass
        else:
            cal_type = CalType['backCond']
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
        for i in range(self.robNum):
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

    def calRoadDur(self, taskID1, taskID2, robID):
        '''
        calculate the time fragment from the time when robID leaves the taskID1 to
        the time when rob arrives the taskID2
        '''
        dis = self.taskDisMat[taskID1][taskID2]
        rob = self.robotLst[robID]
        roadDur = dis / rob.vel
        return roadDur

    def updateEncode(self, cmpltTaskID):
        '''
        correct the encode,
        delete furture tasks which have been completed.
        '''
        for i in range(self.robNum):
            rob = self.robotLst[i]
            for j in range(rob.encodeIndex + 1, self.taskNum):
                if self.encode[i][j] == cmpltTaskID:
                    self.encode[i][j] = -1

    def updateRobLeaveCond(self, robID):
        '''
        update robot's state when the leave event has been triggered.
        '''
        rob = self.robotLst[robID]
        preTaskID = rob.taskID
        self._actionSeq.append(ActionTuple(robID=robID, taskID=rob.taskID,
                                           eventType=EventType.leave, eventTime=rob.leaveTime))

        while True:
            if rob.encodeIndex == (self.taskNum - 1):
                rob.stopBool = True
                break
            rob.encodeIndex += 1
            taskID = self.encode[robID][rob.encodeIndex]
            if self.taskCmplt(taskID):
                continue
            else:
                roadDur = self.calRoadDur(preTaskID, taskID, robID)
                arriveTime = rob.leaveTime + rob.roadDur
                if arriveTime > self.taskLst[taskID].cmpltTime:
                    self.encode[robID][rob.encodeIndex] = -1
                    #                    print('optimal')
                    continue
                rob.roadDur = roadDur
                rob.taskID = taskID
                rob.arriveTime = rob.leaveTime + rob.roadDur
                rob.stateType = RobotState['onRoad']
                break

    def taskCmplt(self, taskID):
        '''
        taskID has been completed or not
        '''
        cmplt = False
        if taskID < 0:
            cmplt = True
        else:
            if self.cmpltLst[taskID]:
                cmplt = True
        return cmplt

    def getRobTask(self, robID=0, encodeIndex=0):
        '''
        get the robot next task ID
        '''
        stopBool = False
        while True:
            if encodeIndex == self.taskNum:
                stopBool = True
                break
            taskID = self.encode[robID][encodeIndex]
            if taskID < 0:
                encodeIndex += 1
                continue
            else:
                break
        return taskID, encodeIndex, stopBool


    def calMakespan(self):
        # makespan for the MPDA problem
        leaveTimeLst = [rob.leaveTime for rob in self.robotLst]
        makespan = max(leaveTimeLst)
        return makespan

    def saveEncode(self):
        '''
        save encode information into the deg2 files

        '''
        pass
        # for i in range(self.robNum):
        #     lst = list(self.encode[i][:])
        #     rd.writeConf(self.deg2, str(i), lst)
        # self.deg2.flush()

    def saveRobotInfo(self):
        '''
        save robot information into the deg files
        '''
        pass
        self.deg.write('\n')
        for i in range(self.robNum):
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
            str_lst = [str(x) for x in lst]
            robInfo = '  '
            robInfo = robInfo.join(str_lst)
            self.deg.write(robInfo + '\n')
        self.deg.write('\n')
        self.deg.flush()

    def saveEventInMemory(self):
        pass

    def leaveCmpltTask(self, actionID):
        pass

    def arriveCmpltTask(self, actionID, encodeInd):
        pass

    def genNoBacktrackEncode(self):
        '''
        generate the no-backtrack encode
        '''
        encode = np.zeros((self.robNum, self.taskNum), dtype=int)
        for i in range(self.robNum):
            ind = 0
            for j in range(self.taskNum):
                if self.encode[i][j] != -1:
                    encode[i][ind] = self.encode[i][j]
                    ind += 1
        return encode

    def endDeg(self):
        self.deg.close()
        self.deg2.close()


class MPDA_Decode_Discrete_NB(MPDA_Decode_Discrete_Base):
    def __init__(self):
        super(MPDA_Decode_Discrete_NB,self).__init__()
        # raise  Exception("xxx")
        pass

    def leaveCmpltTask(self, actionID):
        self.updateRobLeaveCond(actionID)
        # raise Exception('leaveCmpltTask')

    def arriveCmpltTask(self,actionID,encodeInd):
        rob = self.robotLst[actionID]
        rob.leaveTime = rob.arriveTime
        rob.stateType = RobotState['onTask']
        self.decodeTime = rob.arriveTime
        taskID = rob.taskID
        arriveTime = rob.arriveTime
        self._actionSeq.append(ActionTuple(robID=actionID, taskID=taskID,
                                       eventType=EventType.arrive, eventTime=arriveTime))

    def decode(self,encode):
        self.encode = encode
        self.initStates()
        self._calEndCond = self.allTaskCmplt
        cal_type = self.decodeProcessor()
        if cal_type == CalType.stateInvalidCond:
            #            print('invalidState')
            #            invalidState = True
            makespan = sys.float_info.max
        else:
            #            print('validState')
            makespan = self.calMakespan()
        self.saveEncode()
        return makespan

    def partDecode(self,encode,taskID :int = 0):
        self.encode = encode
        self.initStates()
        cal_type = self.decodeProcessor()
        if cal_type == CalType.stateInvalidCond:
            #            print('invalidState')
            #            invalidState = True
            makespan = sys.float_info.max
        else:
            #            print('validState')
            makespan = self.calMakespan()
        taskRes = self.taskLst[taskID]
        return taskRes
    # def lastTask

    def __str__(self):
        return 'mpda_decode_discrete_no_back ' +str(self._ins)

'''
re-calculation 
'''
class MPDA_Decode_Discrete_RC(MPDA_Decode_Discrete_Base):
    def __init__(self):
        super(MPDA_Decode_Discrete_RC,self).__init__()
        # raise  Exception("xxx")
        pass

    def arriveCmpltTask(self,actionID,encodeInd):
        self.encode[actionID][encodeInd] = -1
        rob = self.robotLst[actionID]
        taskID = -1
        while True:
            if len(rob.cmpltTaskLst) == 0:
                if encodeInd == self.taskNum - 1:
                    rob.stopBool = True
                    break

                taskID = self.encode[actionID][encodeInd]

                if taskID < 0:
                    encodeInd  += 1
                    continue
                dis  = self.rob2taskDisMat[actionID][taskID]
                dis_time = dis/rob.vel
                rob.arriveTime = dis_time
                rob.encodeIndex = encodeInd
                break
            else:
                if encodeInd == self.taskNum - 1:
                    rob.stopBool = True
                    break
                taskID = self.encode[actionID][encodeInd]
                if taskID < 0:
                    encodeInd  += 1
                    continue
                preTaskID = rob.cmpltTaskLst[-1]
                roadDur = self.calRoadDur(preTaskID,taskID,actionID)
                rob.arriveTime  = rob.leaveTime + roadDur
                rob.encodeIndex = encodeInd
                break
        rob.taskID = taskID

    def leaveCmpltTask(self, actionID):
        raise Exception('leaveCmpltTask')

    def decode(self,encode):
        # circleTime = 0
        self.encode = encode
        self._calEndCond = self.allTaskCmplt
        while True:
            # print('whileCircle = ',circleTime)
            # circleTime += 1
            #            start = time.clock()
            self.initStates()
            cal_type = self.decodeProcessor()
            #            end = time.clock()
            #            print(end - start)
            if cal_type == CalType['backCond']:
                continue
            else:
                break
        #        print(self.cmpltLst)
        #        self.saveEncode()
        if cal_type == CalType.stateInvalidCond:
            #            print('invalidState')
            #            invalidState = True
            makespan = sys.float_info.max
        else:
            #            print('validState')
            makespan = self.calMakespan()
        #        self.saveEncode()
        return makespan


if __name__ == '__main__':
    insName = '14_14_ECCENTRIC_RANDOMCLUSTERED_SVLCV_LVLCV_thre0.1MPDAins.dat'
    ins = Instance(BaseDir + '\\benchmark\\' + insName)
    print(ins)
    MPDA_Decode_Discrete_Base._ins = ins
    MPDA_Decode_Discrete_NB._ins = ins

    mpda_dis_decode = MPDA_Decode_Discrete_NB()

    print(mpda_dis_decode)
    mpda_decode_base = MPDA_Decode_Discrete_Base()
    encode = np.zeros((ins.robNum, ins.taskNum), dtype=int)
    random.seed(3)

    while True:
        for i in range(ins.robNum):
            permLst = [x for x in range(ins.taskNum)]
            random.shuffle(permLst)
            encode[i][:] = permLst
        # print('encode =  ',encode)
        encode1 = copy.deepcopy(encode)
        makespan1 = mpda_dis_decode.decode(encode1)
        print("makespan1 = ",makespan1)
        print('encode1 =  ',encode1)

        mpda_dis_decode_rc = MPDA_Decode_Discrete_RC()
        encode2 = copy.deepcopy(encode)
        makespan2 = mpda_dis_decode_rc.decode(encode2)
        print("makespan2 = ",makespan2)
        print('encode2 =  ',encode2)

        if makespan1 != makespan2:
            raise  Exception('xxxx')


    f_deg = open(BaseDir + '\\debugData\\discrete_task_seq.txt', 'w')
    f_deg.write(str(mpda_dis_decode._actionSeq))
    # f_deg.write(str(action_seq))
    f_deg.flush()
    # f_deg.close()
    # mpda_dis_decode.partDecode(encode,taskID=)

    print(mpda_dis_decode._actionSeq.examinationSelf())
    print(mpda_dis_decode._actionSeq.actionSeq2DiscreteEncode(ins.robNum,ins.taskNum))

    f_deg.write(str(encode) + '\n')
    f_deg.write(str(mpda_dis_decode._actionSeq.actionSeq2DiscreteEncode(ins.robNum,ins.taskNum)))

    f_deg.close()


    # print('0-0')