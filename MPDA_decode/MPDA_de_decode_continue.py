# import MPDA_decode.instance
import copy
import os
import random
import random as rd
import sys
from collections import namedtuple
from enum import Enum

import numpy

from MPDA_decode.action import ActionSeq, ActionTuple, EventType, RobTaskPair, TaskSeq, TaskStatusTuple, InvalidType
from MPDA_decode.instance import Instance
from MPDA_decode.robot import RobotDe, RobotState
from MPDA_decode.task import Task

# import logging


OnTaskInfoTuple = namedtuple('OnTaskInfoTuple', ['changeRateTime', 'cRate', 'robID', 'robAbi'])

INF_NUM = sys.float_info.max
INF_INT_NUM = sys.maxsize


class SelectType(Enum):
    AllTask = 1
    OneTask = 2


class ActionSeqType(Enum):
    InOrder = 1
    OutOfOrder = 2


'''
random
'''
random.seed(1)

AbsolutePath = os.path.abspath(__file__)
# 将相对路径转换成绝对路径
SuperiorCatalogue = os.path.dirname(AbsolutePath)
# 相对路径的上级路径
BaseDir = os.path.dirname(SuperiorCatalogue)


class MPDA_DE_decode(object):
    _ins = object

    def __init__(self):
        if type(MPDA_DE_decode._ins) != Instance:
            raise TypeError("MPDA_DE_decode._ins must be the Instance class")

        '''
        some paras are const during calculation process
        '''
        self.robNum = MPDA_DE_decode._ins.robNum
        self.taskNum = MPDA_DE_decode._ins.taskNum
        self.threhold = MPDA_DE_decode._ins.threhold
        self.robAbiLst = MPDA_DE_decode._ins.robAbiLst
        self.robVelLst = MPDA_DE_decode._ins.robVelLst
        self.taskStateLst = MPDA_DE_decode._ins.taskStateLst
        self.taskRateLst = MPDA_DE_decode._ins.taskRateLst
        self.rob2taskDisMat = MPDA_DE_decode._ins.rob2taskDisMat
        self.taskDisMat = MPDA_DE_decode._ins.taskDisMat

        self._encode = []

        '''
        some paras are changed during calculation process
        '''
        self._taskLst = []
        self._robotLst = []
        self._cmpltLst = []
        self._actionSeq = ActionSeq()
        self._taskSeq = TaskSeq()

        # logging.basicConfig(level=logging.DEBUG)

    def decode(self, encode):
        self._encode = encode
        self._initState()
        self.decodeProcessor()
        raise Exception("debug")
        try:
            self.decodeProcessor()
        except Exception as e:
            # logging.error(e)
            print("some Error")
            return sys.float_info.max
        else:
            makespan = self._calMakespan()
            return makespan

    def decodeProcessor(self):
        self._allocatedLst = []
        # raise Exception("???")
        select_type = SelectType.AllTask
        select_taskID = INF_NUM

        select_times = 1
        while not self._calEndCondition():

            print("select_times = ", select_times)
            pairCandidate = self._selectRobTaskPair(select_type=select_type, select_taskID=select_taskID)
            print("select_times = ", select_times, " selectSuccess")
            if pairCandidate == RobTaskPair(robID=INF_NUM, taskID=INF_NUM):
                break
            select_type, select_taskID = self._updateSol(pairCandidate)
            print("actionSeq = ", self._actionSeq)
            print("actionTime = ", self._actionSeq.actionTime)
            if self._actionSeq.actionTime == sys.float_info.max:
                raise Exception("行动序列的时间不应该为无穷")
            print("select_times = ", select_times, " updateSuccess")

            select_times += 1

            # break
            # pass

        # for robID in  range(self.robNum):
        #     for taskID in range(self.taskNum):
        #         if self._cmpltLst[taskID] == True:
        #             continue
        #         if (robID,taskID) not in self._allocateLst:
        #             onRoadPeriod = self._calRob2TaskOnRoadPeriod(robID,taskID)
        #             if self._robotLst[robID].leaveTime + onRoadPeriod > self._taskLst[taskID].cmpltTime:
        #                 '''
        #                 preArriveTime > task.cmpltTime
        #                 continue
        #                 this part still need process
        #                 '''
        #                 continue
        #             # preOnRoadPeriodLst.

    def _selectRobTaskPair(self, select_type=SelectType.AllTask, select_taskID=INF_NUM):
        #  pre means predict
        #
        #
        #
        #

        if select_type == SelectType.AllTask:
            selectTaskLst = range(self.taskNum)
        elif select_type == SelectType.OneTask:
            selectTaskLst = [select_taskID]
        else:
            raise Exception("selectType Error")

        self._predictActionSeqType = dict()
        self._predictActionSeq = dict()
        self.taskVariableDic = dict()

        '''
        感觉可以优化 此部分
        '''
        self._predictOnTaskPeriod = dict()

        preOnRoadPeriodLst = []
        preOnTaskPeriodLst = []
        for robID in range(self.robNum):
            for taskID in selectTaskLst:
                if self._cmpltLst[taskID]:
                    continue
                # if RobTaskPair(robID,taskID) not in self._cmpltLst:
                rob = self._robotLst[robID]
                task = self._taskLst[taskID]
                rob_task_pair = RobTaskPair(robID, taskID)
                if rob_task_pair in self._allocatedLst:
                    continue
                onRoadPeriod = self._calRob2TaskOnRoadPeriod(robID, taskID)
                if onRoadPeriod + rob.leaveTime > task.cmpltTime:
                    continue
                '''
                移动机器人没必要去 已经完成的任务点
                '''

                if onRoadPeriod + rob.leaveTime < self._decodeTime:
                    continue
                '''
                the old method 
                '''
                # onTaskPeriod = self._calRobOnTaskPeriod(robID, taskID,preOnRoadPeriod = onRoadPeriod)
                '''
                the new method 
                '''
                onTaskPeriod = self._calActionSeq(robID, taskID, onRoadPeriod)

                preOnRoadPeriodLst.append((rob_task_pair, onRoadPeriod))
                preOnTaskPeriodLst.append((rob_task_pair, onTaskPeriod))

        if len(preOnRoadPeriodLst) == 0:
            return RobTaskPair(robID=INF_NUM, taskID=INF_NUM)

        onRoadPeriodDic = self._sortLst(preOnRoadPeriodLst, reverseBoolean=False)
        onTaskPeriodDic = self._sortLst(preOnTaskPeriodLst, reverseBoolean=False)

        # logging.debug(onRoadPeriodDic)
        # logging.debug(onTaskPeriodDic)

        syntheticalOrderLst = []
        for key in onRoadPeriodDic:
            onRoadOrder = onRoadPeriodDic[key]
            onTaskOrder = onTaskPeriodDic[key]
            rob = self._robotLst[key.robID]
            syntheticalOrder = rob.onRoadPeriodRatio * onRoadOrder + rob.onTaskPeriodRatio * onTaskOrder
            syntheticalOrderLst.append([RobTaskPair(key.robID, key.taskID), syntheticalOrder])

        minRobTaskPair, rankValue = min(syntheticalOrderLst, key=lambda x: x[1])

        print('minRobTaskPair= ', minRobTaskPair)
        # logging.debug(minRobTaskPair)
        # raise  Exception("sad")

        return minRobTaskPair
        # return RobTaskPair(robID=-1,taskID= -1)

    def _initState(self):
        self._actionSeq.clear()
        self._taskLst.clear()
        self._robotLst.clear()
        self._cmpltLst.clear()
        self._cmpltLst = [False] * self.taskNum

        for i in range(self.robNum):
            rob = RobotDe()
            rob.ability = self.robAbiLst[i]
            rob.vel = self.robVelLst[i]
            rob.encodeIndex = 0
            rob.stopBool = False
            rob.stateType = RobotState['onRoad']
            rob.leaveTime = 0
            rob.onRoadPeriodRatio = self._encode[3 * i]
            rob.onTaskPeriodRatio = self._encode[3 * i + 1]
            rob.makespanRatio = self._encode[3 * i + 2]
            self._robotLst.append(rob)

        for i in range(self.taskNum):
            task = Task()
            task.cState = self.taskStateLst[i]
            task.initState = self.taskStateLst[i]
            task.cRate = self.taskRateLst[i]
            task.initRate = self.taskRateLst[i]
            task.threhod = self.threhold
            task.cmpltTime = sys.float_info.max
            self._taskLst.append(task)
        self._decodeTime = 0

        self._initTaskLst = [copy.copy(x) for x in self._taskLst]
        self._initRobLst = [copy.copy(x) for x in self._robotLst]
        self._robActionLst = [[] for x in range(self._ins.robNum)]
        # self._discreteEncode  = -numpy.ones((ins.robNum,ins.taskNum),dtype=)
        # logging.debug("init success")

    def _calEndCondition(self):
        for rob in self._robotLst:
            if rob.stopBool == False:
                return False
        return True
        pass

    def _calMakespan(self):
        # makespan for the MPDA problem
        leaveTimeLst = [rob.leaveTime for rob in self._robotLst]
        makespan = max(leaveTimeLst)
        return makespan

    def _robActionSeqLst2DiscreteEncode(self):
        _discreteEncodeRes = -numpy.ones((self.robNum, self.taskNum), dtype=int)
        for robID, robActionSeq_ in enumerate(self._robActionLst):
            for robEncodeInd, robAction_ in enumerate(robActionSeq_):
                _discreteEncodeRes[robID][robEncodeInd] = robAction_
        return _discreteEncodeRes

    def _calRob2TaskOnRoadPeriod(self, robID, taskID):
        rob = self._robotLst[robID]
        if rob.encodeIndex == 0:
            dis = self.rob2taskDisMat[robID][taskID]
            dur = dis / self.robVelLst[robID]
        else:
            currentTaskID = rob.taskID
            dis = self.taskDisMat[currentTaskID][taskID]
            dur = dis / self.robVelLst[robID]
        return dur

    # def _calRobOnTaskPeriod(self, robID, taskID, preOnRoadPeriod = 0):
    #     rob = self._robotLst[robID]
    #     robAbi = rob.ability
    #     task = self._taskLst[taskID]
    #     calTask = copy.deepcopy(self._taskLst[taskID])
    #     if calTask.changeRateTime > rob.leaveTime + preOnRoadPeriod:
    #         pass
    #
    #         # '''
    #         # this part can be optimized.
    #         # '''
    #         # calTask.recover(*self._taskInitInfo[taskID])
    #         # taskInfo = copy.deepcopy(self._taskInfoLst[taskID])
    #         # taskInfo.append(
    #         #     TaskInfoClass(robID=robID, changeRateTime=(rob.leaveTime + onRoadPeriod), cRate=10, robAbi=rob.ability))
    #         # taskInfo = sorted(taskInfo, key=lambda x: x.changeRateTime)
    #         # '''
    #         # _taskInforLst is sorted by the arriveTime
    #         # '''
    #         # delIndexLst = []
    #         # for i in range(len(taskInfo)):
    #         #     taskInfoUnit = taskInfo[i]
    #         #     if calTask.cmpltTime < taskInfoUnit.changeRateTime:
    #         #         delIndexLst.append(taskInfoUnit)
    #         #         continue
    #         #     calTask.calCurrentState(taskInfoUnit.changeRateTime)
    #         #     calTask.cRate = calTask.cRate - taskInfoUnit.robAbi
    #         #     if calTask.cRate >= 0:
    #         #         executePeriod = INF_NUM
    #         #         calTask.cmpltTime = INF_NUM
    #         #     else:
    #         #         executePeriod = calTask.calExecuteDur()
    #         #         calTask.cmpltTime = taskInfoUnit.changeRateTime + executePeriod
    #         # executePeriod = calTask.cmpltTime - rob.leaveTime - onRoadPeriod
    #         # resTuple = OnTaskInfoClass(vaild=True, time=executePeriod, rate=calTask.cRate)
    #         # if len(delIndexLst):
    #         #     self._delDict[(robID, taskID)] = delIndexLst
    #     #                print(self._delDict)
    #     else:
    #         vaild = calTask.calCurrentState(rob.leaveTime + preOnRoadPeriod)
    #         cRate = sys.float_info.max
    #         executePeriod = sys.float_info.max
    #         if vaild == True:
    #             calTask.cRate = calTask.cRate - robAbi
    #             if calTask.cRate >= 0:
    #                 executePeriod = sys.float_info.max
    #             else:
    #                 executePeriod = calTask.calExecuteDur()
    #             cRate = calTask.cRate
    #         resTuple = OnTaskInfoTuple(changeRateTime=calTask.changeRateTime, vaild = vaild,
    #                                    executePeriod = executePeriod, cRate = cRate)
    #
    #     self.taskVariableDic[(robID, taskID)] = calTask.variableInfo()
    #     return resTuple
    #     '''
    #     此处还没有调试清楚
    #     '''
    #     return random.random()

    def _calCurrentMakespan(self, robID, taskID):
        rob = self._robotLst[robID]
        task = self._taskLst[taskID]
        return random.random()

    def _calRobTaskEventTime(self, robID, taskID):
        rob = self._robotLst[robID]
        task = self._taskLst[taskID]
        self._calRob2TaskOnRoadPeriod(robID, taskID)
        pass

    def _sortLst(self, lst=[], keyFunc=lambda x: x[1], reverseBoolean=False):
        lst = sorted(lst, key=keyFunc, reverse=reverseBoolean)
        val = sys.float_info.min
        # orderInd = -1
        orderInd: int
        dic = dict()
        # index: int
        for index, unit in enumerate(lst):
            if val == unit[1]:
                dic[unit[0]] = orderInd
            else:
                val = unit[1]
                orderInd = index
                dic[unit[0]] = orderInd
        return dic

    def _calActionSeq(self, robID, taskID, preOnRoadPeriod):
        rob = self._robotLst[robID]
        robAbi = rob.ability
        preDictActTime = rob.leaveTime + preOnRoadPeriod
        if preDictActTime >= self._actionSeq.actionTime:
            calTask = copy.deepcopy(self._taskLst[taskID])
            vaild = calTask.calCurrentState(rob.leaveTime + preOnRoadPeriod)
            cRate = sys.float_info.max
            executePeriod = sys.float_info.max
            if vaild == True:
                calTask.cRate = calTask.cRate - robAbi
                if calTask.cRate >= 0:
                    executePeriod = sys.float_info.max
                else:
                    executePeriod = calTask.calExecuteDur()
                cRate = calTask.cRate
            '''
            bug??
            '''
            # print('executePeriod = ', executePeriod)
            cmplt_time = preDictActTime + executePeriod
            # raise Exception("dsadsa")
            action_tuple_lst = []
            action_tuple_lst.append(
                ActionTuple(robID=robID, taskID=taskID, eventType=EventType.arrive, eventTime=preDictActTime))
            if executePeriod != INF_NUM:
                action_tuple_lst.append(
                    ActionTuple(robID=robID, taskID=taskID, eventType=EventType.leave, eventTime=cmplt_time))

            self._predictActionSeq[RobTaskPair(robID=robID, taskID=taskID)] = action_tuple_lst
            self._predictActionSeqType[RobTaskPair(robID=robID, taskID=taskID)] = ActionSeqType.InOrder

            self._predictOnTaskPeriod[RobTaskPair(robID=robID, taskID=taskID)] = executePeriod

            self.taskVariableDic[RobTaskPair(robID=robID, taskID=taskID)] = calTask.variableInfo()

            return executePeriod

            # ActionSeq =
            '''
            deepcopy 存在问题
            '''
            pass
        else:
            print("preDictActTime = ", preDictActTime)
            print("self._actionSeq.actionTime = ", self._actionSeq.actionTime)
            # test
            print(self._actionSeq)
            invalidEventNum = self._actionSeq.invalidEventTilTime(preDictActTime)

            print("invalidEventNum = ", invalidEventNum)
            action_seq = copy.deepcopy(self._actionSeq)

            action_seq.delActionEvent(invalidEventNum)

            action_seq.append(
                ActionTuple(robID=robID, taskID=taskID, eventType=EventType.arrive, eventTime=preDictActTime))
            print(action_seq)
            print(self._actionSeq)
            # raise  Exception()
            # action_seq = copy.copy(self._actionSeq)
            self._calActionSeqStatus(action_seq)
            raise Exception("nothingf")

    def _updateSol(self, rob_task_pair=RobTaskPair(robID=-1, taskID=-1)):
        robID = rob_task_pair.robID
        taskID = rob_task_pair.taskID
        predictTaskCmpltTime = INF_NUM
        rob = self._robotLst[robID]
        task = self._taskLst[taskID]
        if self._predictActionSeqType[rob_task_pair] == ActionSeqType.InOrder:

            self._actionSeq.extend(self._predictActionSeq[rob_task_pair])
            if len(self._predictActionSeq[rob_task_pair]) == 1:
                self._actionSeq.infEventAppend(rob_task_pair)
            else:
                self._actionSeq.eventComplement()
            rob.taskID = taskID
            rob.encodeIndex += 1
            onRoadPeriod = self._calRob2TaskOnRoadPeriod(robID, taskID)
            onTaskPeriod = self._predictOnTaskPeriod[rob_task_pair]
            self._allocatedLst.append(rob_task_pair)
            rob.arriveTime = rob.leaveTime + onRoadPeriod
            print(self.taskVariableDic[rob_task_pair])
            task.recover(*self.taskVariableDic[rob_task_pair])
            print(task)
            self._robActionLst[robID].append(taskID)
            # self._actionSeq.
            # raise  Exception("在这里终结")
            # self.taskVariableDic
            # self.taskVariableDic
        else:
            pass
            raise Exception('dashdkj')

        predictTaskCmpltTime = task.cmpltTime
        '''
        '''
        if predictTaskCmpltTime == INF_NUM:
            return SelectType.OneTask, taskID
            pass
        else:
            return SelectType.AllTask, INF_NUM

    def _calActionSeqStatus(self, action_seq=ActionSeq()):

        # taskLst  = copy.copy(self._initTaskLst)
        # robLst = copy.copy(self._initRobLst)
        taskLst = [copy.copy(x) for x in self._initTaskLst]
        robLst = [copy.copy(x) for x in self._initRobLst]

        for task in taskLst:
            print(task)

        task_seq = TaskSeq()

        '''
        debug 版本的计算模式
        '''
        for action_tuple in action_seq.seq:
            task = taskLst[action_tuple.taskID]
            print('taskID = ', action_tuple.taskID)
            if action_tuple.eventType == EventType.arrive:
                rob = robLst[action_tuple.robID]
                '''
                debug yong
                '''
                b_rate = task.cRate
                task.calRobArrive(action_tuple.eventTime, rob.ability)
                task_status_tuple = TaskStatusTuple(taskID=action_tuple.taskID, cState=task.cState,
                                                    cRate=task.cRate, bRate=b_rate, time=action_tuple.eventTime)
                task_seq.append(task_status_tuple)
                print("cmpltTime= ", task.cmpltTime)
            elif action_tuple.eventType == EventType.leave:
                # pass
                if task.cmpltTime == action_tuple.eventTime:
                    pass
                    task.calRobArrive(action_tuple.eventTime, 0)

                    task_status_tuple = TaskStatusTuple(taskID=action_tuple.taskID, cState=task.cState,
                                                        cRate=InvalidType.Rate, bRate=task.cRate,
                                                        time=action_tuple.eventTime)
                    task_seq.append(task_status_tuple)

                else:
                    print("taskcmpltTime = ", task.cmpltTime)
                    print("actionTupleTime = ", action_tuple.eventTime)

                    raise Exception("sadjsakhdjksahdkj")
                    pass
            else:
                raise Exception("why 不应该哈")
                # action_tuple.

        task_seq.drawPlot(BaseDir + '\\plot\\actionSeq')
        f_deg = open(BaseDir + '\\debugData\\task_seq.txt', 'w')
        f_deg.write(str(task_seq))
        f_deg.write(str(action_seq))
        f_deg.flush()
        f_deg.close()
        raise Exception("sadsal")
        pass


if __name__ == '__main__':
    insName = '14_14_ECCENTRIC_RANDOMCLUSTERED_SVLCV_LVLCV_thre0.1MPDAins.dat'
    ins = Instance(BaseDir + '\\benchmark\\' + insName)
    MPDA_DE_decode._ins = ins
    TaskSeq._ins = ins

    mpda_de_decode = MPDA_DE_decode()

    encode = [rd.random() for x in range(ins.robNum * 3)]

    # task_seq = TaskSeq()
    # task_seq.drawPlot()
    #
    # raise Exception("test_")

    # print(encode)
    # logging.info("encode = " + str(encode))
    mpda_de_decode.decode(encode)

    # result = differential_evolution(ackley, bounds, callback = callbackF,args = (3,3), disp = True)
