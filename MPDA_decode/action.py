

import  sys
from  enum import Enum
from MPDA_decode.instance import Instance,TaskModelType
import numpy
import math

# import plotly.plotly as py
# import plotly.graph_objs as go
# import plotly
# import numpy as np
# import plotly.io as pio


class EventType(Enum):
    arrive = 1
    leave = 2



class InvalidType(Enum):
    Rate =  1

class SolutionDe(object):
    _taskNum = 0
    _robNum = 0
    def __init__(self):
        self.chrom = [0] * SolutionDe._robNum

from collections import namedtuple


ActionTuple = namedtuple('ActionTuple',['robID','taskID','eventType','eventTime'])
RobTaskPair = namedtuple('TaskRobPair',['robID','taskID'])


class ActionSeq(object):

    _modelType = {TaskModelType.ExpModel: '_expCal',
                 TaskModelType.LineModel: '_lineCal'}
    def __init__(self):
        self._seq = []
        self._actionTime = 0
        self._infEvent = []

    @property
    def seq(self):
        return self._seq

    @seq.setter
    def seq(self,_o_seq):
        if type(_o_seq) != type([]):
            raise TypeError('tuple must be LIST')
        else:
            self._seq = _o_seq
    @seq.deleter
    def seq(self):
        del self._seq


    def __eq__(self, other):
        return other._seq == self._seq

    def append(self,action_tuple):
        if type(action_tuple) != ActionTuple:
            raise TypeError('tuple must be ActionTuple')
        elif action_tuple.eventTime < self.actionTime:
            print(self)
            self._seq.append(action_tuple)
            print(self)
            raise Exception('time is out of order in action seq ')
        else:
            self._seq.append(action_tuple)

    def extend(self,action_tuple_lst = []):
        for action_tuple in action_tuple_lst:
            self.append(action_tuple)

    def __getitem__(self, item):
        return self._seq[item]

    def __setitem__(self, key, value):
        self._seq[key] = value

    def __str__(self):
        res_str = str()
        for x in self._seq:
            res_str = res_str + str(x) +'\n'
        return res_str
    def clear(self):
        self._seq.clear()
        self._actionTime = 0


    @property
    def actionTime(self):
        if len(self._seq) == 0:
            self._actionTime = 0
        else:
            ind = -1
            self._actionTime = sys.float_info.max
            while self._actionTime == sys.float_info.max:
                self._actionTime = self._seq[ind].eventTime
                ind -= 1
        return self._actionTime

    @actionTime.setter
    def actionTime(self,action_time):
        self._actionTime = action_time

    @property
    def infEvent(self):
        return self._infEvent

    @infEvent.setter
    def infEvent(self,_o_infEvent):
        if type(_o_infEvent) != type([]):
            raise TypeError('tuple must be LIST')
        else:
            self._infEvent = _o_infEvent

    @infEvent.deleter
    def infEvent(self):
        del self.infEvent

    def infEventAppend(self,rob_task_pair = RobTaskPair(-1,-1)):
        if type(rob_task_pair) != RobTaskPair:
            raise Exception("infEventAppend Type Error")
        else:
            self._infEvent.append(rob_task_pair)

    def infEventClear(self):
        self._infEvent.clear()

    def eventComplement(self):
        if len(self._infEvent) != 0:
            # raise  Exception("开始补全")
            lastActionTuple = self._seq[-1]
            for rob_task_pair in self._infEvent:
                if rob_task_pair.taskID == lastActionTuple.taskID:
                    self._seq.append(ActionTuple(robID=rob_task_pair.robID,taskID=rob_task_pair.taskID,
                                                 eventType= EventType.leave,eventTime=lastActionTuple.eventTime))
                else:
                    raise Exception("补全事件的时候存在问题")
        self._infEvent.clear()

    def invalidEventTilTime(self,event_time):
        invalidEventNum = 0
        # d = list(range())
        for i in range(len(self._seq)- 1,-1,-1):
            action_time = self._seq[i].eventTime
            if event_time > action_time or math.isclose(event_time,action_time):
                break
            invalidEventNum += 1
        return  invalidEventNum

    def delActionEvent(self,invalidEventNum):
        while invalidEventNum > 0:
            self._seq.pop()
            invalidEventNum -= 1


    def examinationSelf(self):
        if len(self._seq):
            print(len(self._seq))
            lst = []
            action_ = self._seq[0]
            lst.append(RobTaskPair(action_.robID,action_.taskID))
            for actionID in range(1,len(self._seq)):

                if action_.eventTime < self._seq[actionID].eventTime or math.isclose(action_.eventTime,self._seq[actionID].eventTime):
                    action_  = self._seq[actionID]
                else:
                    print("action_.eventTime = ",action_.eventTime)
                    print("self._seq[actionID] = ",self._seq[actionID].eventTime)
                    raise Exception("examination  _bug")
                if action_.eventType == EventType.arrive:
                    lst.append(RobTaskPair(action_.robID,action_.taskID))
                else:
                    # print("robTaskPair =",RobTaskPair(action_.robID, action_.taskID))
                    # print("Lst = ",lst)
                    lst.remove(RobTaskPair(action_.robID, action_.taskID))

            if len(lst):
                print(lst)
                raise Exception('lst problem')
                pass
            else:
                pass
        else:
            raise  Exception("examination _bug ")
        return True


    def actionSeq2DiscreteEncode(self,robNum = -1,taskNum = -1):
        encodeRes =  -numpy.ones((robNum,taskNum),dtype = int)
        robEncodeIndLst = [0 for x in range(robNum)]
        for action_ in self._seq:
            if action_.eventType == EventType.arrive:
                encodeRes[action_.robID][robEncodeIndLst[action_.robID]] = action_.taskID
                robEncodeIndLst[action_.robID] += 1
        return encodeRes

    def actionSeq2RobActionLst(self,robNum = -1):
        robActionLstRes = [[] for x in range(robNum)]
        for action_ in self._seq:
            if action_.eventType == EventType.arrive:
                robActionLstRes[action_.robID] = action_.taskID
        return robActionLstRes









'''
任务的状态变化序列来完整的表示一个解
'''

'''
bRate 表示之前的rate值
'''

TaskStatusTuple = namedtuple('TaskStatusTuple',['taskID','cState','cRate','bRate','time'])

class TaskSeq(object):
    # _taskNum = sys.int_info.max
    _ins = object
    _modelType = {TaskModelType.ExpModel: '_expCal',
                 TaskModelType.LineModel: '_lineCal'}

    def __init__(self):
        if type(TaskSeq._ins) != Instance:
            raise TypeError("MPDA_DE_decode._ins must be the Instance class")

        ins = TaskSeq._ins
        if ins.taskModelType in self._modelType:
            self.calStateFunc = getattr(self,self._modelType[ins.taskModelType])
        else:
            raise Exception("no task model Type")

        self._taskSeq = []

        for taskID in range(ins.taskNum):
            task_status_tuple = TaskStatusTuple(taskID = taskID, cState= ins.taskStateLst[taskID],
                                                cRate= ins.taskRateLst[taskID],bRate = 0, time = 0)
            self._taskSeq.append(task_status_tuple)

    @property
    def taskSeq(self):
        return self._taskSeq

    @taskSeq.setter
    def taskSeq(self,_o_taskSeq):
        if type(_o_taskSeq) != type([]):
            raise TypeError('tuple must be LIST')
        else:
            self._taskSeq = _o_taskSeq

    @taskSeq.deleter
    def taskSeq(self):
        del self._taskSeq


    @property
    def actionTime(self):
        if len(self._taskSeq) == 0:
            self._actionTime = 0
        else:
            ind = -1
            self._actionTime = sys.float_info.max
            while self._actionTime == sys.float_info.max:
                self._actionTime = self._taskSeq[ind].time
                ind -= 1
        return self._actionTime

    @actionTime.setter
    def actionTime(self,action_time):
        self._actionTime = action_time




    def append(self,task_tuple):
        if type(task_tuple) != TaskStatusTuple:
            raise TypeError('tuple must be TaskStatusTuple')
        elif task_tuple.time < self.actionTime:
            raise Exception('time is out of order in action seq ')
        else:
            self._taskSeq.append(task_tuple)

    def __eq__(self, other):
        return other._taskSeq == self._taskSeq

    def __str__(self):
        res_str = str()
        for x in self._taskSeq:
            res_str = res_str + str(x) +'\n'
        return res_str

    def drawPlot(self, plotName = 'nothing'):

        taskTimeLst = [[] for x in range(TaskSeq._ins.taskNum)]
        taskStateLst = [[] for x in range(TaskSeq._ins.taskNum)]
        taskStateTupleLst  = [[] for x in range(TaskSeq._ins.taskNum)]


        for task_tuple in self._taskSeq:
            taskStateTupleLst[task_tuple.taskID].append(task_tuple)

        for taskID,task_lst in enumerate(taskStateTupleLst):
            if len(task_lst) == 0:
                continue
            taskTimeLst[taskID].append(task_lst[0].time)
            taskStateLst[taskID].append(task_lst[0].cState)
            for i in range(len(task_lst) - 1):
                if task_lst[i].cRate == InvalidType.Rate:
                    continue
                time_lst,state_lst = self._discretePoint(task_lst[i],task_lst[i+1])
                taskTimeLst[taskID].extend(time_lst)
                taskStateLst[taskID].extend(state_lst)
            # if len()

        # print(taskTimeLst)
        # print(taskStateLst)
        figData = []
        for taskID in range(TaskSeq._ins.taskNum):
            trace = go.Scatter(mode = 'lines', x = taskTimeLst[taskID], y = taskStateLst[taskID])
            figData.append(trace)

        layout = dict()
        layout['xaxis'] = dict( title = 'time')
        layout['yaxis'] = dict(title = 'state')
        fig = go.Figure(data=figData, layout=layout)


        # pio.write_image(fig,'nothing')

        plotly.offline.plot(fig,filename=plotName)


        # for taskID in range(TaskSeq._ins.taskNum):


    def _discretePoint(self,task_state_tuple_begin: TaskStatusTuple, task_state_tuple_end : TaskStatusTuple):
        timeArray = numpy.linspace(task_state_tuple_begin.time,task_state_tuple_end.time)
        timeLst = timeArray.tolist()
        begin_state = task_state_tuple_begin.cState
        begin_rate = task_state_tuple_begin.cRate
        begin_time = task_state_tuple_begin.time

        stateLst  = [self.calStateFunc(cState = begin_state,
                                             cRate = begin_rate,
                                             dur = (x - begin_time)) for x in timeLst]
        return timeLst,stateLst


    def _expCal(self,cState:float,cRate:float,dur:float):
        return cState*numpy.exp(cRate*dur)
    def _lineCal(self,cState:float,cRate:float,dur:float):
        return  cState + cRate * dur




if __name__ == '__main__':
    wtf = ActionTuple(1,2,EventType['arrive'],100)
    wtf2 = ActionTuple(1,2,EventType['arrive'],102)
    seq1 = ActionSeq()
    seq1.append(wtf)
    seq1.append(wtf2)

    seq2 = ActionSeq()
    seq2.append(wtf2)
    seq2.append(wtf)

    seq2[0] = wtf
    seq2[1] = wtf2

    print(seq1)

    if seq1 == seq2:
        print("相等")
    else:
        print("不相等")

    SolutionDe._taskNum = 2
    SolutionDe._robNum = 4
    sol_1  = SolutionDe()
    print(len(sol_1.chrom))
    SolutionDe._robNum = 5
    so_2 = SolutionDe()
    print(len(so_2.chrom))



