import random
import math
import numpy as np
from mpdaInstance import  MPDAInstance
from mpdaDecodeMethod.mpdaDecode import MPDADecoder
# from mpdaTaskACO.mpdaTaskUpdater import ACO_Updater



from deap import algorithms
from deap import base
from deap import benchmarks
from deap import creator
from deap import tools
import numpy
import random
import time
from numpy.random import choice as np_choice
import copy

import os
import sys


AbsolutePath = os.path.abspath(__file__)
SuperiorCatalogue = os.path.dirname(AbsolutePath)
BaseDir = os.path.dirname(SuperiorCatalogue)

import mpdaACO.mpdaACOInit as _init
from mpdaACO.mpdaConstructSol import ACO_Constructor

debugBool = False

import mpdaACO.mpdaConstructSol
mpdaACO.mpdaConstructSol.debugBool = debugBool


def permutationSinglePointSwap(perm):
    unit1,unit2 = random.sample(perm, 2)
    index1 = perm.index(unit1)
    index2 = perm.index(unit2)
    perm[index1] = unit2
    perm[index2] = unit1
    return perm

def permutationSinglePointSwapACO(perm,acoSize):
    index1 = random.randint(0,acoSize - 1)
    unit1 = perm[index1]
    index2 = random.randint(0,len(perm)-1)
    unit2 = perm[index2]
    perm[index1] = unit2
    perm[index2] = unit1
    return perm


class MPDA_Task_ACO(object):
    def __init__(self, ins: MPDAInstance,benchmarkName,
                 decay = 1,maxRunTime = 100, sampleRate = 20,
                 rdSeed = 1, heuristicRule = '_None' , eliteRule = '_Gbt',
                 localSearchMechanism = '_None',
                 localSearchIndNum = 1,
                    localSearchRowNum = 3,
                 updatingMechanism = '_Trad',
                 selectedMechanism = '_Trad',
                 fixMechanism = '_Trad'):
        self._ins = ins
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

        '''
        xxx
        '''
        self.rdSeed = rdSeed
        random.seed(rdSeed)
        np.random.seed(rdSeed)
        self.benchmarkName = benchmarkName
        self._algName = 'aco_'

        creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
        creator.create("Individual",list,
                       fitness = creator.FitnessMin, actionSeq = object)

        self.toolbox = base.Toolbox()
        self.toolbox.register("mpda_attr", _init.mpda_init_aco, self._robNum)

        self.toolbox.register("individual", tools.initIterate, creator.Individual,
                         self.toolbox.mpda_attr)

        # define the population to be a list of individuals
        self.toolbox.register("population", tools.initRepeat, list, self.toolbox.individual)

        self.n_ants = self._taskNum * self._robNum

        self.n_best = math.floor(self.n_ants * sampleRate/100 )
        self.n_iterations = 100
        self.decay = decay /100
        self.alpha = 1
        self.beta = 1
        self._maxRunTime = maxRunTime

        calTime = max(self._robNum,self._taskNum) * self._robNum * self._taskNum * 0.1
        calTime = min(29990,calTime)
        print('cal Time = ',calTime)
        maxCalTime = calTime
        self._maxRunTime = maxCalTime

        self._robAbiSum = sum(self._robAbiLst)

        self._heuristicRule = heuristicRule
        self._algName += self._heuristicRule
        self._algName = self._algName + '_SR' + str(int(sampleRate ))
        self._algName = self._algName + '_DR' + str(int(decay))
        self._algName = self._algName + eliteRule
        self._algName = self._algName + updatingMechanism
        self._algName = self._algName + selectedMechanism
        self._algName = self._algName + localSearchMechanism
        self._heuristicRule = heuristicRule
        self._updatingMechanism = updatingMechanism
        self._selectedMechanism = selectedMechanism

        if eliteRule == '_None':
            self.eliteRule = eliteRule
        elif eliteRule == '_Gbt':
            self.eliteRule = eliteRule
        else:
            raise Exception('elite rule is not in the list')

        if updatingMechanism == '_Trad':
            self.updatingMechanism = updatingMechanism
        elif updatingMechanism == '_Prob':
            self.updatingMechanism = updatingMechanism
        elif updatingMechanism == '_Eset':
            self.updatingMechanism = updatingMechanism
        elif updatingMechanism == '_ACS':
            self.updatingMechanism = updatingMechanism
            self.n_ants = 10
            self.decay = 0.9
        else:
            raise Exception('updating mechanism is not in this list')

        if fixMechanism == '_None':
            self.fixMechanism = fixMechanism
            self._algName = self._algName + '_F' + fixMechanism
        elif fixMechanism == '_Trad':
            self.fixMechanism = fixMechanism
            self._algName = self._algName + '_F' + fixMechanism
        else:
            raise Exception('fix mechanism is not in this list')

        if localSearchMechanism == '_None':
            self.localSearchMechanism = localSearchMechanism
            self._algName = self._algName + '_IN' + str(localSearchIndNum)
            self._algName = self._algName + '_RN' + str(localSearchRowNum)
        elif localSearchMechanism == '_Trad':
            self.localSearchMechanism = localSearchMechanism
            self._algName = self._algName + '_IN' + str(localSearchIndNum)
            self._algName = self._algName + '_RN' + str(localSearchRowNum)
            self.localSearchIndNum = localSearchIndNum
            self.localSearchRowNum = localSearchRowNum
        elif localSearchMechanism == '_EqTrad':
            self.localSearchMechanism = localSearchMechanism
            self._algName = self._algName + '_IN' + str(localSearchIndNum)
            self._algName = self._algName + '_RN' + str(localSearchRowNum)
            self.localSearchIndNum = localSearchIndNum
            self.localSearchRowNum = localSearchRowNum
        else:
            raise Exception('local search is not in this list')


    def run(self):
        randomSeed = self.rdSeed
        floderName = BaseDir + '//debugData//' + str(self.benchmarkName) + '//' + str(self._algName)
        folder = os.path.exists(BaseDir + '//debugData//' + str(self.benchmarkName) + '//' + str(self._algName))
        if not folder:
            os.makedirs(BaseDir + '//debugData//' + str(self.benchmarkName) + '//' + str(self._algName))
        f_con = open(
            BaseDir + '//debugData//' + str(self.benchmarkName) + '//' + str(self._algName) + '//' + 'r_' + str(
                randomSeed) + '.dat', 'w')
        save_data = BaseDir + '//debugData//' + str(self.benchmarkName) + '//' + str(self._algName) + '//' + 'r_' + str(
            randomSeed) + '.dat'
        print(save_data)
        if debugBool:
            f_con_deg = open(BaseDir + '//debugData//' + str(self.benchmarkName) + '//' + str(self._algName) + '//' + 'r_' + str(
                randomSeed) + 'debug.dat', 'w')
        self.rob_sum_Abi = np.sum(self._robAbiLst)
        print('rob_sum_abi = ', self.rob_sum_Abi)
        self.task_sum_Abi = np.sum(self._taskRateLst)
        print('task_sum_rate = ', self.task_sum_Abi)
        print('phi = ', self.task_sum_Abi/ self.rob_sum_Abi)
        min_h_fit = _init.init_pheromone(self._ins,self.benchmarkName)

        NFE = 0
        # exit()

        if self.updatingMechanism == '_ACS':
            self.taskPheromoneLst = [np.ones([self._taskNum, self._taskNum]) / (min_h_fit * self._taskNum) for x in range(self._robNum)]
            self.robTaskPheromoneLst = [np.ones([self._taskNum]) / (min_h_fit * self._taskNum) for x in range(self._robNum)]
            self._min_h_fit = min_h_fit
        else:
            self.taskPheromoneLst =  [np.ones([self._taskNum,self._taskNum])/( min_h_fit) for x in range(self._robNum)]
            self.robTaskPheromoneLst = [np.ones([self._taskNum])/( min_h_fit) for x in range(self._robNum)]
        self._maxEventNum = self._taskNum * self._robNum
        if debugBool:
            f_con_deg.write(str(self.robTaskPheromoneLst[0])+ '\n')
            f_con_deg.flush()

        ngen = 7000
        NP = self.n_ants

        hof = tools.HallOfFame(1)
        stats = tools.Statistics(lambda ind: ind.fitness.values)
        stats.register("avg", numpy.mean)
        stats.register("std", numpy.std)
        stats.register("min", numpy.min)
        stats.register("max", numpy.max)
        logbook = tools.Logbook()
        logbook.header = "gen", "min", "std", "avg", "max"
        self.hof = tools.HallOfFame(1)
        # self.eliteHof = tools.HallOfFame(self.n_ants)
        MAXNFE = self._taskNum * self._robNum *7000
        start = time.time()

        self.pop = self.toolbox.population(n = NP)

        constructor = ACO_Constructor(self._ins, self.alpha, self.beta,
                                      self.taskPheromoneLst, self.robTaskPheromoneLst,
                                      self._selectedMechanism, self._heuristicRule,self.fixMechanism,
                                      self.updatingMechanism, self.rob_sum_Abi, self.task_sum_Abi )
        for gen in range(ngen):
            allAntSol = []
            for i in range(self.n_ants):
                unit = constructor.construct()
                allAntSol.append(unit)
                mpdaACO.mpdaConstructSol.debugBool = False
                if self.updatingMechanism == '_ACS':
                    self.localUpdatingPheronome_acs(unit[0])

            '''Statistics results'''
            for i,unit in enumerate(allAntSol):
                self.pop[i].clear()
                self.pop[i].extend(unit[0])
                self.pop[i].fitness.values = (unit[1],)
                self.pop[i].actionSeq = unit[2]

            self.hof.update(self.pop)
            # self.eliteHof.update(self.pop)

            # for ind in self.eliteHof:
            #     print(ind.fitness.values[0])
            record = stats.compile(self.pop)
            logbook.record(gen=gen, **record)
            print(logbook.stream)
            runTime = time.time() - start
            NFE = NFE + self.n_ants
            self.writeDir(f_con, record, gen,
                          NFE = NFE,
                          runTime = runTime, hofFitness= self.hof[0].fitness.values[0])
            if runTime > self._maxRunTime:
                break
            if NFE > MAXNFE:
                break


            if self.localSearchMechanism == '_Trad':
                '''
                the elite local search method 
                '''
                NFE += self._taskNum * self.localSearchIndNum
                localBase = copy.deepcopy(self.hof[0])
                localBase = []
                localBaseSize = []
                # for robID in range
                for robID in range(self._robNum):
                    localBase.append(copy.deepcopy(self.hof[0][robID]))
                    localBaseSize.append(len(self.hof[0][robID]))
                    seq = [x for x in range(self._taskNum)]
                    for unit in localBase[robID]:
                        seq.remove(unit)
                    random.shuffle(seq)
                    localBase[robID].extend(seq)
                decoder = MPDADecoder(self._ins)
                indLst = []
                for x in range(self._taskNum * self.localSearchIndNum):
                    ind = copy.deepcopy(localBase)
                    # decoder.decode(ind)
                    r_step = random.randint(1, self.localSearchRowNum)
                    if r_step > self._robNum:
                        r_step = self._robNum - 1
                    r_stepLst = random.sample(list(range(self._robNum)), r_step)
                    for rdRobID in r_stepLst:
                        # print(ind[rdRobID])
                        perm = permutationSinglePointSwapACO(ind[rdRobID],localBaseSize[rdRobID])
                        ind[rdRobID] = perm
                        # print(ind[rdRobID])
                    # indLst.append(ind)
                    pb,_actSeq = decoder.decode(ind)
                    fitness = decoder.calMakespan()
                    indLst.append((_actSeq,fitness))
                # minlInd = min(lIndLst, key=lambda x: x.fitness.values[0])
                minInd = min(indLst,key= lambda  x: x[1])
                if minInd[1] < self.hof[0].fitness.values[0]:
                    perm = minInd[0].convert2MultiPerm(self._ins._robNum)
                    for robID in range(self._robNum):
                        self.hof[0][robID] = perm[robID]
                    self.hof[0].fitness.values = (minInd[1],)
            if self.localSearchMechanism == '_EqTrad':
                for unit in allAntSol:
                    if random.random() < 0.2:
                        NFE += self._taskNum * self.localSearchIndNum
                        lsMethodBool,lsInd,lsFitness = self.localSearchBaseInd(unit[0],unit[1])
                        if lsMethodBool == True:
                            unit[0] = lsInd
                            unit[1] = lsFitness
            if self._updatingMechanism == '_ACS':
                self.globalUpdatingPheronome_acs()
            else:
                for robID in range(self._robNum):
                    self.taskPheromoneLst[robID] = self.taskPheromoneLst[robID] * self.decay
                    self.robTaskPheromoneLst[robID] = self.robTaskPheromoneLst[robID] * self.decay
                self.spread_pheronome(allAntSol,self.n_best)
            if self._selectedMechanism == '_Limit':
                self.limitedNumMechanism()


            if debugBool:
                f_con_deg.write(str(gen) + '  ' +str(self.robTaskPheromoneLst[0]) + '\n')
                f_con_deg.flush()
        end = time.time()
        runTime = end - start
        print('runTime', runTime)
        print('hof = ', self.hof[0])
        print('hofFitness = ', self.hof[0].fitness.values[0])
        f_con.write(str(self.hof[0]) + '\n')
        f_con.write('min  ' + str(self.hof[0].fitness.values[0]) + '\n')
        encode = np.ones([self._robNum, self._taskNum], dtype=int) * -1
        robEncodeInd = [0 for x in range(self._robNum)]
        for robID, seq in enumerate(self.hof[0]):
            for pos, taskID in enumerate(seq):
                encode[robID][pos] = taskID
        gaHof = []
        for unit in encode:
            gaHof.extend(unit)
        f_con.write('gaMin ' + str(gaHof) + '\n')
        print('gaMin = ', gaHof)
        f_con.write('runTime ' + str(runTime) + '\n')
        # if self.selectedMechanism == '_Limit':
        #     print('limited = ', self.limitedNum)
        f_con.close()


    def spread_pheronome(self, all_ant_sols, n_best):
        if self.updatingMechanism == '_Eset':
            for ind in self.eliteHof:
                for robID in range(self._robNum):
                    seq = ind[robID]
                    self.robTaskPheromoneLst[robID][seq[0]] +=  1 / (ind.fitness.values[0] * self._robNum *self._taskNum)
                    for i in range(len(seq) - 1):
                        self.taskPheromoneLst[robID][seq[i]][seq[i+1]] = 1 / (ind.fitness.values[0] * self._robNum *self._taskNum)
                # print(ind)
            return
        # exit()
        sorted_ant_sols = sorted(all_ant_sols, key=lambda x: x[1])
        for ant_sol, fitness,x in sorted_ant_sols[:n_best]:
            for robID in range(self._robNum):
                seq = ant_sol [robID]
                if self.updatingMechanism == '_Trad':
                    self.robTaskPheromoneLst[robID][seq[0]] +=  1 / (fitness )
                if self.updatingMechanism == '_Prob':
                    self.robTaskPheromoneLst[robID][seq[0]] +=  1 / (fitness * self._robNum *self._taskNum)
                for i in range(len(seq) - 1):
                    if self.updatingMechanism == '_Trad':
                        self.taskPheromoneLst[robID][seq[i]][seq[i + 1]] += 1 / (fitness )
                    if self.updatingMechanism == '_Prob':
                        self.taskPheromoneLst[robID][seq[i]][seq[i + 1]] += 1 / (fitness * self._robNum *self._taskNum)
        if self.eliteRule == '_Gbt':
            encode = self.hof[0]
            for robID in range(self._robNum):
                seq = encode[robID]
                if self.updatingMechanism == '_Trad':
                    self.robTaskPheromoneLst[robID][seq[0]] +=  1 / (self.hof[0].fitness.values[0] )
                if self.updatingMechanism == '_Prob':
                    if len(seq) != 0:
                        self.robTaskPheromoneLst[robID][seq[0]] +=  1 / (self.hof[0].fitness.values[0] * self._robNum *self._taskNum)
                for i in range(len(seq) - 1):
                    if self.updatingMechanism == '_Trad':
                        self.taskPheromoneLst[robID][seq[i]][seq[i + 1]] += 1 / (self.hof[0].fitness.values[0])
                    if self.updatingMechanism == '_Prob':
                        self.taskPheromoneLst[robID][seq[i]][seq[i + 1]] += 1 / (self.hof[0].fitness.values[0] * self._robNum * self._taskNum)
    def localUpdatingPheronome_acs(self,sol):
        for robID in range(self._robNum):
            seq = sol[robID]
            self.robTaskPheromoneLst[robID][seq[0]]  = self.robTaskPheromoneLst[robID][seq[0]] * self.decay
            self.robTaskPheromoneLst[robID][seq[0]] += (1-self.decay)*1/(self._min_h_fit * self._taskNum)
            for i in range(len(seq) - 1):
                self.taskPheromoneLst[robID][seq[i]][seq[i + 1]] = self.taskPheromoneLst[robID][seq[i]][seq[i + 1]] * self.decay
                self.taskPheromoneLst[robID][seq[i]][seq[i + 1]] += (1-self.decay)/(self._min_h_fit * self._taskNum)

    def globalUpdatingPheronome_acs(self):
        for robID in range(self._robNum):
            seq = self.hof[0][robID]
            self.robTaskPheromoneLst[robID][seq[0]]  = self.robTaskPheromoneLst[robID][seq[0]] * self.decay
            self.robTaskPheromoneLst[robID][seq[0]] += (1-self.decay)/(self.hof[0].fitness.values[0])
            for i in range(len(seq) - 1):
                self.taskPheromoneLst[robID][seq[i]][seq[i + 1]] = self.taskPheromoneLst[robID][seq[i]][seq[i + 1]] * self.decay
                self.taskPheromoneLst[robID][seq[i]][seq[i + 1]] += (1-self.decay)/(self.hof[0].fitness.values[0])

    def localSearchBaseInd(self,baseInd:[],baseFitness):
        localBase = copy.deepcopy(baseInd)
        localBaseSize = []
        # for robID in range
        for robID in range(self._robNum):
            localBase.append(copy.deepcopy(baseInd[robID]))
            localBaseSize.append(len(baseInd[robID]))
            seq = [x for x in range(self._taskNum)]
            for unit in localBase[robID]:
                seq.remove(unit)
            random.shuffle(seq)
            localBase[robID].extend(seq)
        decoder = MPDADecoder(self._ins)
        indLst = []
        for x in range(self._taskNum * self.localSearchIndNum):
            ind = copy.deepcopy(localBase)
            # decoder.decode(ind)
            r_step = random.randint(1, self.localSearchRowNum)
            if r_step > self._robNum:
                r_step = self._robNum - 1
            r_stepLst = random.sample(list(range(self._robNum)), r_step)
            for rdRobID in r_stepLst:
                # print(ind[rdRobID])
                perm = permutationSinglePointSwapACO(ind[rdRobID], localBaseSize[rdRobID])
                ind[rdRobID] = perm
                # print(ind[rdRobID])
            # indLst.append(ind)
            pb, _actSeq = decoder.decode(ind)
            fitness = decoder.calMakespan()
            indLst.append((_actSeq, fitness))
        # minlInd = min(lIndLst, key=lambda x: x.fitness.values[0])
        resInd = [[] for x in range(self._robNum)]
        resFitness = baseFitness
        minInd = min(indLst, key=lambda x: x[1])
        if minInd[1] < baseFitness:
            perm = minInd[0].convert2MultiPerm(self._ins._robNum)
            for robID in range(self._robNum):
                resInd[robID] = perm[robID]
            resFitness = minInd[1]
        if resFitness < baseFitness:
            return  True,resInd, resFitness
        else:
            return  False,None,None
    def limitedNumMechanism(self):

        for ind in self.pop:
            ind.actionSeq
            # pop[i].actionSeq
        pass

    def writeDir(self, f_con, RecordDic, gen, NFE,runTime,hofFitness):
        f_con.write('gen '+ str(gen) + ' ')
        f_con.write(str(NFE) + ' ')
        f_con.write(str(runTime) + ' ')
        f_con.write(str(RecordDic['avg']) + ' ')
        f_con.write(str(RecordDic['std']) + ' ')
        f_con.write(str(RecordDic['min']) + ' ')
        f_con.write(str(RecordDic['max']) + ' ')
        f_con.write(str(hofFitness) + '\n')
        f_con.flush()

    def drawPlt(self,insName):

        import readcfg as r_d
        import matplotlib.pyplot as plt

        fileName = BaseDir + '//debugData//'+ insName + '//' + self._algName + '//r_'+ str(self.rdSeed)  +'.dat'

        rdData = r_d.Read_Cfg(fileName)

        genLst = []
        fitLst = []
        hofFitLst = []
        NFELst = []
        runTimeLst = []

        with open(fileName) as txtData:
            lines = txtData.readlines()
            for line in lines:
                lineData = line.split()
                if (len(lineData) == 0):
                    continue
                if (len(lineData) == 9):
                    # print(lineData)
                    if lineData[0] == 'gen':
                        genLst.append(int(lineData[1]))
                        fitLst.append(float(lineData[6]))
                        NFELst.append(float(lineData[2]))
                        runTimeLst.append(float(lineData[3]))
                        hofFitLst.append(float(lineData[8]))
        _fitLst = []
        _hofLst = []
        _meanLst = []
        _genLst = []
        for gen, _data in enumerate(fitLst):
            # _meanLst.append(np.mean(_data))
            _genLst.append(gen)
            _fitLst.append(_data)
            _hofLst.append(hofFitLst[gen])
        plt.plot(_genLst, _fitLst, label = 'iter')
        plt.plot(_genLst,_hofLst, label = 'global')
        # plt.plot(_genLst, _minLst)
        plt.show()

if __name__ == '__main__':
    ins = MPDAInstance()
    benchmarkName = '11_11_CLUSTERED_RANDOM_UNITARY_UNITARY_UNITARY_thre0.1MPDAins'
    benchmarkName = '11_15_RANDOM_CLUSTERED_MSVFLV_SVLCV_LVLCV_thre0.1MPDAins'
    benchmarkName = '23_23_RANDOMCLUSTERED_RANDOM_LVSCV_UNITARY_QUADRANT_thre0.1MPDAins'
    # benchmarkName = '1_25_ECCENTRIC_CLUSTERED_LVLCV_LVLCV_LVLCV_thre0.1MPDAins'
    benchmarkName = '10_15_RANDOMCLUSTERED_RANDOMCLUSTERED_SVSCV_LVSCV_QUADRANT_thre0.1MPDAins'
    benchmarkName = '20_20_RANDOM_RANDOM_UNITARY_LUNITARY_LVLCV_thre0.1MPDAins'
    benchmarkName = '10_10_RANDOM_RANDOM_UNITARY_LUNITARY_LVLCV_thre0.1MPDAins'
    # benchmarkName = '11_11_CLUSTERED_RANDOMCLUSTERED_SVSCV_MSVFLV_MSVFLV_thre0.1MPDAins'
    # benchmarkName = '10_15_RANDOMCLUSTERED_RANDOMCLUSTERED_SVSCV_LVSCV_QUADRANT_thre0.1MPDAins'
    # benchmarkName = '11_15_RANDOM_RANDOMCLUSTERED_SVSCV_MSVFLV_QUADRANT_thre0.1MPDAins'
    # benchmarkName = '3_25_ECCENTRIC_CLUSTERED_LVLCV_LVLCV_LVLCV_thre0.1MPDAins'
    benchmarkName = '23_20_CENTRAL_RANDOMCLUSTERED_MSVFLV_SVLCV_QUADRANT_thre0.1MPDAins'
    benchmarkName = '17_23_RANDOMCLUSTERED_RANDOM_SVSCV_SVLCV_SVLCV_thre0.1MPDAins'
    benchmarkName = '10_15_CENTRAL_RANDOM_SVSCV_SVSCV_LVLCV_thre0.1MPDAins'
    benchmarkName = '10_20_CENTRAL_RANDOM_MSVFLV_SVSCV_LVLCV_thre0.1MPDAins'
    # benchmarkName = '10_20_CENTRAL_RANDOM_SVSCV_SVSCV_LVLCV_thre0.1MPDAins'
    benchmarkName = '20_10_CENTRAL_RANDOM_MSVFLV_SVSCV_LVLCV_thre0.1MPDAins'
    benchmarkName = '10_15_CENTRAL_RANDOM_MSVFLV_SVSCV_LVLCV_thre0.1MPDAins'
    benchmarkName = '10_10_CENTRAL_RANDOM_SSVSCV_SVSCV_LVLCV_thre0.1MPDAins'
    benchmarkName = '10_20_CENTRAL_RANDOM_MSVFLV_SVSCV_LVLCV_thre0.1MPDAins'
    benchmarkName = '10_15_CENTRAL_RANDOM_MSVFLV_SVSCV_LVLCV_thre0.1MPDAins'
    benchmarkName = '40_30_CENTRAL_RANDOM_SVSCV_SVSCV_LVLCV_thre0.1MPDAins'
    benchmarkName = '40_40_ECCENTRIC_RANDOMCLUSTERED_SSVSCV_SVSCV_LVLCV_thre0.1MPDAins'
    benchmarkName = '15_10_CENTRAL_CLUSTERED_SVSCV_TLVLCV_LVLCV_thre0.1MPDAins'
    benchmarkName = '15_10_ECCENTRIC_CLUSTERED_SSVSCV_SVSCV_LVLCV_thre0.1MPDAins'
    benchmarkName = '20_10_ECCENTRIC_CLUSTERED_SVSCV_SVSCV_LVLCV_thre0.1MPDAins'
    benchmarkName = '20_20_ECCENTRIC_RANDOMCLUSTERED_SSVSCV_SSVSCV_LVLCV_thre0.1MPDAins'
    # benchmarkName = '15_30_CENTRAL_CLUSTERED_SVSCV_SVSCV_LVLCV_thre0.1MPDAins'
    # benchmarkName = '3_15_ECCENTRIC_CLUSTERED_SSVSCV_SSVSCV_LVLCV_thre0.1MPDAins'
    benchmarkName = '5_20_ECCENTRIC_RANDOM_SSVSCV_SSVSCV_LVLCV_thre0.1MPDAins'
    # benchmarkName = '10_15_CENTRAL_RANDOMCLUSTERED_SSVSCV_SSVSCV_LVLCV_thre0.1MPDAins'
    benchmarkName = '10_10_CENTRAL_RANDOM_SVSCV_SLVLCV_LVLCV_thre0.1MPDAins'
    benchmarkName = '40_10_ECCENTRIC_RANDOM_SVSCV_TLVLCV_LVLCV_thre0.1MPDAins'
    randomSeed = 5
    maxRunTime = 400
    sampleRate = 100
    decay = 95
    heuristicRule = '_Dis'
    heuristicRule = '_Pdis'
    # heuristicRule = '_CDis'
    eliteRule = '_Gbt'
    # eliteRule = '_None'
    # eliteRule = '_None'
    localSearch = '_None'
    updatingMechanism = '_Trad'
    updatingMechanism = '_Prob'
    # updatingMechanism = '_ACS'
    # updatingMechanism = '_Eset'
    # selectedMechanism  = '_Limit'
    selectedMechanism = '_Trad'
    localSearchMechanism = '_AllTri'
    localSearchMechanism = '_None'
    localSearchMechanism = '_Trad'
    localSearchMechanism = '_None'
    fixMechanism = '_Trad'
    fixMechanism = '_None'
    # fixMechanism = ''
    # fixMechanism = '_None'

    # fixMechanism = ''
    # localSearchMechanism = '_EqTrad'
    localSearchIndNum  = 10
    localSearchRowNum = 3
    # updatingMechanism = '_Prob'
    # ins.loadCfg(fileName= BaseDir + '//conbenchmark//'+benchmarkName+'.dat')
    # ins.loadCfg(fileName= BaseDir + '//testBenchmark//'+benchmarkName+'.dat')
    # ins.loadCfg(fileName = BaseDir +'//anaBenchmark//' + benchmarkName +'.dat')
    ins.loadCfg(fileName = BaseDir + '//f_anaBenchmark//' + benchmarkName + '.dat')
    # ins.loadCfg(fileName= BaseDir +'//sBenchmark//' + benchmarkName +'.dat')
    # ins.drawIns()
    # exit()
    # ins.loadCfg(fileName= BaseDir +'//largeAbiConBenchmark//' + benchmarkName +'.dat')
    mpda_as = MPDA_Task_ACO(ins, benchmarkName=benchmarkName,
                       decay=decay, sampleRate=sampleRate,
                       rdSeed=randomSeed,
                       maxRunTime=maxRunTime,
                       heuristicRule = heuristicRule,
                           eliteRule = eliteRule,
                            localSearchMechanism = localSearchMechanism
                            ,localSearchIndNum = localSearchIndNum,
                            localSearchRowNum = localSearchRowNum,
                            updatingMechanism = updatingMechanism,
                            selectedMechanism = selectedMechanism,
                            fixMechanism = fixMechanism)
    print(mpda_as)
    mpda_as.run()
    mpda_as.drawPlt(benchmarkName)