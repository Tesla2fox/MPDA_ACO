import os
import sys

'''
'''

from mpdaACO import MPDA_Task_ACO
from mpdaInstance import MPDAInstance


if __name__ == '__main__':
    print('begin to run task ACO\n')
    ins = MPDAInstance()
    # insConfDir = './/scopeBenchmark//'
    insConfDir = './/staticMpdaBenchmarkSet//'
    # insConfDir = './/anaBenchmark//'
    print(sys.argv)
    print(len(sys.argv))

    if len(sys.argv) == 14:
        benchmarkName = sys.argv[1]
        randomSeed = int(sys.argv[2])
        sampleRate = float(sys.argv[3])
        decay = float(sys.argv[4])
        heuristicRule = sys.argv[5]
        eliteRule = sys.argv[6]
        maxRunTime = float(sys.argv[7])
        updatingMechanism = sys.argv[8]
        selectedMechanism = sys.argv[9]
        localSearchMechanism = sys.argv[10]
        localSearchIndNum = int(sys.argv[11])
        localSearchRowNum = int(sys.argv[12])
        fixMechanism = sys.argv[13]
        # '_None'
        # maxRunTime = float(sys.argv[5])
    elif len(sys.argv) == 1:
        benchmarkName = 'M_20_20_0.97'
        randomSeed = 11
        maxRunTime = 200
        sampleRate = 100
        decay = 95
        heuristicRule = '_Dis'
        heuristicRule = '_Pdis'
        heuristicRule = '_LPdis'
        # heuristicRule = '_Gta'
        # heuristicRule = '_Dvis'
        # heuristicRule = '_None'
        # heuristicRule = '_Syn'
        eliteRule = '_Gbt'
        # eliteRule = '_None'
        # eliteRule = '_None'
        localSearch = '_None'
        updatingMechanism = '_Trad'
        updatingMechanism = '_Prob'
        # updatingMechanism = '_ACS'
        # updatingMechanism = '_Eset'
        selectedMechanism = '_Limit'
        selectedMechanism = '_Trad'
        localSearchMechanism = '_AllTri'
        localSearchMechanism = '_Trad'
        fixMechanism  = '_Trad'
        localSearchIndNum = 1
        localSearchRowNum = 2
    else:
        raise Exception('something wrong on the sys.argv')
        pass
    print(sys.argv)
    ins.loadCfg(fileName=insConfDir + benchmarkName + '.txt')
    print(benchmarkName)
    mpda_as = MPDA_Task_ACO(ins, benchmarkName=benchmarkName,
                       decay=decay, sampleRate=sampleRate,
                       rdSeed=randomSeed,
                       maxRunTime=maxRunTime,
                       heuristicRule = heuristicRule,
                           eliteRule = eliteRule,
                            localSearchMechanism = localSearchMechanism,
                            localSearchIndNum= localSearchIndNum,
                            localSearchRowNum= localSearchRowNum,
                            updatingMechanism = updatingMechanism,
                            selectedMechanism = selectedMechanism,
                            fixMechanism = fixMechanism)
    mpda_as.run()
