# -*- coding: utf-8 -*-
"""
Created on Sat May 12 20:47:33 2018
@author: robot

"""
    

def writeConf(fcon ,name = '0-0',data = []):
    fcon.write(name+' ')
    for unit in data:
        fcon.write(' '+ str(unit))
    fcon.write('\n')


class Read_Cfg:
    def __init__(self,fileName = '0-0'):
        self.fileName = fileName
    '''
    get the list val in txt    
    '''
    def get(self, str_val ='val' ,ListVal = [],dtype = 'float'):
        ListVal.clear()
        getBoolean = False
        with open(self.fileName)  as txtData:
           lines = txtData.readlines()
           for line in lines:
               lineData = line.split()
               if(len(lineData)==0):
                   continue
               else:
                   if(lineData[0]==str_val):
                       for i in range(1,len(lineData)):
                            if dtype == 'int':
                                ListVal.append(int(lineData[i]))
                            if dtype == 'float':
                                ListVal.append(float(lineData[i]))
                       getBoolean = True
                       break
                   else:
                       continue
        return getBoolean

    def getSingleVal(self, str_val ='val',dtype = 'double'):
#        ListVal.clear()
        getBoolean = False
        with open(self.fileName)  as txtData:
           lines = txtData.readlines()
           for line in lines:
               lineData = line.split()
               if(len(lineData)<=1):
                   continue
               else:
#                   print(lineData[0])
                   if(lineData[0]==str_val):
                       val = float(lineData[1])
                       break
                   else:
                       continue
#        print('val = ',val)
        if dtype == 'double':
            return val
        if dtype == 'int':
            val = int(val)
        return val

if __name__ == '__main__':
    
    conFileDir = './/data//'    
    degNameCfg = conFileDir + 'map_debug.txt'
    print(degNameCfg)
    readCfg = Read_Cfg(degNameCfg)
    lista = []
    readMark = readCfg.get('range_y',lista)
    print('____')
    readMark = readCfg.get('grid_x',lista)

    print(lista)
    if(readCfg.get('range_x'),lista):
        print('success')
    else:
        print('failed')
    
    
