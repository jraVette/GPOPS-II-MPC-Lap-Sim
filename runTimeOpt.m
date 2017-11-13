function runTimeOpt(fileToLoad)

global DAQ
loadFiles(pwd,fileToLoad)
daq = DAQ{1};
[daq,lapTime,conv] = gpopsMPC_withSwitchingAndAD(daq);





    
