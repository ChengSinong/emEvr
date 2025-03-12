#!../../bin/linux-x86_64/emEvr

#- You may have to change emEvr to something else
#- everywhere it appears in this file

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/emEvr.dbd"
emEvr_registerRecordDeviceDriver pdbbase

## Load record instances

dbLoadRecords "db/emEvr.db", "user=control"

#- Set this to see messages from mySub
#-var mySubDebug 1

#- Run this to trace the stages of iocInit
#-traceIocInit

cd "${TOP}/iocBoot/${IOC}"

configure_emEvr("uio0",0)
configure_event_handler

iocInit

## Start any sequence programs
#seq sncExample, "user=control"
