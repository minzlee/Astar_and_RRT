#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/minzheplugin')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')
    RRTModule = RaveCreateModule(env,'RRTModule')
    print RRTModule.SendCommand('help')
finally:
    RaveDestroy()
