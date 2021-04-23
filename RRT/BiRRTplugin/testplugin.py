#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/BiRRTplugin')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')
    BiRRTModule = RaveCreateModule(env,'BiRRTModule')
    print BiRRTModule.SendCommand('help')
finally:
    RaveDestroy()
