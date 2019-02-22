#!/bin/bash
cp empty.xml $GRASPIT/worlds/
cp -r panda $GRASPIT/models/robots
python scripts/scale_wrl_file.py # run this to generate ycb objects
cp mesh/* $GRASPIT/models/objects/
cp src/searchStateImpl.cpp $GRASPIT/src/EGPlanner/
cp src/searchStateImpl.h $GRASPIT/include/graspit/EGPlanner/