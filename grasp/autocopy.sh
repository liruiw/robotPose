#!/bin/bash
cp empty.xml $GRASPIT/worlds/
cp -r panda $GRASPIT/models/robots
#python script/scale_wrl_file.py # run this to generate ycb objects
cp script/meshes/* $GRASPIT/models/objects/
cp src/searchStateImpl.cpp $GRASPIT/src/EGPlanner/
cp src/searchStateImpl.h $GRASPIT/include/graspit/EGPlanner/
