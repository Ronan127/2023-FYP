#!/bin/sh

#Obtained at https://dev-portal.onshape.com/keys
export ONSHAPE_API=https://cad.onshape.com
export ONSHAPE_ACCESS_KEY=dvzjop0q6z8LowpPDrYt7ZGT
export ONSHAPE_SECRET_KEY=cpHZ3UTDp2Z8GrIPbPJYDb0o0Il6YFDCswr3i7btPvoi5CWi

onshape-to-robot $1
python sdf-formatter.py $1
python stl2obj.py $1 --extension stl
python set_friction.py $1 --wheel 1.15 --tail 0.1 --frame 0.1

