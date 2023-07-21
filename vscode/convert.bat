@echo off

set ONSHAPE_API="https://cad.onshape.com"
set ONSHAPE_ACCESS_KEY="dvzjop0q6z8LowpPDrYt7ZGT"
set ONSHAPE_SECRET_KEY="cpHZ3UTDp2Z8GrIPbPJYDb0o0Il6YFDCswr3i7btPvoi5CWi"

onshape-to-robot %1
python sdf-formatter.py %1
python stl2obj.py %1 --extension stl
