from paraview.simple import *
import sys
import os

path = sys.argv[1]

# create a new 'OpenFOAMReader'
foamReader = OpenFOAMReader(FileName=path)

# Properties modified on foamReader
if sys.argc==4 and sys.argv[3]=="-d":
    foamReader.CaseType = 'Decomposed Case'

foamReader.UpdatePipeline()
foamReader.UpdatePipelineInformation()

# Properties modified on foamReader
foamReader.MeshRegions = ['internalMesh']
foamReader.CellArrays = ['U']
foamReader.UpdatePipeline()

# create a new 'Cell Centers'
cellCenters1 = CellCenters(Input=foamReader)

# Properties modified on cellCenters1
cellCenters1.VertexCells = 1
cellCenters1.UpdatePipeline()

# save data
directory=sys.argv[2]

if not os.path.exists(directory):
    os.makedirs(directory)

SaveData(directory+"/"+sys.argv[2]+".csv", proxy=cellCenters1, WriteAllTimeSteps=1)
