from tkinter import *
from tkinter import filedialog
import re


root = Tk()
root.geometry('780x700')

left=Frame(root, width=350)
right=Frame(root, width=350)
left.grid(row=0, column=0, sticky=N)
right.grid(row=0, column=1, sticky=N)

leftList=[]
modelFiles=[]
windF = ""


###################################################################################################################################
#   GADEN_preprocessing.launch
###################################################################################################################################

def getModelFile(ent):
    file= filedialog.askopenfilename(filetypes=(("STL files", "*.stl"), ("All files", "*.*") ) )
    ent.delete(0,END)
    ent.insert(0, file)

def getCSVFile(ent):
    file= filedialog.askopenfilename(filetypes=(("CSV files", "*.csv"), ("All files", "*.*") ) )
    ent.delete(0,END)
    ent.insert(0, file)

def addFile():
    entryModel = Entry(left)
    modelLabel = Label(left, text="Model "+str(len(modelFiles)))
    modelFileButton = Button(left, text="...", command=lambda: getModelFile(entryModel))

    #rearrange the grid to fit the new entry field
    leftList.insert(len(modelFiles)+2,[modelLabel,entryModel,modelFileButton])
    modelFiles.append(entryModel)
    for row in range(0,len(leftList)):
        for col in range(0,len(leftList[row])):
            leftList[row][col].grid(row=row, column=col)


#entry field for size of cell side
cellSizeLabel= Label(left, text="Cell size: ")
cellSizeEntry = Entry(left)

#label for list of models
modelsLabel= Label(left, text="STL models of environment")

#button to create more filepath entry fields
addFileButton= Button(left, text="+", command=addFile)

#empty point coordinates
emptyPointLabel=Label(left, text="Coordinates of empty point")
xEmptyPoint = Entry(left)
yEmptyPoint = Entry(left)
zEmptyPoint = Entry(left)
xEmptyPoint.insert(END, "x")
yEmptyPoint.insert(END, "y")
zEmptyPoint.insert(END, "z")

#wind files
windLabel = Label(left, text="Wind files")
homogeneous=BooleanVar()
homogeneous.set(False)
check = Checkbutton(left, text="Homogeneous", variable=homogeneous, onvalue=True, offvalue=False)
windEntry=Entry(left)
windFileButton = Button(left, text="...", command=lambda: getCSVFile(windEntry))


leftList.append([cellSizeLabel, cellSizeEntry])
leftList.append([modelsLabel])
leftList.append([addFileButton])
leftList.append([emptyPointLabel])
leftList.append([xEmptyPoint])
leftList.append([yEmptyPoint])
leftList.append([zEmptyPoint])
leftList.append([windLabel])
leftList.append([check,windEntry,windFileButton])

#create the first file entry field for CAD models automatically
addFile()


#right side of window
outletModelFiles=[]
rightList=[]
def addOutletFile():
    entryModel = Entry(right)
    modelLabel = Label(right, text="Outlet model "+str(len(outletModelFiles)))
    modelFileButton = Button(right, text="...", command=lambda: getModelFile(entryModel))    

    #rearrange the grid to fit the new entry field
    rightList.insert(len(outletModelFiles)+2,[modelLabel,entryModel,modelFileButton])
    outletModelFiles.append(entryModel)
    for row in range(0,len(rightList)):
        for col in range(0,len(rightList[row])):
            rightList[row][col].grid(row=row, column=col)

def getOutputFile(ent):
    file= filedialog.askdirectory()
    ent.delete(0,END)
    ent.insert(0, file)

#empty row
emptyLabel= Label(right, text=" ")

#label for list of outlet
outletsLabel= Label(right, text="STL models of outlets")

#button to create more filepath entry fields
addOutletButton= Button(right, text="+", command=addOutletFile)

#output path for files
outputLabel = Label(right, text="Preprocessing output path: ")
outputEntry=Entry(right) 
outputButton = Button(right, text="...", command=lambda: getOutputFile(outputEntry))

#save launch file
def savePreprocessingFile():
    global windF
    f=open("GADEN_preprocessing.launch","w")
    f.write(
    "<launch>\n"+
    "   <node pkg=\"gaden_preprocessing\" type=\"preprocessing\" name=\"preprocessing\" output=\"screen\">\n"+
    "       <param name=\"cell_size\" value=\""+cellSizeEntry.get()+"\"/>\n\n"+
    "       <param name=\"number_of_models\" value=\""+str(len(modelFiles))+"\"/>\n"
    )
    for ind in range(0,len(modelFiles)):
        f.write(
        "       <param name=\"model_"+str(ind)+"\" value=\""+modelFiles[ind].get()+"\"/>\n")
    f.write("\n"+
    "       <param name=\"number_of_outlet_models\" value=\""+str(len(outletModelFiles))+"\"/>\n")


    for ind in range(0,len(outletModelFiles)):
        f.write(
        "       <param name=\"outlets_model_"+str(ind)+"\" value=\""+outletModelFiles[ind].get()+"\"/>\n")

    windF = re.sub("_0.csv","",windEntry.get())
    f.write(
        "\n"+
        "       <param name=\"empty_point_x\" value=\""+str(xEmptyPoint.get())+"\"/>\n"+
        "       <param name=\"empty_point_y\" value=\""+str(yEmptyPoint.get())+"\"/>\n"+
        "       <param name=\"empty_point_z\" value=\""+str(zEmptyPoint.get())+"\"/>\n\n"+
        "       <param name=\"uniformWind\" value=\""+str(homogeneous.get())+"\"/>\n"
        "       <param name=\"wind_files\" value=\""+windF+"\"/>\n\n"+
        "       <param name=\"output_path\" value=\""+outputEntry.get()+"\"/>\n"
        "   </node>\n"+
        "</launch>"
    )
    savedLabel= Label(right, text="Launch file saved!")
    nextButton = Button(right, text="Continue", command=filamentSim)
    rightList.append([savedLabel])
    rightList.append([nextButton])
    for row in range(0,len(rightList)):
        for col in range(0,len(rightList[row])):
            rightList[row][col].grid(row=row, column=col)
    
saveButton = Button(right, text="Save", command=savePreprocessingFile)


rightList.append([emptyLabel])
rightList.append([outletsLabel])
rightList.append([addOutletButton])
rightList.append([outputLabel, outputEntry, outputButton])
rightList.append([saveButton])
addOutletFile()


###################################################################################################################################
#   GADEN.launch
###################################################################################################################################

def getColladaFile(ent):
    file= filedialog.askopenfilename(filetypes=(("Collada files", "*.dae"), ("All files", "*.*") ) )
    ent.delete(0,END)
    ent.insert(0, file)

def getWindFile(ent):
    file= filedialog.askopenfilename(filetypes=(("Wind files", "*_U"), ("All files", "*.*") ) )
    ent.delete(0,END)
    ent.insert(0, file)

def addColladaFile():
    entryModel = Entry(left)
    modelLabel = Label(left, text="Model "+str(len(modelFiles)))
    modelFileButton = Button(left, text="...", command=lambda: getColladaFile(entryModel))

    #rearrange the grid to fit the new entry field
    leftList.insert(len(modelFiles)+2,[modelLabel,entryModel,modelFileButton])
    modelFiles.append(entryModel)
    for row in range(0,len(leftList)):
        for col in range(0,len(leftList[row])):
            leftList[row][col].grid(row=row, column=col)

startWind = Entry(left)
endWind = Entry(left)

def loopWind(*args):
    if looping.get():
        startLabel = Label(left,text="From:")
        endLabel = Label(left,text="To:")
        leftList.append([startLabel, startWind])
        leftList.append([endLabel, endWind])
    else:
        for col in range(0,len(leftList[len(leftList)-1])):
            leftList[len(leftList)-1][col].grid_forget()
        leftList.pop(len(leftList)-1)
        for col in range(0,len(leftList[len(leftList)-1])):
            leftList[len(leftList)-1][col].grid_forget()
        leftList.pop(len(leftList)-1)

    for row in range(0,len(leftList)):
        for col in range(0,len(leftList[row])):
            leftList[row][col].grid(row=row, column=col)

def filamentSim():
    global leftList, rightList, modelFiles, looping
    for row in range(0,len(leftList)):
        for col in range(0,len(leftList[row])):
            leftList[row][col].grid_forget()
    
    for row in range(0,len(rightList)):
        for col in range(0,len(rightList[row])):
            rightList[row][col].grid_forget()
    leftList=[]
    rightList=[]
    modelFiles=[]

    windEntryFilament.insert(END, windF+"_0.csv_U")
    occupancy3DEntry.insert(END, outputEntry.get()+"/OccupancyGrid3D.csv")

    leftList.append([occupancyLabel,occupancy3DEntry,occupancy3DButton])
    leftList.append([modelsLabel])
    leftList.append([addFileButton])
    leftList.append([SourcePosLabel])
    leftList.append([xSourcePos])
    leftList.append([ySourcePos])
    leftList.append([zSourcePos])
    leftList.append([windLabelFilament,windEntryFilament,windButtonFilament])
    leftList.append([windDurationLabel, windDurationEntry])
    leftList.append([checkLooping])
    addColladaFile()

    rightList.append([checkVerbose])
    rightList.append([simTimeLabel,simTimeEntry])
    rightList.append([numFilamentsLabel,numFilamentsEntry])
    rightList.append([checkVariableRate])
    rightList.append([ppmLabel,ppmEntry])
    rightList.append([initialStdLabel,initialStdEntry])
    rightList.append([growthGammaLabel,growthGammaEntry])
    rightList.append([initialStdLabel,initialStdEntry])
    rightList.append([noiseLabel,noiseEntry])
    rightList.append([gasTypeLabel,gasTypeChoice])
    rightList.append([temperatureLabel,temperatureEntry])
    rightList.append([pressureLabel,pressureEntry])
    rightList.append([emptyLabel])
    rightList.append([resultsPathLabel,resultsPathEntry,resultsPathButton])
    rightList.append([resultsMinLabel,resultsMinEntry])
    rightList.append([resultsIntervalLabel,resultsIntervalEntry])
    rightList.append([saveFilButton])


    for row in range(0,len(rightList)):
        for col in range(0,len(rightList[row])):
            rightList[row][col].grid(row=row, column=col)


occupancyLabel = Label(left, text="Occupancy3D file: ")
occupancy3DEntry=Entry(left)
occupancy3DButton = Button(left, text="...", command=lambda: getCSVFile(occupancy3DEntry))

#label for list of models
modelsLabel= Label(left, text="Collada models (visualization)")
#button to create more filepath entry fields
addFileButton= Button(left, text="+", command=addColladaFile)

#source position
SourcePosLabel=Label(left, text="Coordinates of gas source")
xSourcePos = Entry(left)
ySourcePos = Entry(left)
zSourcePos = Entry(left)
xSourcePos.insert(END, "x")
ySourcePos.insert(END, "y")
zSourcePos.insert(END, "z")

#wind options
windLabelFilament = Label(left, text="Wind filepath: ")
windEntryFilament=Entry(left)
windButtonFilament = Button(left, text="...", command=lambda: getWindFile(windEntryFilament))

windDurationLabel = Label(left, text="Seconds per\nwind snapshot")
windDurationEntry = Entry(left)

looping = BooleanVar()
looping.set(False)
checkLooping = Checkbutton(left, text="Allow looping", variable=looping, onvalue=True, offvalue=False)
looping.trace("w",loopWind)


### right side

verbose = BooleanVar()
verbose.set(False)
checkVerbose = Checkbutton(right, text="Verbose", variable=verbose, onvalue=True, offvalue=False)

simTimeLabel= Label(right, text="Simulation length (s)")
simTimeEntry = Entry(right)
simTimeEntry.insert(END, "300")

timeStepLabel= Label(right, text="Time step (s)")
timeStepEntry = Entry(right)
timeStepEntry.insert(END, "0.1")

numFilamentsLabel= Label(right, text="Filaments/second")
numFilamentsEntry = Entry(right)
numFilamentsEntry.insert(END, "10")

variableRate = BooleanVar()
variableRate.set(False)
checkVariableRate = Checkbutton(right, text="Variable rate", variable=variableRate, onvalue=True, offvalue=False)

ppmLabel= Label(right, text="Concentration at\nfilament center(ppm)")
ppmEntry = Entry(right)
ppmEntry.insert(END, "10")

initialStdLabel= Label(right, text="Initial stdDev (cm)")
initialStdEntry = Entry(right)
initialStdEntry.insert(END, "5")

growthGammaLabel= Label(right, text="Filament growth (cm²/s)")
growthGammaEntry = Entry(right)
growthGammaEntry.insert(END, "5")

noiseLabel= Label(right, text="Movement noise (m)")
noiseEntry = Entry(right)
noiseEntry.insert(END, "0.01")

gasTypeLabel=Label(right, text="Gas Type:")
gasTypeList=["Ethanol","Methane","Hydrogen","Propanol","Chlorine","Fluorine","Acetone","Neon","Helium"]
gasType = StringVar(root)
gasType.set("Ethanol")
gasTypeChoice=OptionMenu(right, gasType, *gasTypeList)

temperatureLabel= Label(right, text="Temperature (K)")
temperatureEntry = Entry(right)
temperatureEntry.insert(END, "298")

pressureLabel= Label(right, text="Pressure (atm)")
pressureEntry = Entry(right)
pressureEntry.insert(END, "1")

#output path for files
resultsPathLabel = Label(right, text="Results directory path: ")
resultsPathEntry=Entry(right) 
resultsPathButton = Button(right, text="...", command=lambda: getOutputFile(resultsPathEntry))

resultsMinLabel= Label(right, text="Results start (s)")
resultsMinEntry = Entry(right)
resultsMinEntry.insert(END, "0.0")

resultsIntervalLabel= Label(right, text="Results interval (s)")
resultsIntervalEntry = Entry(right)
resultsIntervalEntry.insert(END, "0.5")

#save launch file
def saveFilamentSimFile():
    global windF
    f=open("GADEN.launch","w")
    f.write(
    "<launch>\n"+
    "   <node pkg=\"gaden_environment\" type=\"environment\" name=\"environment\" output=\"screen\">\n"+
    "       <param name=\"verbose\" value=\"false\"/>\n"+
    "       <param name=\"wait_preprocessing\" value=\"false\"/>\n"+
    "       <param name=\"number_of_CAD\" value=\""+str(len(modelFiles))+"\"/>\n"+
    "       <rosparam subst_value=\"True\">\n"
    )
    for ind in range(0,len(modelFiles)):
        f.write(
        "           CAD_"+str(ind)+": file:// "+modelFiles[ind].get()+"\n"+
        "           CAD_"+str(ind)+"_color: [0.5,0.5,0.5]\n")
    
    f.write("       </rosparam>\n\n"+
    "       <param name=\"occupancy3D_data\" value=\""+occupancy3DEntry.get()+"\"/>\n\n"+
    "       <param name=\"number_of_sources\" value=\"1\"/>\n"+
    "       <param name=\"source_0_position_x\" value=\""+xSourcePos.get()+"\"/>\n"+
    "       <param name=\"source_0_position_y\" value=\""+ySourcePos.get()+"\"/>\n"+
    "       <param name=\"source_0_position_z\" value=\""+zSourcePos.get()+"\"/>\n"+
    "       <rosparam>\n"+
    "           source_0_scale: 0.2\n"
    "           source_0_color: [0.0, 1.0, 0.0]\n" 
    "       </rosparam>\n"+
    "</node>\n\n")

    f.write(
    "   <node pkg=\"gaden_filament_simulator\" type=\"filament_simulator\" name=\"filament_simulator\" output=\"screen\">\n"+
    "       <param name=\"verbose\" value=\""+str(verbose.get())+"\"/>\n"+
    "       <param name=\"wait_preprocessing\" value=\"false\"/>\n"+
    "       <param name=\"sim_time\" value=\""+simTimeEntry.get()+"\"/>\n"+
    "       <param name=\"time_step\" value=\""+timeStepEntry.get()+"\"/>\n"+
    "       <param name=\"num_filaments_sec\" value=\""+numFilamentsEntry.get()+"\"/>\n"+
    "       <param name=\"variable_rate\" value=\""+str(variableRate.get())+"\"/>\n"+
    "       <param name=\"filaments_stop_steps\" value=\"0\"/>\n"+
    "       <param name=\"ppm_filament_center\" value=\""+ppmEntry.get()+"\"/>\n"+
    "       <param name=\"filament_initial_std\" value=\""+initialStdEntry.get()+"\"/>\n"+
    "       <param name=\"filament_growth_gamma\" value=\""+growthGammaEntry.get()+"\"/>\n"+
    "       <param name=\"filament_noise_std\" value=\""+noiseEntry.get()+"\"/>\n"+
    "       <param name=\"gas_type\" value=\""+str(gasTypeList.index(gasType.get()))+"\"/>\n"+
    "       <param name=\"temperature\" value=\""+temperatureEntry.get()+"\"/>\n"+
    "       <param name=\"pressure\" value=\""+pressureEntry.get()+"\"/>\n"+
    "       <param name=\"concentration_unit_choice\" value=\"1\"/>\n"+
    "       <param name=\"occupancy3D_data\" value=\""+occupancy3DEntry.get()+"\"/>\n"+
    "       <param name=\"fixed_frame\" value=\"map\"/>\n\n"+
    "       <param name=\"wind_data\" value=\""+re.sub("0.csv_U","",windEntryFilament.get())+"\"/>\n"+
    "       <param name=\"wind_time_step\" value=\""+windDurationEntry.get()+"\"/>\n"+
    "       <param name=\"allow_looping\" value=\""+str(looping.get())+"\"/>\n"+
    "       <param name=\"loop_from_step\" value=\""+startWind.get()+"\"/>\n"+
    "       <param name=\"loop_to_step\" value=\""+endWind.get()+"\"/>\n\n"+
    "       <param name=\"source_position_x\" value=\""+xSourcePos.get()+"\"/>\n"+
    "       <param name=\"source_position_y\" value=\""+ySourcePos.get()+"\"/>\n"+
    "       <param name=\"source_position_z\" value=\""+zSourcePos.get()+"\"/>\n\n"+
    "       <param name=\"save_results\" value=\"1\"/>\n"+
    "       <param name=\"results_min_time\" value=\""+resultsMinEntry.get()+"\"/>\n"+
    "       <param name=\"results_time_step\" value=\""+resultsIntervalEntry.get()+"\"/>\n"+
    "       <param name=\"writeConcentrations\" value=\"true\"/>\n"+
    "       <param name=\"results_location\" value=\""+resultsPathEntry.get()+"\"/>\n"+
    "   </node>\n\n"
    )

    f.write(
    "   <node name=\"rviz\" pkg=\"rviz\" type=\"rviz\" args=\"-d $(find test_env)/10x6_empty_room/launch/ros/gaden.rviz\"/>\n"+
    "</launch>")
    savedLabel= Label(right, text="Launch file saved!")
    nextButton = Button(right, text="Continue",command=player)
    rightList.append([savedLabel])
    rightList.append([nextButton])
    for row in range(0,len(rightList)):
        for col in range(0,len(rightList[row])):
            rightList[row][col].grid(row=row, column=col)

saveFilButton = Button(right, text="Save", command=saveFilamentSimFile)


###################################################################################################################################
#   GADEN_player.launch
###################################################################################################################################
def parseF(st):
    try:
        return "{:.2f}".format(float(st))
    except ValueError:
        return st

def player():
    global leftList, rightList, startWind, endWind
    looping.set(False)

    for row in range(0,len(leftList)):
        for col in range(0,len(leftList[row])):
            leftList[row][col].grid_forget() 
    
    for row in range(0,len(rightList)):
        for col in range(0,len(rightList[row])):
            rightList[row][col].grid_forget()

    leftList = []
    rightList = []

    leftList.append([occupancyLabel,occupancy3DEntry,occupancy3DButton])
    leftList.append([modelsLabel])
    for index in range(0,len(modelFiles)):
        entryModel = modelFiles[index]
        modelLabel = Label(left, text="Model "+str(index))
        modelFileButton = Button(left, text="...", command=lambda: getColladaFile(entryModel))
        leftList.append([modelLabel, entryModel, modelFileButton])
    leftList.append([addFileButton])

    leftList.append([initialIterationLabel, initialIterationEntry])
    
    startWind = Entry(left)
    endWind = Entry(left)
    leftList.append([checkLooping])

    for row in range(0,len(leftList)):
        for col in range(0,len(leftList[row])):
            leftList[row][col].grid(row=row, column=col)

    ####
    rightList.append([logFilesLabel])
    rightList.append([logFilesEntry, logFilesButton])
    logFilesEntry.insert(0, 
        resultsPathEntry.get()+"/FilamentSimulation_gasType_"+str(gasTypeList.index(gasType.get())) 
        +"_sourcePosition_"+parseF(xSourcePos.get())+"_"+parseF(ySourcePos.get())+"_"+parseF(zSourcePos.get())+"_iteration_"
    )

    rightList.append([frequencyLabel, frequencyEntry])
    try:
        frequencyEntry.insert(0, int(1.0/float(resultsIntervalEntry.get())))
    except:
        pass

    
    xSourcePos_2 = Entry(right)
    xSourcePos_2.insert(0, xSourcePos.get())
    ySourcePos_2 = Entry(right)
    ySourcePos_2.insert(0, ySourcePos.get())
    zSourcePos_2 = Entry(right)
    zSourcePos_2.insert(0, zSourcePos.get())

    SourcePosLabel=Label(right, text="Coordinates of gas source")
    rightList.append([SourcePosLabel])
    rightList.append([xSourcePos_2])
    rightList.append([ySourcePos_2])
    rightList.append([zSourcePos_2])
    rightList.append([savePlayerButton])
    for row in range(0,len(rightList)):
        for col in range(0,len(rightList[row])):
            rightList[row][col].grid(row=row, column=col)


def savePlayer():
    f=open("GADEN_player.launch","w")
    f.write(
    "<launch>\n"+
    "   <node pkg=\"gaden_environment\" type=\"environment\" name=\"environment\" output=\"screen\">\n"+
    "       <param name=\"verbose\" value=\"false\"/>\n"+
    "       <param name=\"wait_preprocessing\" value=\"false\"/>\n"+
    "       <param name=\"number_of_CAD\" value=\""+str(len(modelFiles))+"\"/>\n"+
    "       <rosparam subst_value=\"True\">\n"
    )
    for ind in range(0,len(modelFiles)):
        f.write(
        "           CAD_"+str(ind)+": file:// "+modelFiles[ind].get()+"\n"+
        "           CAD_"+str(ind)+"_color: [0.5,0.5,0.5]\n")
    
    f.write("       </rosparam>\n\n"+
    "       <param name=\"occupancy3D_data\" value=\""+occupancy3DEntry.get()+"\"/>\n\n"+
    "       <param name=\"number_of_sources\" value=\"1\"/>\n"+
    "       <param name=\"source_0_position_x\" value=\""+xSourcePos.get()+"\"/>\n"+
    "       <param name=\"source_0_position_y\" value=\""+ySourcePos.get()+"\"/>\n"+
    "       <param name=\"source_0_position_z\" value=\""+zSourcePos.get()+"\"/>\n"+
    "       <rosparam>\n"+
    "           source_0_scale: 0.2\n"
    "           source_0_color: [0.0, 1.0, 0.0]\n" 
    "       </rosparam>\n"+
    "</node>\n\n")

    f.write(
    "   <node pkg=\"gaden_player\" type=\"gaden_player\" name=\"gaden_player\" output=\"screen\">\n"+
    "       <param name=\"verbose\" value=\"false\" />\n"+
    "       <param name=\"player_freq\" value=\""+frequencyEntry.get()+"\"/>\n"+
    "       <param name=\"initial_iteration\" value=\""+initialIterationEntry.get()+"\"/>\n"+
    "       <param name=\"num_simulators\" value=\"1\" />\n"+
    "       <param name=\"simulation_data_0\" value=\""+logFilesEntry.get()+"\" />\n"+    
    "       <param name=\"allow_looping\" value=\""+str(looping.get())+"\" />\n"+
    "       <param name=\"loop_from_iteration\" value=\""+startWind.get()+"\" />\n"+
    "       <param name=\"loop_to_iteration\" value=\""+endWind.get()+"\" />\n"        
    "   </node>\n\n"
    )

    f.write(
    "   <node name=\"rviz\" pkg=\"rviz\" type=\"rviz\" args=\"-d $(find test_env)/10x6_empty_room/launch/ros/gaden.rviz\"/>\n"+
    "</launch>")

    doneLabel = Label(right, text="Done!")
    rightList.append([doneLabel])
    for row in range(0,len(rightList)):
        for col in range(0,len(rightList[row])):
            rightList[row][col].grid(row=row, column=col)


def getLogs(ent):
    file= filedialog.askopenfilename()
    ent.delete(0,END)
    ent.insert(0, re.sub("iteration_.*","iteration_",file))


initialIterationLabel = Label(left, text="Initial iteration:")
initialIterationEntry = Entry(left)
initialIterationEntry.insert(0, "0")

frequencyLabel = Label(right, text="Playback frequency (Hz):")
frequencyEntry = Entry(right)

logFilesLabel= Label(right, text="Simulation log files:")
logFilesEntry = Entry(right)
logFilesButton = Button(right, text="...", command=lambda: getLogs(logFilesEntry))

savePlayerButton = Button(right, text="Save", command=savePlayer)

root.mainloop()

