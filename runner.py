from __future__ import absolute_import
from __future__ import print_function

import optparse
import os
import random
import sys
import time
import xml.etree.ElementTree as ET
import pickle

# we need to import python modules from the $SUMO_HOME/tools directory
import numpy
import matplotlib.pyplot as plt

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary
import traci
import itertools

edgeE1in = "E1"
edgeE2in = "E2"
edgeE3in = "E3"
edgeE4in = "E4"

edgeE1out = "-E1"  # out
edgeE2out = "-E2"
edgeE3out = "-E3"
edgeE4out = "-E4"

trafficLightId = "TL1"

aimVehiclesPerSecond = 5/3


def generate_routefile(n, scenario):

    random.seed(42)  # make tests reproducible
    N = n  # number of time steps

    # demand per second from different directions
    p = numpy.zeros((12, 6))
    p[0] = [0.0500, 0.1399, 0.0313, 0.0440, 0.1200, 0.0400]  # WN
    p[1] = [0.2499, 0.6997, 0.1566, 0.1319, 0.2500, 0.1500]  # WE
    p[2] = [0.1214, 0.3399, 0.0761, 0.0880, 0.0300, 0.3000]  # WS
    p[3] = [0.0357, 0.1000, 0.0224, 0.0616, 0.1300, 0.0200]  # NE
    p[4] = [0.1070, 0.3000, 0.0670, 0.3080, 0.0400, 0.1340]  # NS
    p[5] = [0.0714, 0.1999, 0.0447, 0.1495, 0.1400, 0.2700]  # NW
    p[6] = [0.0893, 0.2499, 0.0305, 0.0440, 0.0500, 0.0800]  # ES
    p[7] = [0.2356, 0.6597, 0.1477, 0.1232, 0.1700, 0.1253]  # EW
    p[8] = [0.1071, 0.2999, 0.0671, 0.0704, 0.2600, 0.0300]  # EN
    p[9] = [0.0357, 0.1000, 0.0224, 0.0600, 0.0300, 0.1800]  # SW
    p[10] = [0.1000, 0.2799, 0.0626, 0.2903, 0.0400, 0.0700]  # SN
    p[11] = [0.0571, 0.1599, 0.0358, 0.1319, 0.0300, 0.0500]  # SE

    # 0 -> 1 Morning-Day-Rush
    # 1 -> 2 Low-High
    # 2 -> 3 High-Low
    # 3 -> 4 Hor-Ver
    # 4 -> 5 123
    # 5 -> 6 Balanced

    with open("data/tlcs.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="typeNormal" accel="2.6" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="50" \
guiShape="passenger"/>

        <route id="WE" edges="E1 -E3" />
        <route id="WN" edges="E1 -E2" />
        <route id="WS" edges="E1 -E4" />
        
        <route id="NE" edges="E2 -E3" />
        <route id="NW" edges="E2 -E1" />
        <route id="NS" edges="E2 -E4" />
        
        <route id="EW" edges="E3 -E1" />
        <route id="EN" edges="E3 -E2" />
        <route id="ES" edges="E3 -E4" />
        
        <route id="SE" edges="E4 -E3" />
        <route id="SN" edges="E4 -E2" />
        <route id="SW" edges="E4 -E1" />""", file=routes)

        vehNr = 0
        for i in range(N):
            #straight routes
            if random.uniform(0, 1) < p[1][scenario]:
                print('    <vehicle id="WE_%i" type="typeNormal" route="WE" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p[7][scenario]:
                print('    <vehicle id="EW_%i" type="typeNormal" route="EW" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p[4][scenario]:
                print('    <vehicle id="NS_%i" type="typeNormal" route="NS" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p[10][scenario]:
                print('    <vehicle id="SN_%i" type="typeNormal" route="SN" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1

            # right routes
            if random.uniform(0, 1) < p[2][scenario]:
                print('    <vehicle id="WS_%i" type="typeNormal" route="WS" depart="%i" color="1,0,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p[11][scenario]:
                print('    <vehicle id="SE_%i" type="typeNormal" route="SE" depart="%i" color="1,0,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p[8][scenario]:
                print('    <vehicle id="EN_%i" type="typeNormal" route="EN" depart="%i" color="1,0,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p[5][scenario]:
                print('    <vehicle id="NW_%i" type="typeNormal" route="NW" depart="%i" color="1,0,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1

            # left routes
            if random.uniform(0, 1) < p[0][scenario]:
                print('    <vehicle id="WN_%i" type="typeNormal" route="WN" depart="%i" color="0,1,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p[3][scenario]:
                print('    <vehicle id="NE_%i" type="typeNormal" route="NE" depart="%i" color="0,1,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p[6][scenario]:
                print('    <vehicle id="ES_%i" type="typeNormal" route="ES" depart="%i" color="0,1,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p[9][scenario]:
                print('    <vehicle id="SW_%i" type="typeNormal" route="SW" depart="%i" color="0,1,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1

        print("</routes>", file=routes)

# The program looks like this
#    <tlLogic id="TL1" type="static" programID="0" offset="0">
#         <phase duration="33" state="GGgrrrGGgrrr"/>
#         <phase duration="3"  state="yygrrryygrrr"/>
#         <phase duration="6"  state="rrGrrrrrGrrr"/>
#         <phase duration="3"  state="rryrrrrryrrr"/>
#         <phase duration="33" state="rrrgGGrrrGGg"/>
#         <phase duration="3"  state="rrryygrrryyg"/>
#         <phase duration="6"  state="rrrrrGrrrrrG"/>
#         <phase duration="3"  state="rrrrryrrrrry"/>
#     </tlLogic>


def getTrafficSplit(w, n, e, s):
    """compute the Traffic distribution to NS and WE Lanes in comparison"""
    if (w+n+e+s) > 0:
        split = (float(n)+float(s))/(float(w)+float(n)+float(e)+float(s))
    else:
        split = 0.5
    return split


# def run(retQ, useLearning):
#     """execute the TraCI control loop"""
#     step = 0
#     lastArrivalDistance = 0
#
#     # timestamps for the green and yellow phases
#     phaseDuration = 30
#     firstYellowPhase = 3  # step in which the yellow phase which leads into the green phase starts
#     greenPhase = 6
#     secondYellowPhase = 0  # step in which the yellow phase which ends the green phase starts
#
#     incSplit = numpy.zeros((4, 4))  # matrix for the number of cars on every route in the system going to the traffic light
#     #   N E S W   From To Matrix to know all the routes
#     # N 0
#     # E   0
#     # S     0
#     # W       0
#
#     # Q-Learning variables
#     decision = numpy.zeros(12)
#     state = 0
#     action = 0
#     learningRate = 0.5
#     reward = 0
#
#     state_size = 112
#     action_size = 112
#     numpy.random.seed(79)
#     Q = numpy.random.rand(state_size, action_size)
#     Q = Q * 0.5  # q-table get initialized with random value between 0 and 0,5
#
#     if retQ is not None:  # to reuse a q-table which was given to the method
#         Q = retQ
#
#     # we start with phase 2 where NS has green
#     traci.trafficlight.setPhase(trafficLightId, 0)
#     while traci.simulation.getMinExpectedNumber() > 0:
#         traci.simulationStep()
#
#         lst = getTlPhases()  # list that contains every possible greenphase
#
#         idl = traci.vehicle.getIDList()  # list that contains all ids of the cars which are in the simulation at that step
#
#         for i in range(len(idl)):
#             vehIDTmp = idl[i]  # id of the vehicle
#             tlsVehData = traci.vehicle.getNextTLS(vehIDTmp)  # id of the upcoming traffic light the vehicle is heading
#             vehRoadId = traci.vehicle.getRoadID(vehIDTmp)  # id of the road the vehicle is driving on
#             vehRoute = traci.vehicle.getRouteID(vehIDTmp)  # id of the route the vehicle has
#
#             if step == secondYellowPhase:
#                 meanSpeed = (traci.edge.getLastStepMeanSpeed(edgeE1in) + traci.edge.getLastStepMeanSpeed(edgeE2in) + traci.edge.getLastStepMeanSpeed(edgeE3in) + traci.edge.getLastStepMeanSpeed(edgeE4in)) / 4
#                 arrivalDistance = meanSpeed * (phaseDuration + 3)
#
#                 if len(tlsVehData) > 0:  # checks if the vehicle is heading towards the traffic light
#                     distance = float(tlsVehData[0][2])  # distance between the car and the traffic light
#
#                     #compute distance in which every car gets across the trafficlight for the next Greenphase duration
#                     if (vehRoadId == edgeE1in or vehRoadId == edgeE2in or vehRoadId == edgeE3in or vehRoadId == edgeE4in) and distance < arrivalDistance:
#                         incSplit = builtArray(vehRoute, incSplit)
#
#                 # computes the number of cars which passed the traffic light in the last green phase
#                 if (vehRoadId == edgeE1out or vehRoadId == edgeE2out or vehRoadId == edgeE3out or vehRoadId == edgeE4out) and traci.vehicle.getLanePosition(vehIDTmp) < lastArrivalDistance:
#                     reward += 1
#
#                 lastArrivalDistance = arrivalDistance
#
#         if step == secondYellowPhase:
#
#             prevDecision = decision
#
#             # Reward System for updating the q-table
#             reward = reward / (phaseDuration * aimVehiclesPerSecond)
#             newState = action
#
#             if reward != 0:
#                 Q[state, action] = Q[state, action] + learningRate * (reward - Q[state, action])  # update formula for the q-table
#
#             translate = numpy.array([[0, 0, 1, 2], [5, 0, 3, 4], [7, 8, 0, 6], [9, 10, 11, 0]])  # translation array to switch from the incoming 4x4 to the decision array 1x12
#
#             # algorithm to find the ultimate decision regarding every possibility
#             maxIndex = 0
#             maxValue = 0
#
#             for i in range(len(lst)):
#                 tmpValue = 0
#                 for q in range(len(lst[i])):
#                     if lst[i][q] == 1:
#                         if q == 0:
#                             x = 0
#                             y = 1
#                         else:
#                             result = numpy.where(translate == q)
#                             x = result[0][0]
#                             y = result[1][0]
#
#                         tmpIndex = i
#                         qv = Q[newState, i]
#                         if qv != 0.0 and useLearning == 1:  # checks if learning is wanted to be used
#                             tmpValue += 10 * incSplit[x][y] + incSplit[x][y] * qv  # calculates value for every possible greenphase
#                         else:
#                             tmpValue += 10 * incSplit[x][y]  # calculates value for every possible greenphase
#
#                         if tmpValue > maxValue:
#                             maxValue = tmpValue
#                             maxIndex = tmpIndex
#
#             decision = numpy.array(lst[maxIndex])  # the phase with the max value gets taken as decision
#             action = maxIndex
#             state = newState
#             reward = 0
#             secondYellowPhase += phaseDuration + 6
#
#             # changes the decisions for the yellow phase to distinguish between a green phase that has to stay green and a green phase which has to turn yellow
#             for i in range(len(decision)):
#                 if decision[i] == prevDecision[i] and decision[i] == 1:  # greenphase that has to stay green
#                     decision[i] = 3
#                     prevDecision[i] = 3
#                 if prevDecision[i] == 3 and decision[i] == 1:  # greenphase that has to stay green
#                     decision[i] = 3
#                 if prevDecision[i] == 3 and decision[i] == 0:  # greenphase which has to turn yellow
#                     prevDecision[i] = 1
#
#             yellowDec = numpy.where(numpy.array(prevDecision) == 1, 2, numpy.array(prevDecision))  # defines the logic for the second yellow phase which is oriented to the last greenphase
#             traci.trafficlight.setRedYellowGreenState(trafficLightId, translateIntoSumoTlLogic(yellowDec))  # performs the new phase of the traffic light
#
#         if step == firstYellowPhase:
#             yellowDec = numpy.where(numpy.array(decision) == 1, 2, numpy.array(decision))  # defines the logic for the first yellow phase
#             traci.trafficlight.setRedYellowGreenState(trafficLightId, translateIntoSumoTlLogic(yellowDec))  # performs the new phase of the traffic light
#             firstYellowPhase += phaseDuration + 6
#
#         if step == greenPhase:
#             traci.trafficlight.setRedYellowGreenState(trafficLightId, translateIntoSumoTlLogic(decision))  # performs the new phase of the traffic light
#             greenPhase += phaseDuration + 6
#
#         incSplit = numpy.zeros((4, 4))
#
#         step += 1
#     traci.close()
#     sys.stdout.flush()
#     return Q




def runHybrid():
    """execute the TraCI control loop"""
    step = 0

    # timestamps for the green and yellow phases
    phaseDuration = 30
    nmbDurations = 10
    minPhaseDuration = 30
    incr = 2
    firstYellowPhase = 3  # step in which the yellow phase which leads into the green phase starts
    greenPhase = 6
    secondYellowPhase = 0  # step in which the yellow phase which ends the green phase starts


    incSplitList = [numpy.zeros((4, 4)) * i for i in range(nmbDurations)]  # List of matrices for the number of cars on every route in the system going to the traffic light

    ##   N E S W   From To Matrix to know all the routes
    # N 0
    # E   0
    # S     0
    # W       0

    decision = numpy.zeros(12)

    # we start with phase 2 where NS has green
    traci.trafficlight.setPhase(trafficLightId, 0)
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        lst = getHybridTlPhases()  # list that contains every possible greenphas

        idl = traci.vehicle.getIDList()  # list that contains all ids of the cars which are in the simulation at that step

        for i in range(len(idl)):
            vehIDTmp = idl[i]  # id of the vehicle
            tlsVehData = traci.vehicle.getNextTLS(vehIDTmp)  # id of the upcoming traffic light the vehicle is heading
            vehRoadId = traci.vehicle.getRoadID(vehIDTmp)  # id of the road the vehicle is driving on
            vehRoute = traci.vehicle.getRouteID(vehIDTmp)  # id of the route the vehicle has

            if step == secondYellowPhase:
                meanSpeed = (traci.edge.getLastStepMeanSpeed(edgeE1in) + traci.edge.getLastStepMeanSpeed(edgeE2in) + traci.edge.getLastStepMeanSpeed(edgeE3in) + traci.edge.getLastStepMeanSpeed(edgeE4in)) / 4
                meanSpeedWest = traci.edge.getLastStepMeanSpeed(edgeE1in)
                meanSpeedNorth = traci.edge.getLastStepMeanSpeed(edgeE2in)
                meanSpeedEast = traci.edge.getLastStepMeanSpeed(edgeE3in)
                meanSpeedSouth = traci.edge.getLastStepMeanSpeed(edgeE4in)


                arrivalDistanceList = numpy.zeros(nmbDurations)

                for i in range(nmbDurations):
                    arrivalDistanceList[i] = meanSpeed * (minPhaseDuration + (incr * i) + 3)



                if len(tlsVehData) > 0:  # checks if the vehicle is heading towards the traffic light
                    distance = float(tlsVehData[0][2])  # distance between the car and the traffic light


                    for y in range(nmbDurations):
                        if (vehRoadId == edgeE1in or vehRoadId == edgeE2in or vehRoadId == edgeE3in or vehRoadId == edgeE4in) and distance < arrivalDistanceList[y]:
                            incSplitList[y] = builtArray(vehRoute, incSplitList[y])
                    # vehicles within arrivaldistance are grouped after their route and direction
                    # we get a list of the cars which can pass through the traffic light on each track within the next phase
                    # exmpl: 4 6 3 7 2 3 5 8 3 2 5 6

        if step == secondYellowPhase:

            # hier muss die phasendauer bestimmt werden

            prevDecision = decision

            translate = numpy.array([[0, 0, 1, 2], [5, 0, 3, 4], [7, 8, 0, 6], [9, 10, 11, 0]])  # translation array to switch from the incoming 4x4 to the decision array 1x12

            # algorithm to find the ultimate decision regarding every possibility
            maxGreenPhaseComboIndexList = [0] * nmbDurations
            maxValue = [0] * nmbDurations


            for i in range(len(lst)):     # for 112 possible greenphases - rows
                tmpValueList = [0] * nmbDurations

                for q in range(len(lst[0])):    # or 12 different values of each greenphase combinations, exmp: 0;1;0;0;1;0;1;1;1;0;1;0  1 for green - columns

                    if lst[i][q] == 1:    # if a value in the greenphase is green
                        if q == 0:        # if value is red
                            x = 0      # red
                            y = 1      # green
                        else:
                            result = numpy.where(translate == q)
                            x = result[0][0]
                            y = result[1][0]                # 101010101000  greenphase value array translated from 4x4 matrix to 1x12 array

                        tmpIndex = i

                        for j in range(nmbDurations):
                            tmpValueList[j] += incSplitList[j][x][y]  # calculates value for every possible greenphase

                        for k in range(nmbDurations):
                            if tmpValueList[k] > maxValue[k]:     #sorting out best greenphases
                                maxValue[k] = tmpValueList[k]
                                maxGreenPhaseComboIndexList[k] = tmpIndex  # best mögliche grünphasen kombi für jeweilige phasendauer

            for i in range(len(maxValue)):                    # auf einen nenner bringen
                #quot = (minPhaseDuration + 3) + incr * i
                quot = minPhaseDuration + incr * i
                greenPhaseFactor = 1 - (6 / quot)           # percentage of green phases, factor mostly suitable for longer phaseDurations
                                        #gelbphasenfaktor
                #every greenphase has 2 yellowphases, only calculating with the greenphases to determine a more suitable standardized common denominator

                maxValue[i] = float((maxValue[i] / quot) * greenPhaseFactor) #factors such as 0.5* the quot doesnt result favorable tl/car values




            index =0
            tmpMax = float(maxValue[0])         # maximal greenphase combination
            for i in range(len(maxValue)):  # beste von den 5 kombis wird ausgewählt

                if maxValue[i] > tmpMax:
                    index = i  # 4
                    tmpMax = float(maxValue[i])

            for l in range(nmbDurations):      # phasendauer und decision werden angepasst
                if index == l:       # ==4 ?
                    phaseDuration = minPhaseDuration + incr * l # 26 - maxPhasendauer setzung
                    decision = numpy.array(lst[maxGreenPhaseComboIndexList[l]])   # index der decision im 112 array  - maxGreenPhaseCombination setzung

            print(phaseDuration)


            #decision = numpy.array(lst[maxIndex])  # the phase with the max value gets taken as decision
            secondYellowPhase += phaseDuration + 6

            # changes the decisions for the yellow phase to distinguish between a green phase that has to stay green and a green phase which has to turn yellow
            for i in range(len(decision)):
                if decision[i] == prevDecision[i] and decision[i] == 1:  # greenphase that has to stay green
                    decision[i] = 3
                    prevDecision[i] = 3
                if prevDecision[i] == 3 and decision[i] == 1:  # greenphase that has to stay green
                    decision[i] = 3
                if prevDecision[i] == 3 and decision[i] == 0:  # greenphase which has to turn yellow
                    prevDecision[i] = 1

            yellowDec = numpy.where(numpy.array(prevDecision) == 1, 2, numpy.array(prevDecision))  # defines the logic for the second yellow phase which is oriented to the last greenphase
            traci.trafficlight.setRedYellowGreenState(trafficLightId, translateIntoSumoTlLogic(yellowDec))  # performs the new phase of the traffic light

        if step == firstYellowPhase:
            yellowDec = numpy.where(numpy.array(decision) == 1, 2, numpy.array(decision))  # defines the logic for the first yellow phase
            traci.trafficlight.setRedYellowGreenState(trafficLightId, translateIntoSumoTlLogic(yellowDec))  # performs the new phase of the traffic light
            firstYellowPhase += phaseDuration + 6

        if step == greenPhase:
            traci.trafficlight.setRedYellowGreenState(trafficLightId, translateIntoSumoTlLogic(decision))  # performs the new phase of the traffic light
            greenPhase += phaseDuration + 6

        incSplitList = [numpy.zeros((4, 4)) * i for i in range(nmbDurations)]  # incSplitList refreshen

        step += 1
    traci.close()
    sys.stdout.flush()



def runHybridFair():
    """execute the TraCI control loop"""
    step = 0

    # timestamps for the green and yellow phases
    phaseDuration = 30
    nmbDurations = 10
    minPhaseDuration = 30
    incr = 2
    firstYellowPhase = 3  # step in which the yellow phase which leads into the green phase starts
    greenPhase = 6
    secondYellowPhase = 0  # step in which the yellow phase which ends the green phase starts


    incSplitList = [numpy.zeros((4, 4)) * i for i in range(nmbDurations)]  # List of matrices for the number of cars on every route in the system going to the traffic light

    #   N E S W   From To Matrix to know all the routes
    # N 0
    # E   0
    # S     0
    # W       0

    decision = numpy.zeros(12)

    # we start with phase 2 where NS has green
    traci.trafficlight.setPhase(trafficLightId, 0)
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        # wartende linksabbieger mit einbeziehen
        lst = getHybridTlPhases()  # list that contains every possible greenphase

        # diese methode sollte um die wartenden linksabbieger erweitert werden

        idl = traci.vehicle.getIDList()  # list that contains all ids of the cars which are in the simulation at that step

        meanWaitTimeWest = traci.edge.getWaitingTime(edgeE1in)
        meanWaitTimeNorth = traci.edge.getWaitingTime(edgeE2in)
        meanWaitTimeEast = traci.edge.getWaitingTime(edgeE3in)
        meanWaitTimeSouth = traci.edge.getWaitingTime(edgeE4in)
        meanWaitTime = [meanWaitTimeWest, meanWaitTimeNorth, meanWaitTimeEast, meanWaitTimeSouth]
        maxMeanWaitTimeEdge = max(meanWaitTime)
        maxMeanWaitTimeEdgeIndex = meanWaitTime.index(maxMeanWaitTimeEdge)         #index/name of edge that has longest waiting time
        if maxMeanWaitTimeEdgeIndex == 0:
            maxWaitEdge = edgeE1in
        if maxMeanWaitTimeEdgeIndex == 1:
            maxWaitEdge = edgeE2in
        if maxMeanWaitTimeEdgeIndex == 2:
            maxWaitEdge = edgeE3in
        if maxMeanWaitTimeEdgeIndex == 3:
            maxWaitEdge = edgeE4in





        for i in range(len(idl)):
            vehIDTmp = idl[i]  # id of the vehicle
            tlsVehData = traci.vehicle.getNextTLS(vehIDTmp)  # id of the upcoming traffic light the vehicle is heading
            vehRoadId = traci.vehicle.getRoadID(vehIDTmp)  # id of the road the vehicle is driving on
            vehRoute = traci.vehicle.getRouteID(vehIDTmp)  # id of the route the vehicle has

            if step == secondYellowPhase:
                meanSpeed = (traci.edge.getLastStepMeanSpeed(edgeE1in) + traci.edge.getLastStepMeanSpeed(edgeE2in) + traci.edge.getLastStepMeanSpeed(edgeE3in) + traci.edge.getLastStepMeanSpeed(edgeE4in)) / 4
                meanSpeedWest = traci.edge.getLastStepMeanSpeed(edgeE1in)
                meanSpeedNorth = traci.edge.getLastStepMeanSpeed(edgeE2in)
                meanSpeedEast = traci.edge.getLastStepMeanSpeed(edgeE3in)
                meanSpeedSouth = traci.edge.getLastStepMeanSpeed(edgeE4in)


                # a = minPhaseDuration + 3
                # arrivalDistanceList = [meanSpeed * i for i in range(a, (a + nmbDurations) * incr, incr)]


                arrivalDistanceList = numpy.zeros(nmbDurations)

                for i in range(nmbDurations):
                    arrivalDistanceList[i] = meanSpeed * (minPhaseDuration + (incr * i) + 3)


                if len(tlsVehData) > 0:  # checks if the vehicle is heading towards the traffic light
                    distance = float(tlsVehData[0][2])  # distance between the car and the traffic light


                    for y in range(nmbDurations):
                        if (vehRoadId == edgeE1in or vehRoadId == edgeE2in or vehRoadId == edgeE3in or vehRoadId == edgeE4in and vehRoadId == maxWaitEdge) and distance < arrivalDistanceList[y]:
                            incSplitList[y] = builtArray(vehRoute, incSplitList[y])
                    # vehicles within arrivaldistance are grouped after their route and direction
                    # we get a list of the cars which can pass through the traffic light on each track within the next phase
                    # exmpl: 4 6 3 7 2 3 5 8 3 2 5 6

        if step == secondYellowPhase:

            # hier muss die phasendauer bestimmt werden

            prevDecision = decision

            translate = numpy.array([[0, 0, 1, 2], [5, 0, 3, 4], [7, 8, 0, 6], [9, 10, 11, 0]])  # translation array to switch from the incoming 4x4 to the decision array 1x12

            # algorithm to find the ultimate decision regarding every possibility
            maxGreenPhaseComboIndexList = [0] * nmbDurations
            maxValue = [0] * nmbDurations


            #wartende linksabbieger potenziell mit einbeziehen


            for i in range(len(lst)):     # for 112 possible greenphases - rows
                tmpValueList = [0] * nmbDurations

                for q in range(len(lst[0])):    # or 12 different values of each greenphase combinations, exmp: 0;1;0;0;1;0;1;1;1;0;1;0  1 for green - columns

                    if lst[i][q] == 1:    # if a value in the greenphase is green
                        if q == 0:        # if value is red
                            x = 0      # red
                            y = 1      # green
                        else:
                            result = numpy.where(translate == q)
                            x = result[0][0]
                            y = result[1][0]                # 101010101000  greenphase value array translated from 4x4 matric to 1x12 array

                        tmpIndex = i

                        for j in range(nmbDurations):
                            tmpValueList[j] += incSplitList[j][x][y]  # calculates value for every possible greenphase

                        for k in range(nmbDurations):
                            if tmpValueList[k] > maxValue[k]:     #sorting out best greenphases
                                maxValue[k] = tmpValueList[k]
                                maxGreenPhaseComboIndexList[k] = tmpIndex  # best mögliche grünphasen kombi für jeweilige phasendauer

            for i in range(len(maxValue)):                    # auf einen nenner bringen
                #quot = (minPhaseDuration + 3) + incr * i
                quot = minPhaseDuration + incr * i
                greenPhaseFactor = 1 - (6 / quot)           # percentage of green phases, factor mostly suitable for longer phaseDurations
                                        #gelbphasenfaktor
                #every greenphase has 2 yellowphases, only calculating with the greenphases to determine a more suitable standardized common denominator

                maxValue[i] = float((maxValue[i] / quot) * greenPhaseFactor) #factors such as 0.5* the quot doesnt result favorable tl/car values



            index = 0
            tmpMax = float(maxValue[0])         # maximal greenphase combination
            for i in range(len(maxValue)):  # beste von den 5 kombis wird ausgewählt

                if maxValue[i] > tmpMax:
                    index = i  # 4
                    tmpMax = float(maxValue[i])
            #[27,25,20,30,21]
                      #26s Phasendauer wird ausgewählt
            for l in range(nmbDurations):      # phasendauer und decision werden angepasst
                if index == l:       # ==4 ?
                    phaseDuration = minPhaseDuration + incr * l # 26 - maxPhasendauer setzung
                    decision = numpy.array(lst[maxGreenPhaseComboIndexList[l]])   # index der decision im 112 array  - maxGreenPhaseCombination setzung

            #print(phaseDuration)


            #decision = numpy.array(lst[maxIndex])  # the phase with the max value gets taken as decision
            secondYellowPhase += phaseDuration + 6

            # changes the decisions for the yellow phase to distinguish between a green phase that has to stay green and a green phase which has to turn yellow
            for i in range(len(decision)):
                if decision[i] == prevDecision[i] and decision[i] == 1:  # greenphase that has to stay green
                    decision[i] = 3
                    prevDecision[i] = 3
                if prevDecision[i] == 3 and decision[i] == 1:  # greenphase that has to stay green
                    decision[i] = 3
                if prevDecision[i] == 3 and decision[i] == 0:  # greenphase which has to turn yellow
                    prevDecision[i] = 1

            yellowDec = numpy.where(numpy.array(prevDecision) == 1, 2, numpy.array(prevDecision))  # defines the logic for the second yellow phase which is oriented to the last greenphase
            traci.trafficlight.setRedYellowGreenState(trafficLightId, translateIntoSumoTlLogic(yellowDec))  # performs the new phase of the traffic light

        if step == firstYellowPhase:
            yellowDec = numpy.where(numpy.array(decision) == 1, 2, numpy.array(decision))  # defines the logic for the first yellow phase
            traci.trafficlight.setRedYellowGreenState(trafficLightId, translateIntoSumoTlLogic(yellowDec))  # performs the new phase of the traffic light
            firstYellowPhase += phaseDuration + 6

        if step == greenPhase:
            traci.trafficlight.setRedYellowGreenState(trafficLightId, translateIntoSumoTlLogic(decision))  # performs the new phase of the traffic light
            greenPhase += phaseDuration + 6

        incSplitList = [numpy.zeros((4, 4)) * i for i in range(nmbDurations)]  # incSplitList refreshen

        step += 1
    traci.close()
    sys.stdout.flush()

def getTlPhases():
    lst = list(itertools.product([0, 1], repeat=12))  # List with all possible phases of the traffic light -> contains phases which arent possible
    delete = []  # list in which every phase gets saved that is not possible

    # following the contradicting statements are defined to prevent cars crashing into each other
    contra = numpy.zeros((12, 6))
    contra[0] = [3, 4, 7, 8, 9, 10]
    contra[1] = [3, 4, 6, 9, 10, 11]
    contra[2] = [4, 6, 100, 100, 100, 100]
    contra[3] = [0, 1, 7, 6, 10, 11]
    contra[4] = [0, 1, 2, 7, 6, 9]
    contra[5] = [7, 9, 100, 100, 100, 100]
    contra[6] = [1, 3, 4, 9, 10, 2]
    contra[7] = [0, 5, 3, 4, 9, 10]
    contra[8] = [0, 10, 100, 100, 100, 100]
    contra[9] = [0, 1, 4, 7, 6, 5]
    contra[10] = [0, 1, 3, 7, 8, 6]
    contra[11] = [1, 3, 100, 100, 100, 100]

    # algorithm that removes all phases that aren't possible
    for i in range(len(lst)):
        b = 0
        for q in range(len(lst[i])):
            if lst[i][q] == 1:
                for p in range(len(contra[q])):
                    if contra[q][p] < 12:
                        if lst[i][int(contra[q][p])] == 1:
                            # lst.remove(lst[i])
                            delete.append(lst[i])
                            b = 1
                            break
                if b == 1:
                    break

    for i in range(len(delete)):
        lst.remove(delete[i])

    return lst  # returns the list which contains all possible phases (112 instances)

def getHybridTlPhases():
    lst = list(itertools.product([0, 1], repeat=12))  # List with all possible phases of the traffic light -> contains phases which arent possible
    delete = []  # list in which every phase gets saved that is not possible

    #waitingleftlist     010001100001 --> 010005100001

    # following the contradicting statements are defined to prevent cars crashing into each other
    contra = numpy.zeros((12, 6))
    contra[0] = [3, 4, 7, 8, 9, 10]  #links
    contra[1] = [3, 4, 6, 9, 10, 11]
    contra[2] = [4, 6, 100, 100, 100, 100]
    contra[3] = [0, 1, 7, 6, 10, 11]  #links
    contra[4] = [0, 1, 2, 7, 6, 9]
    contra[5] = [7, 9, 100, 100, 100, 100]
    contra[6] = [1, 3, 4, 9, 10, 2]  #links
    contra[7] = [0, 5, 3, 4, 9, 10]
    contra[8] = [0, 10, 100, 100, 100, 100]
    contra[9] = [0, 1, 4, 7, 6, 5]  #links
    contra[10] = [0, 1, 3, 7, 8, 6]
    contra[11] = [1, 3, 100, 100, 100, 100]

    # algorithm that removes all phases that aren't possible
    for i in range(len(lst)):
        b = 0
        for q in range(len(lst[i])):
            if lst[i][q] == 1:
                for p in range(len(contra[q])):
                    if contra[q][p] < 12:
                        if lst[i][int(contra[q][p])] == 1:
                            # lst.remove(lst[i])
                            delete.append(lst[i])
                            b = 1
                            break
                if b == 1:
                    break

    for i in range(len(delete)):
        lst.remove(delete[i])

    return lst  # returns the list which contains all possible phases (112 instances)


def translateIntoSumoTlLogic(logicPy):
    # method to translate the intuitive decision array into the finished sumo traffic light phase description
    logicSumo = numpy.zeros((12))
    sumoPhase = ""
    translate = [2, 1, 0, 5, 4, 3, 8, 7, 6, 11, 10, 9]  # translates the decision array into the right order as parameter for sumo/traci

    for i in range(len(logicPy)):
        if logicPy[i] == 1:
            logicSumo[translate[i]] = 1
        if logicPy[i] == 2:
            logicSumo[translate[i]] = 2
        if logicPy[i] == 3:
            logicSumo[translate[i]] = 3

    for i in range(len(logicSumo)):
        if logicSumo[i] == 1 or logicSumo[i] == 3:
            sumoPhase += "G"
        if logicSumo[i] == 0:
            sumoPhase += "r"
        if logicSumo[i] == 2:
            sumoPhase += "y"

    return sumoPhase

#
# def firstDynamicSolution():
#     """execute the TraCI control loop"""
#     step = 0  # variable for iterating through the simulation
#     tlTimestamp = 90  # timestamp for the first control point of the trafficlight
#
#     # we start with phase 2 where NS has green
#     traci.trafficlight.setPhase(trafficLightId, 0)
#     while traci.simulation.getMinExpectedNumber() > 0:
#         traci.simulationStep()  # performs one simulation step at a time
#
#         idl = traci.vehicle.getIDList()  # get a list with every id of every car which is in the simulation at that time
#         countN = 0
#         countE = 0
#         countS = 0
#         countW = 0
#
#         for i in range(len(idl)):  # iterates through the id list
#             vehIDTmp = idl[i]
#             vehRoadId = traci.vehicle.getRoadID(vehIDTmp)  # get the road the vehicle the vehicle is driving on
#
#             if vehRoadId == edgeE1in:  # western road which is heading to the traffic light
#                 countW += 1
#             if vehRoadId == edgeE2in:  # northern road which is heading to the traffic light
#                 countN += 1
#             if vehRoadId == edgeE3in:  # eastern road which is heading to the traffic light
#                 countE += 1
#             if vehRoadId == edgeE4in:  # southern road which is heading to the traffic light
#                 countS += 1
#
#         tlSplitTmp = getTrafficSplit(countW, countN, countE, countS)  # returns NS lane percentage of general Traffic
#         tlPhase = traci.trafficlight.getPhase(trafficLightId)  # get the phase of the traffic light
#
#         if step == tlTimestamp:  # waits till the step in which the traffic light has to change is reached
#             if tlPhase == 0:  # phase where vertical axle has green
#                 tmp = int(66 * tlSplitTmp)  # calculates the green time of the phase
#                 traci.trafficlight.setPhaseDuration(trafficLightId, tmp)  # sets the duration of the Phase
#                 tlTimestamp = step + tmp + 12 + 1  # sets the next step in which the duration has to be manipulated
#             if tlPhase == 4:  # phase where horizontal axle has green
#                 tmp = int(66 * (1-tlSplitTmp))  # calculates the green time of the phase
#                 traci.trafficlight.setPhaseDuration(trafficLightId, tmp)  # sets the duration of the Phase
#                 tlTimestamp = step + tmp + 12 + 1  # sets the next step in which the duration has to be manipulated
#
#         step += 1
#     traci.close()
#     sys.stdout.flush()
#

def builtArray(vehRoute, Split):
    # method which sorts a vehicle instance into an array by using its route

    if vehRoute == "NE":
        Split[0][1] += 1
    if vehRoute == "NS":
        Split[0][2] += 1
    if vehRoute == "NW":
        Split[0][3] += 1

    if vehRoute == "EN":
        Split[1][0] += 1
    if vehRoute == "ES":
        Split[1][2] += 1
    if vehRoute == "EW":
        Split[1][3] += 1

    if vehRoute == "SN":
        Split[2][0] += 1
    if vehRoute == "SE":
        Split[2][1] += 1
    if vehRoute == "SW":
        Split[2][3] += 1

    if vehRoute == "WN":
        Split[3][0] += 1
    if vehRoute == "WE":
        Split[3][1] += 1
    if vehRoute == "WS":
        Split[3][2] += 1

    return Split  # returns the new array with the new car added


# def standardTrafficLight():
#     """execute the TraCI control loop"""
#     step = 0
#     # we start with phase 2 where NS has green
#     traci.trafficlight.setPhase(trafficLightId, 0)
#     while traci.simulation.getMinExpectedNumber() > 0:
#         traci.simulationStep()
#         # performs the whole simulation without manipulating anything -> standard traffic light
#         step += 1
#     traci.close()
#     sys.stdout.flush()


def computeWaitingTime(name):
    # computes the cumulated waiting time of all cars after the simulation
    tree = ET.parse('tripinfo.xml')
    root = tree.getroot()

    wtData = []
    tlData = []

    count = 0
    cumWt = 0
    cumTl = 0

    for tripinfo in root.findall('tripinfo'):  # gets the needed values from the tripinfo xml file
        wt = tripinfo.get('waitingTime')
        wtData.append(wt)
        cumWt += float(wt)

        tl = tripinfo.get('timeLoss')
        tlData.append(tl)
        cumTl += float(tl)

        count += 1

    wtData = numpy.array(wtData).astype(numpy.float)
    tlData = numpy.array(tlData).astype(numpy.float)

    path = 'Plots/' + name + str(time.time()) + '.png'

    testfile = open("simLog.txt", "a")
    testfile.write(path + ": " + "WaitingTime: " + str(cumWt) + " Wt/Car: " + str(float(cumWt/count)) + "; TimeLoss: " + str(cumTl) + " Tl/Car: " + str(float(cumTl/count)) + " TotalAmountOfCars: " + str(count) + "\n")
    testfile.close()
    print(count, cumWt, float(cumWt/count), cumTl, float(cumTl/count))

    return wtData, tlData


def savePlot(wt, tl, name, x):  # method to create and save the boxplots used in the evaluation
    fig1, ax = plt.subplots()
    ax.set_title('Waiting Time')
    ax.set_xlabel(x)
    ax.set_ylabel('Zeit in Sekunden pro Fahrzeug')
    ax.boxplot(wt, showfliers=False, showmeans=True)
    path = 'Plots/' + name + "_wt_" + str(time.time()) + '.png'
    plt.savefig(path)

    fig2, ax = plt.subplots()
    ax.set_title('Time Loss')
    ax.set_xlabel(x)
    ax.set_ylabel('Zeit in Sekunden pro Fahrzeug')
    ax.boxplot(tl, showfliers=False, showmeans=True)
    path = 'Plots/' + name + "_tl_" + str(time.time()) + '.png'
    plt.savefig(path)

    for i in range(len(tl)):
        get_summary_statistics(numpy.array(tl[i]), path, i)


def get_summary_statistics(dataset, path, axis):  # sums up the data which is presented in the boxplots
    mean = numpy.round(numpy.mean(dataset), 2)
    median = numpy.round(numpy.median(dataset), 2)
    min_value = numpy.round(dataset.min(), 2)
    max_value = numpy.round(dataset.max(), 2)
    quartile_1 = numpy.round(numpy.quantile(dataset, 0.25), 2)
    quartile_3 = numpy.round(numpy.quantile(dataset, 0.75), 2)
    # Interquartile range
    iqr = numpy.round(quartile_3 - quartile_1, 2)
    print('Min: %s' % min_value)
    print('Mean: %s' % mean)
    print('Max: %s' % max_value)
    print('25th percentile: %s' % quartile_1)
    print('Median: %s' % median)
    print('75th percentile: %s' % quartile_3)
    print('Interquartile range (IQR): %s' % iqr)

    logFile = open("boxplotlog.txt", "a")
    logFile.write(path + "__" + str(axis) + "__::  " + " Min: " + str(min_value) + " Mean: " + str(mean) + " Max: " + str(max_value) + " 25th percentile: " + str(quartile_1) + " Median: " + str(median) + " 75th percentile: " + str(quartile_3) + " Interquartile range (IQR): " + str(iqr) + "\n")
    logFile.close()


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


def checkQTableUse():  # method for evaluation of the used state-action pairs over a simulation time of 24hrs
    state_size = 112
    action_size = 112
    numpy.random.seed(79)
    Q = numpy.random.rand(state_size, action_size)
    Q = Q * 0.5  # q-table get initialized with random value between 0 and 0,5

    with open('randomQ_day_sz3.pickle', 'rb') as f:
        Q3 = numpy.array(pickle.load(f))
    equalValues3 = numpy.sum(Q == Q3)
    usedValues3 = (112 * 112) - equalValues3
    print("Szenario 3 Number of used Q-Values: " + str(usedValues3))

    with open('randomQ_day_sz4.pickle', 'rb') as g:
        Q4 = numpy.array(pickle.load(g))
    equalValues4 = numpy.sum(Q == Q4)
    usedValues4 = (112 * 112) - equalValues4
    print("Szenario 4 Number of used Q-Values: " + str(usedValues4))

    with open('randomQ_day_sz5.pickle', 'rb') as h:
        Q5 = numpy.array(pickle.load(h))
    equalValues5 = numpy.sum(Q == Q5)
    usedValues5 = (112 * 112) - equalValues5
    print("Szenario 5 Number of used Q-Values: " + str(usedValues5))


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # checkQTableUse()

    # simulation time in which cars are spawned has to be defined manually
    n = 900  # number of seconds in which cars are spawned in the simulation

    # 0 -> 1 Morning-Day-Rush
    # 1 -> 2 Low-High
    # 2 -> 3 High-Low
    # 3 -> 4 Hor-Ver
    # 4 -> 5 123
    # 5 -> 6 Balanced

    # szenario to be used has to be defined manually
    scenario = 0  # 0 to 5 to specify the traffic scenario used in the following simulations
    generate_routefile(n, scenario)

    # variables that gather the data for the first three systems for a boxplot
    wtOne = []
    tlOne = []

    # # this is the normal way of using traci. sumo is started as a
    # # subprocess and then the python script connects and runs
    # traci.start([sumoBinary, "-c", "data/tlcs.sumocfg",
    #              "--tripinfo-output", "tripinfo.xml"])
    #
    # standardTrafficLight()
    # print("Standard Traffic Light:")
    # standardReturn = computeWaitingTime("SZ_" + str(scenario) + "_Standard_" + str(n) + "_")
    # wtOne.append(standardReturn[0])
    # tlOne.append(standardReturn[1])
    # #
    # # # this is the normal way of using traci. sumo is started as a
    # # # subprocess and then the python script connects and runs
    # traci.start([sumoBinary, "-c", "data/tlcs.sumocfg",
    #              "--tripinfo-output", "tripinfo.xml"])
    #
    # firstDynamicSolution()
    # print("First dynamic solution:")
    # firstDynReturn = computeWaitingTime("SZ_" + str(scenario) + "_FirstDyn_" + str(n) + "_")
    # wtOne.append(firstDynReturn[0])
    # tlOne.append(firstDynReturn[1])
    # #
    # # this is the normal way of using traci. sumo is started as a
    # # subprocess and then the python script connects and runs
    # traci.start([sumoBinary, "-c", "data/tlcs.sumocfg",
    #              "--tripinfo-output", "tripinfo.xml"])

    #
    # run(None, 0)
    # print("Full dynamic without Q-Learning:")
    # fullDynReturn = computeWaitingTime("SZ_" + str(scenario) + "_FullDyn_" + str(n) + "_")
    # wtOne.append(fullDynReturn[0])
    # tlOne.append(fullDynReturn[1])
    ##
    #
    # # plot for the first three traffic-light-logics are created
    # savePlot(wtOne, tlOne, "SZ_" + str(scenario) + "_First3BP_" + str(n) + "_", "Ansatz")
    #
    # Q = None
    #
    # # following statement can be commented in to use one of the 24 hour q-tables
    # # with open('randomQ_day_sz2.pickle', 'rb') as f:
    # #     Q = pickle.load(f)
    #
    # reuseQ = 4
    # # variables which sum up the data of q-learning over the episodes to create a boxplot
    # wtData = []
    # tlData = []
    # for i in range(reuseQ):
    #     # this is the normal way of using traci. sumo is started as a
    #     # subprocess and then the python script connects and runs
    #     traci.start([sumoBinary, "-c", "data/tlcs.sumocfg",
    #                  "--tripinfo-output", "tripinfo.xml"])
    #
    #     Q = run(Q, 1)
    #
    #     # following statement can be commented in to create a 24 hour q-table with an explicit name
    #     # with open('randomQ_day_sz.pickle', 'wb') as f:
    #     #     pickle.dump(Q, f)  # save the q table for further runs (used to gather 24 hour q table)
    #
    #     print("QL with reward system (random Q):")
    #     qReturn = computeWaitingTime("SZ_" + str(scenario) + "_QL_RS_" + str(i) + "_" + str(n) + "_")
    #     wtData.append(qReturn[0])
    #     tlData.append(qReturn[1])
    #
    # # plot for the q-learning episodes are created
    # savePlot(wtData, tlData, "SZ_" + str(scenario) + "_QL_24hQTable_" + str(i) + "_" + str(n) + "_", "Episode")

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "data/tlcs.sumocfg",
                 "--tripinfo-output", "tripinfo.xml"])

    runHybrid()
    print("hybrid solution:")
    firstDynReturn = computeWaitingTime("SZ_" + str(scenario) + "_hybrid_" + str(n) + "_")
    wtOne.append(firstDynReturn[0])
    tlOne.append(firstDynReturn[1])

# this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "data/tlcs.sumocfg",
                 "--tripinfo-output", "tripinfo.xml"])

    runHybridFair()
    print("fair hybrid solution:")
    firstDynReturn = computeWaitingTime("SZ_" + str(scenario) + "_hybridFair_" + str(n) + "_")
    wtOne.append(firstDynReturn[0])
    tlOne.append(firstDynReturn[1])