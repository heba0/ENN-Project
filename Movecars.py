import statistics
import math
import numpy

from ResetCarAndLifeTime import ResetCarAndLifeTime
from ApplyGA import ApplyGA
from Feedforward import Feedforward


#  Prerequisites  Chromosomes,  Chromosomes_Fitness
# Outputs  Fitness(standing vector: an element for each car)
# Initializations
def MoveCars(env, nbrOfTimeStepsToTimeout, GA, dt, sensor, car, num, smallXYVariance, Chromosomes_Fitness, Chromosomes, Network_Arch, unipolarBipolarSelector, collison_value):
    carLocations = env.start_points  # Car Initial Location[X, Y] in [Meters]
    carHeadings = env.start_headings  # Car Initial Heading Counter Clock Wise[Degrees]
    steerAngles = env.start_steerAngles  # [Degrees] Counter Clock Wise(Same for all cars)

    timesteps = 1
    Old_Locations = []
    for i in range(int(nbrOfTimeStepsToTimeout) - 1):
        l = []
        for j in range(2):
            l.append(0)
        Old_Locations.append(l)

    Generation_ids = 0
    Chromosome_ids = 1
    LifeTimes = 0  # In number of draw steps(multiple of GA.dt)
    timeStepsDone = 0
    prev_carLines = []
    BestFitnessChromoID = 1
    Car_Finished_Pool = 0
    nbrOfParentsToKeep = math.ceil(GA.PercentBestParentsToKeep * GA.populationSize / 100)

    All_Chromosomes = []
    All_Chromosomes_Fitness = []
    for i in range(GA.populationSize):
        l = []
        for j in range(GA.chromosomeLength):
            l.append(0)
        All_Chromosomes.append(l)
        All_Chromosomes_Fitness.append(l)


    # Iterating Generations
    while (1):
        # Move Car and Draw Environment - Get Sensor Readings and Collision State
        sensor_readings = []
        y = 0
        print("Sensor readings: ")  ###############input sensor readings with angles - spectrum - distance
        for i in range(len(sensor_readings)):
            sensor_readings[i] = int(input())

        sensor_readings= [25,25,25,25,25,25,25,25,25,25,25,25,0,0,25,25,25,25,25]

        dist = min(sensor_readings)
        id = sensor_readings.index(dist)

        collison_bools = False
        if dist<= collison_value:
            collison_bools = True
        else:
            collison_bools = False

        timeStepsDone = timeStepsDone + 1

        # Increase lifetimes by 1
        LifeTimes = LifeTimes + 1


        # Update Fitness
        Fitness = LifeTimes

        # If car is almost in same place after nbrOfTimeStepsToTimeout has passed, set rotating_around_my_self_bool
        rotating_around_my_self_bool = 0
        if (LifeTimes >= nbrOfTimeStepsToTimeout):
            Old_Locations.append(carLocations)
            mean_x = statistics.mean(Old_Locations[:][0])
            mean_y = statistics.mean(Old_Locations[:][1])
            x = Old_Locations[0]
            for i in range(len(x)):
                try:
                    x[i] = math.pow((x[i] - mean_x), 2)
                except OverflowError:
                    x[i] = float('inf')
                var_x = statistics.mean(x)  # numpy.mean(( - mean_x) ^ 2)
                x = Old_Locations[1]
                for i in range(len(x)):
                    try:
                        x[i] = math.pow((x[i] - mean_y), 2)
                    except OverflowError:
                        x[i] = float('inf')
                var_y = statistics.mean(x)

                if var_x <= smallXYVariance and var_y <= smallXYVariance:
                    rotating_around_my_self_bool = 1
        else:
            Old_Locations[LifeTimes-1][0] = carLocations[0]
            Old_Locations[LifeTimes-1][1] = carLocations[1]

        if (collison_bools):
            if (Fitness > max(Chromosomes_Fitness)):
                BestFitnessChromoID = Chromosome_ids  # Save Best Fitness

            Chromosomes_Fitness[Chromosome_ids] = Fitness

            if (Fitness >= GA.goodFitness):
                Car_Finished_Pool = 1
                BestFitnessChromoID = Chromosome_ids

            ResetCarAndLifeTime(carLocations, env, 0, carHeadings, steerAngles, LifeTimes, prev_carLines)

            if (Car_Finished_Pool != 1):
                Chromosome_ids = Chromosome_ids + 1

        elif (rotating_around_my_self_bool == 1):
            All_Chromosomes_Fitness[0][Chromosome_ids] = 0  # TODO Is this good ?
            ResetCarAndLifeTime(carLocations, env, 0, carHeadings, steerAngles, LifeTimes, prev_carLines)

            if (Car_Finished_Pool != 1):
                Chromosome_ids = Chromosome_ids + 1
            rotating_around_my_self_bool = 0

        # Jump to car next Generation if necessary
        if (Chromosome_ids >= GA.populationSize and (Car_Finished_Pool != 1)):
            if (Generation_ids >= GA.nbrOfGenerations_max):
                Car_Finished_Pool = 1
                Chromosome_ids = BestFitnessChromoID
            else:
                # if (GA.replacement_option == 0)
                All_Chromosomes[(i - 1) * GA.populationSize : i * GA.populationSize] = Chromosomes
                x = 0
                for i in range((i - 1) * GA.populationSize,i * GA.populationSize ):
                    All_Chromosomes_Fitness[i][y] = Chromosomes_Fitness[x]
                    x += 1

                y += 1
                tmp = All_Chromosomes_Fitness.copy()
                idx = numpy.argsort(tmp, kind='mergesort', axis=0).tolist()[::-1]
                idx2 = numpy.array(idx).tolist()[0][0:nbrOfParentsToKeep]
                ParentsToKeep = All_Chromosomes[idx2[0]]

                tmp = Chromosomes_Fitness.copy()
                idx = numpy.argsort(tmp, kind='mergesort', axis=0).tolist()[::-1]
                idx2 = numpy.array(idx).tolist()[0:len(idx)-nbrOfParentsToKeep]
                Current_Chromosomes = []
                Current_Fitness = []
                for i in range(len(idx2)):
                    Current_Chromosomes.append(Chromosomes[idx2[i]])
                    Current_Fitness.append(Chromosomes_Fitness[idx2[i]])

                Chromosomes_Childs = ApplyGA(GA, Current_Chromosomes, Current_Fitness)
                Chromosomes = [ParentsToKeep, Chromosomes_Childs]

                Chromosome_ids = 1
                Generation_ids = Generation_ids + 1
                Chromosomes_Fitness = 0 * Chromosomes_Fitness
                BestFitnessChromoID = 1
        current_chromosome = Chromosomes[Chromosome_ids]

        # Apply sensor reading to ANN to calculate steerAngle

        outputs = Feedforward(sensor_readings, current_chromosome, Network_Arch, unipolarBipolarSelector)
        steerAngles = numpy.pi / 2 * (outputs[1] - outputs[0])  # From - 90 to 90 degrees
        frontWheel = []
        backWheel = []
        # 2D car steering physics(Calculate carLocation and carHeading)
        frontWheel.append(float(carLocations[0] + car.wheelBase / 2 * math.cos(carHeadings)))
        frontWheel.append(float(carLocations[1] + car.wheelBase / 2 * math.sin(carHeadings)))
        backWheel.append(float(carLocations[0] - car.wheelBase / 2 * math.cos(carHeadings)))
        backWheel.append(float(carLocations[1] - car.wheelBase / 2 * math.sin(carHeadings)))
        backWheel[0] = backWheel[0] + car.speed * dt * math.cos(carHeadings)
        backWheel[1] = backWheel[1] + car.speed * dt * math.sin(carHeadings)
        frontWheel[0] = frontWheel[0] + car.speed * dt * math.cos(carHeadings + steerAngles)
        frontWheel[1] = frontWheel[1] + car.speed * dt * math.sin(carHeadings + steerAngles)
        for i in range(len(carLocations)):
            carLocations[i] = (frontWheel[i] + backWheel[i]) / 2
        carHeadings = math.atan2(frontWheel[1] - backWheel[1], frontWheel[0] - backWheel[0])

        print("Front Wheel: ", frontWheel)
        print("Back Wheel: ", backWheel)
        print("Steering Angles: ", steerAngles)

