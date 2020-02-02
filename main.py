#classes to encapsulate all the data needed
import numpy
from numpy.matlib import rand, math

from Movecars import MoveCars


class settings:
    def __init__(self, nbrOfNeuronsInEachHiddenLayer, nbrOfOutNodes,unipolarBipolarSelector, dt, timeout, smallXYVariance, num,num2):
        self.nbrOfNeuronsInEachHiddenLayer = nbrOfNeuronsInEachHiddenLayer
        self.nbrOfOutNodes = nbrOfOutNodes
        self.unipolarBipolarSelector = unipolarBipolarSelector # unipolar and bipolar
        self.dt = dt
        self.timeout = timeout
        self.smallXYVariance = smallXYVariance
        self.num = num
        self.num2 = num2

    def set_nbrOfInputNodes_NetworkArch(self,nbrOfInputNodes):
        self.nbrOfInputNodes = nbrOfInputNodes
        self.NetworkArch = [self.nbrOfInputNodes,self.nbrOfNeuronsInEachHiddenLayer, self.nbrOfOutNodes]
        self.nbrOfTimeStepsToTimeout = self.timeout / self.dt

    def collison_value(self):
        self.collision_distance = int(input())
        if (self.collision_distance <0):
            while self.collision_distance <0:
                self.collision_distance = int(input())


class  GenticAlgorithmSettings:
    def __init__(self, nbrOfGenerations_max,goodFitness,populationSize, corssoverProb_mean_percent, corssoverProb_stdDev_percent,mutationProb,selection_option,tournament_size,
                 truncation_percentage,replacement_option,PercentBestParentsToKeep,keptParentsAreGolobal_option,weightsRange,chromosomeLength ):
        self.nbrOfGenerations_max = nbrOfGenerations_max
        self.goodFitness = goodFitness
        self.populationSize = populationSize
        self.corssoverProb_mean_percent = corssoverProb_mean_percent
        self.corssoverProb_stdDev_percent = corssoverProb_stdDev_percent
        self.mutationProb = mutationProb
        self.selection_option = selection_option  # Tournament or  Truncation
        self.tournament_size = tournament_size
        self.truncation_percentage = truncation_percentage
        self.replacement_option = replacement_option
        self.PercentBestParentsToKeep = PercentBestParentsToKeep
        self.keptParentsAreGolobal_option = keptParentsAreGolobal_option
        self.weightsRange = weightsRange
        self.chromosomeLength = chromosomeLength

class CarSettings:
    def __init__(self,wheelBase, width,length,wheelLength,wheelWidth, speed):
        self.wheelBase = wheelBase
        self.width = width
        self.length = length
        self.wheelLength = wheelLength
        self.wheelWidth = wheelWidth
        self.speed = speed

class SensorSettings:
    def __init__(self,angles, range, sensor_dot_radius_ratio):
        self.angles = angles
        self.range = range
        self.sensor_dot_radius_ratio = sensor_dot_radius_ratio

class EnvSettings:
    def __init__(self, nbrOfCars, dx_dy, intial_point, start_points,destination_dot_radius_ratio ):
        self.nbrOfCars = nbrOfCars
        self.dx_dy = dx_dy
        self.intial_point = intial_point
        self.start_points = start_points
        self.start_headings = numpy.array((180 * rand(1, self.nbrOfCars) - 90) * math.pi / 180).reshape(-1,).tolist()[0]
        self.start_steerAngles = [0] # is it a bunch of zeros ?????????????????????????????
        self.destination_dot_radius_ratio = destination_dot_radius_ratio


#object decelerations and variables
settings_obj = settings(3,2,0,0.1,5,math.pow(8,2),100,10)
GA = GenticAlgorithmSettings( 100,2000,5,95, 5, 0.10, 0, 10,40,0,10, 1,1,0)
car = CarSettings(2.6, 2.5,4.3,0.45,0.22,10)

angles = [0,10,20,30 ,40 ,50 ,60 ,70 ,80 ,90 ,100 ,110, 120, 130, 140, 150, 160, 170, 180]
for i in range(len(angles)):
    angles[i] = angles[i] * math.pi / 180
sensor = SensorSettings(angles,25,0.05)
env = EnvSettings(1,[settings_obj.num, settings_obj.num, - settings_obj.num, - settings_obj.num], [0,0],[20,30],1)

settings_obj.set_nbrOfInputNodes_NetworkArch(len(sensor.angles))

previousNbrOfNeurons = settings_obj.NetworkArch[0]
for i in range(1,len(settings_obj.NetworkArch)): #length of chromosome based on Network Architecture - number of nodes and biases
    GA.chromosomeLength = GA.chromosomeLength + (previousNbrOfNeurons + 1) * settings_obj.NetworkArch[i]
    previousNbrOfNeurons = settings_obj.NetworkArch[i]

Chromosomes = [] #population size  * chromosome length
# for i in range(GA.populationSize):
#     l = []
#     for j in range(GA.chromosomeLength):
#         l.append(0)
#     Chromosomes.append(l)

Chromosomes_Fitness= [] #population size
for i in range(GA.populationSize):
    Chromosomes_Fitness.append(0)

BestFitness_perGeneration = [] #number of max generations
AvgFitness_perGeneration = [] #number of max generations
for i in range(GA.nbrOfGenerations_max):
    BestFitness_perGeneration.append(-1) #intialize with -1
    AvgFitness_perGeneration.append(-1)

for pop in range(GA.populationSize):
    l = numpy.array(GA.weightsRange * (2 * rand(1, GA.chromosomeLength) - 1)).reshape(-1,).tolist()# -1<value<1
    Chromosomes.append(l)

settings_obj.collison_value()

MoveCars(env, settings_obj.nbrOfTimeStepsToTimeout, GA, settings_obj.dt,sensor, car, settings_obj.num,
         settings_obj.smallXYVariance, Chromosomes_Fitness, Chromosomes, settings_obj.NetworkArch, settings_obj.unipolarBipolarSelector, settings_obj.collision_distance)

