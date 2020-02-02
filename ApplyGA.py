import random
import numpy
from numpy.matlib import rand


def sub2ind(array_shape, rows, cols):
    ind = rows*array_shape[1] + cols
    ind[ind < 0] = -1
    ind[ind >= array_shape[0]*array_shape[1]] = -1
    return ind

def  ApplyGA(GA, Chromosomes, Chromosomes_Fitness): # Because number of chromosomes are not nessesarly GA.populationSize
    smallerPopulationSize = len(Chromosomes_Fitness) # Should be even number !

    # Selection
    if (GA.selection_option == 0): # Tournament
        T = rand(smallerPopulationSize, GA.tournament_size) * (smallerPopulationSize - 1) + 1# Tournaments(Random from 1 to smallerPopulationSize)
        T = numpy.matrix(T).tolist()
        x =[]
        for k in range(len(T)):
            l =[]
            for i in range(len(T[k])):
                T[k][i] = round(T[k][i])
                l.append(Chromosomes_Fitness[T[k][i]-1])
            x.append(l)
        tmp = (numpy.array(x)).max(1)
        tmp = tmp.tolist()
        idx = []
        for i in range(len(tmp)):
            idx.append(T[i].index(tmp[i]))
         # Index to determine the winners
        WinnersIdx = []          #Winners Indeces
        for i in range(len(idx)):
            WinnersIdx.append(T[i][idx[i]])
    elif(GA.selection_option == 1): # Truncation
        V = sorted(Chromosomes_Fitness, 'descend') # Sort fitness in ascending order
        nbrOfSelections = round(smallerPopulationSize * GA.truncation_percentage / 100) # Number of selected chromosomes
        V = V[1:nbrOfSelections] # Winners Pool
        WinnersIdx = V(round(random.randint(smallerPopulationSize, 1) * (nbrOfSelections - 1) + 1)); # Winners Indeces

    # Crossover
    all_parents = []
    for i in range(len(WinnersIdx)):
        all_parents.append(Chromosomes[WinnersIdx[i]-1])
    x = rand(int(smallerPopulationSize / 2), 1)
    x = x.tolist()
    first_parents = []
    for i in range(len(x)):
        for j in range(len(x[i])):
            x[i][j] = round(x[i][j]* (smallerPopulationSize - 1) + 1)
            first_parents.append(all_parents[x[i][j]]) # Random smallerPopulationSize / 2 Parents
    x = rand(int(smallerPopulationSize / 2), 1)
    x = x.tolist()
    second_parents = []
    for i in range(len(x)):
        for j in range(len(x[i])):
            x[i][j] = round(x[i][j] * (smallerPopulationSize - 1) + 1)
            second_parents.append(all_parents[x[i][j]])  # Random smallerPopulationSize / 2 Parents
    references_matrix = []
    for j in range(int(smallerPopulationSize / 2)):
        l = []
        for i in range(GA.chromosomeLength):
            l.append(0)
        references_matrix.append(l)
    for j in range(int(smallerPopulationSize / 2)):
        for i in range(GA.chromosomeLength):
            references_matrix[j][i]=i# = numpy.ones(smallerPopulationSize / 2, 1) [1:GA.chromosomeLength] # The Reference Matrix
    randNums = []
    x = rand(int(smallerPopulationSize / 2),1).tolist()
    for i in range(len(x)):
        for j in range(len(x[i])):
            randNums.append((GA.corssoverProb_stdDev_percent * GA.chromosomeLength / 100) * round(x[i][j]) + GA.corssoverProb_mean_percent * GA.chromosomeLength / 100)
    #randNums = min(round(randNums), GA.chromosomeLength) # Truncation
    #randNums = max(randNums, 1) # Truncation: Vector of smallerPopulationSize / 2  length  of random numbers in range of 1: GA.chromosomeLength
    x = []
    for i in range(len(randNums)):
        l =[]
        for j in range(GA.chromosomeLength):
            r = 1*round(randNums[i])
            if (r>references_matrix[i][j]):
                l.append()
        x.append(l)

    idx = (randNums * numpy.ones(1, G)) > references_matrix # Binary matrix of selected genes for each parents couple
    Chromosomes_Childs1 = numpy.zeros(numpy.size(first_parents))
    Chromosomes_Childs2 = numpy.zeros(numpy.size(first_parents))
    # Do actual corssover
    Chromosomes_Childs1[idx] = first_parents[idx]
    Chromosomes_Childs1[~idx] = second_parents[~idx]
    Chromosomes_Childs2[idx] = second_parents[idx]
    Chromosomes_Childs2[~idx] = first_parents[~idx]
    Chromosomes_Childs =[Chromosomes_Childs1, Chromosomes_Childs2]

    # Mutation
    idx = random.randint(GA.chromosomeLength, smallerPopulationSize)
    idx = (idx <= GA.mutationProb)                                # Indeces for mutations
    mutedValues = GA.weightsRange * (2 * random.randint([1, sum(sum(idx))]) - 1) # Random mutation values from -1 to 1
    Chromosomes_Childs[idx] = mutedValues # Do actual mutation
    return Chromosomes_Childs