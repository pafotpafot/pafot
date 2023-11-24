import random
import copy
from loguru import logger

import aux


touched_chs = []
npc_top_speed = 90

def mutation(pm, population):
    population = copy.deepcopy(population)
    pop_size = len(population)
    npc_size = len(population['pop_0'])
    time_size = len(population['pop_0']['npc0'])
    new_pops = 0

    for i in range(int(pop_size)):
        current_ind = population['pop_'+str(i)]

        if pm >= random.random():
            for j in range(npc_size):
                npc_index = random.randint(0, npc_size)
                time_index = random.randint(0,time_size-1)
                action_index = random.randint(0,1)
                new_pops += 1
                # print(i,j)

                if action_index == 0:
                    population['pop_'+str(i)]['npc'+str(j)][time_index][0] = random.randrange(0,npc_top_speed)

                elif action_index == 1:
                    current_pos = population['pop_'+str(i)]['npc'+str(j)][time_index][1]
                    population['pop_'+str(i)]['npc'+str(j)][time_index][1] = aux.generate_next_pos(current_pos)

    logger.info("Generated " + str(new_pops) + " mutated scenarios")

    return population


def crossover(pc, popul):
    population = copy.deepcopy(popul)
    pop_size = len(population)
    npc_size = len(population['pop_0'])
    new_pop = {}
    for i in range(int(pop_size)):
        if pc > random.random():
            k = 0
            j = 0
            while k == j:
                k = random.randint(0,pop_size-1)
                j = random.randint(0,pop_size-1)
            pop_k = population['pop_'+str(k)]
            pop_j = population['pop_'+str(j)]
            npc_k = random.randint(0,npc_size-1)
            npc_j = random.randint(0,npc_size-1)

            swap_index = random.randint(0,5)

            temp = pop_k['npc'+str(npc_k)][swap_index]
            pop_k['npc'+str(npc_k)][swap_index] = pop_j['npc'+str(npc_j)][swap_index]
            pop_j['npc'+str(npc_j)][swap_index] = temp


            population.update({'pop_'+str(k):pop_k, 'pop_'+str(j):pop_j})

    logger.info("Generated " + str(len(population)) + " mutated scenarios")
    return population

def top_2(population_top):
    max = -1111
    max_k = '0'
    max2 = -1111
    max2_k = '0'
    for k in population_top.copy().keys():

        if 'fitness' in k:

            if population_top[k]>max:
                max2 = max

                max2_k = max_k
                max = population_top[k]
                max_k = k
                
            elif population_top[k] > max2 and population_top[k] < max:
                max2 = population_top[k]
                max2_k = k
    logger.info("Top 2 individuals: " + str(max_k) + " and " + str(max2_k))
    return['pop_'+str(max_k[-1]), 'pop_'+str(max2_k[-1])]
    
def process(population, pm, pc):
    top2_index = top_2(population)
    new_pops = {}
    i=0
    for key in top2_index:
        new_pops['pop_'+str(i)] = population[key]
        i=+1

    cross_best = crossover(pc, new_pops)

    for keys in cross_best.keys():
        i+=1
        new_pops['pop_'+str(i)] = cross_best[keys]

    mutated_best = mutation(pm, new_pops)
        
    return mutated_best

def top_2_lis(population_top):
    max = -1111
    max_k = '0'
    max2 = -1111
    max2_k = '0'
    for k in population_top.copy().keys():

        if 'fitness' in k:
            # print(k, population_top[k])
            if population_top[k]>max:
                max2 = max
                # print(max2_k)
                max2_k = max_k
                max = population_top[k]
                max_k = k
                # print(type(max_k))
                
            elif population_top[k] > max2 and population_top[k] < max:
                max2 = population_top[k]
                max2_k = k
    logger.info("Top 2 individuals: " + str(max_k) + " and " + str(max2_k))
    return['pop_lis_'+str(max_k[-1]), 'pop_lis_'+str(max2_k[-1])]

def crossover_lis(pc, popul):
    population = copy.deepcopy(popul)
    pop_size = len(population)

    npc_size = len(population['pop_lis_0'])
 
    new_pop = {}
    for i in range(int(pop_size)):
        if pc > random.random():
            k = 0
            j = 0
            while k == j:
                k = random.randint(0,pop_size-1)
                j = random.randint(0,pop_size-1)
            pop_k = population['pop_lis_'+str(k)]
            pop_j = population['pop_lis_'+str(j)]
            npc_k = random.randint(0,npc_size-1)
            npc_j = random.randint(0,npc_size-1)


            swap_index = random.randint(0,5)


            temp = pop_k['npc'+str(npc_k)][swap_index]
            pop_k['npc'+str(npc_k)][swap_index] = pop_j['npc'+str(npc_j)][swap_index]
            pop_j['npc'+str(npc_j)][swap_index] = temp


            population.update({'pop_lis_'+str(k):pop_k, 'pop_lis_'+str(j):pop_j})

    logger.info("Generated " + str(len(population)) + " mutated scenarios in LIS")
    return population


def mutation_lis(pm, population):
    population = copy.deepcopy(population)
    pop_size = len(population)
    npc_size = len(population['pop_lis_0'])
    time_size = len(population['pop_lis_0']['npc0'])
    # print(time_size)
    new_pops = 0

    for i in range(int(pop_size)):
        current_ind = population['pop_lis_'+str(i)]

        if pm >= random.random():
            for j in range(npc_size):
                npc_index = random.randint(0, npc_size)
                time_index = random.randint(0,time_size-1)
                action_index = random.randint(0,1)
                new_pops += 1
                # print(i,j)

                if action_index == 0:
                    population['pop_lis_'+str(i)]['npc'+str(j)][time_index][0] = random.randrange(0,npc_top_speed)

                elif action_index == 1:
                    current_pos = population['pop_lis_'+str(i)]['npc'+str(j)][time_index][1]
                    population['pop_lis_'+str(i)]['npc'+str(j)][time_index][1] = aux.generate_next_pos(current_pos)

    logger.info("Generated " + str(new_pops) + " mutated scenarios in LIS")

    return population
    
def process(population, pm, pc):
    top2_index = top_2(population)
    new_pops = {}
    i=0
    for key in top2_index:
        new_pops['pop_'+str(i)] = population[key]
        i=+1
    cross_best = crossover(pc, new_pops)
    for keys in cross_best.keys():
        i+=1
        new_pops['pop_'+str(i)] = cross_best[keys]
    mutated_best = mutation(pm, new_pops)
        
    return mutated_best

def process_lis(population, pm, pc):
    top2_index = top_2_lis(population)
    new_pops = {}
    i=0
    for key in top2_index:
        new_pops['pop_lis_'+str(i)] = population[key]
        i=+1
    cross_best = crossover_lis(pc, new_pops)
    for keys in cross_best.keys():
        i+=1
        new_pops['pop_lis_'+str(i)] = cross_best[keys]
    mutated_best = mutation_lis(pm, new_pops)
        
    return mutated_best







if __name__ == "__main__":
    pop = {'pop_0': {'npc0': [[0, 3], [64, 2], [26, 1], [89, 2], [81, 3], [59, 4]], 'npc1': [[35, 5], [82, 4], [58, 3], [51, 2], [17, 1], [76, 8]]}, 'fitness_0': 134.69806584611888, 'pop_1': {'npc0': [[5, 8], [69, 7], [20, 8], [4, 7], [1, 8], [71, 7]], 'npc1': [[24, 6], [89, 5], [43, 4], [89, 5], [30, 6], [36, 5]]}, 'fitness_1': 106.75755248612506, 'pop_2': {'npc0': [[27, 3], [66, 2], [66, 1], [89, 2], [70, 1], [45, 2]], 'npc1': [[74, 1], [45, 2], [58, 1], [27, 2], [31, 1], [13, 2]]}, 'fitness_2': 81.56269405204802, 'pop_3': {'npc0': [[49, 3], [19, 2], [79, 3], [35, 2], [9, 1], [36, 2]], 'npc1': [[88, 3], [24, 4], [72, 3], [37, 4], [39, 3], [3, 2]]}, 'fitness_3': 108.8532843546482}
    # Uncomment for testing
    # print(pop)
    # populatione = crossover(1.0, pop)
    # print(populatione)
    # # print(pop.keys())
    # pop_mutaded = mutation(1.0, pop)
    # print(pop_mutaded)

    # print(pop.keys())
    # best = top_2(pop)
    # print(best)

    # new_pops = {}
    # i=0
    # for key in best:
    #     new_pops['pop_'+str(i)] = pop[key]
    #     i=+1
    # # print(new_pops)

    # populatione = crossover(1.0, new_pops)
    # # print(populatione)
    # for keys in populatione.keys():
    #     i+=1
    #     new_pops['pop_'+str(i)] = populatione[keys]
        
    # print(new_pops)

    process(pop)