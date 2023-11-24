import carla
import loguru
import time
import math
import random
from datetime import datetime
import liability
import aux
import os
import json
import csv
import copy

logger = loguru.logger

from example_sof import Player
from ego import Ego
import mutator

# Create a folder for saving the files
output_folder = "Output/"+datetime.now().strftime("%Y-%m-%d-%H:%M")
os.mkdir(output_folder)
logger.add(output_folder+'.log')

# Variables
map_name = 'Town06'
map_name_long = "Carla/Maps/Town06"
ego_circle_sens = 20
npc = None
npc_top_speed = 90

# Mutator/GA Parameters
pop_size = 8 
p_mutation = 0.5
p_crossover = 0.5
max_gen = 100

best_scenarios = []
no_progress = False
generation_last_restart = 0
min_generation_lis = 2
collision_stop_sim = False


was_restarted = False
best_ff_after_restart = 0.0
is_lis = False
lis_gens = 4
lis_counter = 0
pop_before_lis = None
aux_pop = []


def calculate_distance(ego, spawn):
    return math.sqrt((math.pow(ego.location.x - spawn.location.x,2) + math.pow(ego.location.y - spawn.location.y,2)))
    
def get_npc_spawns(ego):
    ego_transform = ego.vehicle.get_transform()
    spawns = world.get_map().get_spawn_points()
    npc_spawns = []
    for point in spawns:
        d = calculate_distance(ego_transform, point)
        if d <= 30 and d!= 0.0: 
            npc_spawns.append(point)
    logger.info("Total possible Spawn points: " + str(len(npc_spawns)))
    return npc_spawns

def get_valid_npc_spawns(ego): 
    valid = []
    spawns = get_npc_spawns(ego)
    ego_transform = ego.get_transform()
    for spawn in spawns:
        if (ego_transform.rotation.yaw - 45 <= abs(spawn.rotation.yaw)) and (ego_transform.rotation.yaw + 45 >= abs(spawn.rotation.yaw)):
            valid.append(spawn)
    logger.info("Total valid Spawn points: " + str(len(valid)))
    return valid

def get_npc_quadrant_pos(ego, npc):
    npc_loc = npc.vehicle.get_transform()
    ego_loc = ego.vehicle.get_transform()
    pi = math.pi
    
    angle = math.atan2((npc_loc.location.y - ego_loc.location.y),(npc_loc.location.x - ego_loc.location.x))
    d_npc = calculate_distance(ego_loc, npc_loc)
    
    d_lateral_npc = abs(math.sin(angle) * d_npc)
    d_long_npc = abs(math.cos(angle) * d_npc)
    
    ego_bb = ego.vehicle.bounding_box
    
    if d_npc > ego_circle_sens:
        return 0  

    if pi/2 <= angle and angle <= pi and d_long_npc > ego_bb.extent.x and d_lateral_npc > ego_bb.extent.y:
        return 1
    elif abs(angle) > pi/2 and d_long_npc > ego_bb.extent.x and d_lateral_npc < ego_bb.extent.y:
        return 2
    elif  -pi <= angle and angle < -pi/2 and d_long_npc > ego_bb.extent.x and d_lateral_npc > ego_bb.extent.y:
        return 3
    elif angle < 0 and d_long_npc < ego_bb.extent.x and d_lateral_npc > ego_bb.extent.y: 
        return 4
    elif -pi/2 < angle and angle <= 0 and d_long_npc > ego_bb.extent.x and d_lateral_npc > ego_bb.extent.y:
        return 5
    elif abs(angle) < pi/2 and d_long_npc > ego_bb.extent.x and d_lateral_npc < ego_bb.extent.y:
        return 6
    elif 0 <= angle and angle < pi/2 and d_long_npc > ego_bb.extent.x and d_lateral_npc > ego_bb.extent.y:
        return 7
    elif angle > 0 and d_long_npc < ego_bb.extent.x and d_lateral_npc > ego_bb.extent.y:
        return 8  
    else:
        return 0
    
def get_quadrant(ego, npc):
    npc_loc = npc.get_transform()
    ego_loc = ego.get_transform()
    pi = math.pi
    
    angle = math.atan2((npc_loc.location.y - ego_loc.location.y),(npc_loc.location.x - ego_loc.location.x))
    d_npc = calculate_distance(ego_loc, npc_loc)
    
    d_lateral_npc = abs(math.sin(angle) * d_npc)
    d_long_npc = abs(math.cos(angle) * d_npc)
    
    ego_bb = ego.bounding_box
    
    if d_npc > ego_circle_sens:
        return 0  

    if pi/2 <= angle and angle <= pi and d_long_npc > ego_bb.extent.x and d_lateral_npc > ego_bb.extent.y:
        return 1
    elif abs(angle) > pi/2 and d_long_npc > ego_bb.extent.x and d_lateral_npc < ego_bb.extent.y:
        return 2
    elif  -pi <= angle and angle < -pi/2 and d_long_npc > ego_bb.extent.x and d_lateral_npc > ego_bb.extent.y:
        return 3
    elif angle < 0 and d_long_npc < ego_bb.extent.x and d_lateral_npc > ego_bb.extent.y: 
        return 4
    elif -pi/2 < angle and angle <= 0 and d_long_npc > ego_bb.extent.x and d_lateral_npc > ego_bb.extent.y:
        return 5
    elif abs(angle) < pi/2 and d_long_npc > ego_bb.extent.x and d_lateral_npc < ego_bb.extent.y:
        return 6
    elif 0 <= angle and angle < pi/2 and d_long_npc > ego_bb.extent.x and d_lateral_npc > ego_bb.extent.y:
        return 7
    elif angle > 0 and d_long_npc < ego_bb.extent.x and d_lateral_npc > ego_bb.extent.y:
        return 8  
    else:
        return 0

def world_settings(world):
    # World Settings:
    settings = world.get_settings()
    settings.synchronous_mode = True # Enables synchronous mode
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

def get_spawn_wps(ego, quadrant):
    dist_buff =  8  #TODO change this to be in function of the EV's velocity
    wps = []
    ego_wp = map.get_waypoint(ego.vehicle.get_location())
    if quadrant == 1 or quadrant == "1":
        wp = ego_wp.next(ego.vehicle.bounding_box.extent.x + dist_buff)[0].get_left_lane()
    elif quadrant == 2 or quadrant == "2": 
        wp = ego_wp.next(ego.vehicle.bounding_box.extent.x + dist_buff)[0]
    elif quadrant == 3 or quadrant == "3":
        wp = ego_wp.next(ego.vehicle.bounding_box.extent.x + dist_buff)[0].get_right_lane()
    elif quadrant == 4 or quadrant == "4":
        wp = ego_wp.get_right_lane()
    elif quadrant == 5 or quadrant == "5":
        wp = ego_wp.previous(ego.vehicle.bounding_box.extent.x + dist_buff)[0].get_right_lane()
    elif quadrant == 6 or quadrant == "6":
        wp = ego_wp.previous(ego.vehicle.bounding_box.extent.x + dist_buff)[0]
    elif quadrant == 7 or quadrant == "7":
        wp = ego_wp.previous(ego.vehicle.bounding_box.extent.x + dist_buff)[0].get_left_lane()
    elif quadrant == 8 or quadrant == "8":
        wp = ego_wp.get_left_lane()
        #wps.append(wp)
    else:
        logger.error("The quadrant does not exist")
        return None
    try:
            wps.append(wp)
    except:
            raise Exception 
    return wps


def get_quadrant_wps(ego, quadrant):
    if ego.vehicle.get_velocity().length() > 0.0:
        dist_buff = ego.vehicle.get_velocity().length() *2.0
    else:
        #TODO Check if this works
        dist_buff = ego.vehicle.bounding_box.extent.x * 3
    wps = []
    ego_wp = map.get_waypoint(ego.vehicle.get_location())
    if quadrant == 1 or quadrant == "1":
        wp = ego_wp.next(ego.vehicle.bounding_box.extent.x + dist_buff)[0].get_left_lane()
    elif quadrant == 2 or quadrant == "2": 
        wp = ego_wp.next(ego.vehicle.bounding_box.extent.x + dist_buff)[0]
    elif quadrant == 3 or quadrant == "3":
        wp = ego_wp.next(ego.vehicle.bounding_box.extent.x + dist_buff)[0].get_right_lane()
    elif quadrant == 4 or quadrant == "4":
        wp = ego_wp.get_right_lane()
    elif quadrant == 5 or quadrant == "5":
        wp = ego_wp.previous(ego.vehicle.bounding_box.extent.x + dist_buff)[0].get_right_lane()
    elif quadrant == 6 or quadrant == "6":
        wp = ego_wp.previous(ego.vehicle.bounding_box.extent.x + dist_buff)[0]
    elif quadrant == 7 or quadrant == "7":
        wp = ego_wp.previous(ego.vehicle.bounding_box.extent.x + dist_buff)[0].get_left_lane()
    elif quadrant == 8 or quadrant == "8":
        wp = ego_wp.get_left_lane()

    else:
        logger.error("The quadrant does not exist")
        return None
    try:
            wps.append(wp.next(dist_buff)[0])
    except:
            raise Exception 
    return wps

def npc_is_ahead(ego, npc):
    npc_loc = npc.vehicle.get_transform()
    ego_loc = ego.vehicle.get_transform()
    pi = math.pi
    
    angle = math.atan2((npc_loc.location.y - ego_loc.location.y),(npc_loc.location.x - ego_loc.location.x))
    d_npc = calculate_distance(ego_loc, npc_loc)
    
    d_long_npc = abs(math.cos(angle) * d_npc)
    ego_bb = ego.vehicle.bounding_box
    if ((-pi <= angle and angle < -pi/2) or (pi/2 < angle and angle < pi)) and d_long_npc > ego_bb.extent.x:
        return True
    else:
        return False

def on_collision(args):
    npc_col = args.other_actor
    ego_col = args.actor
    npc_col_quad = get_quadrant(ego_col, npc_col)
    ego_speed = ego.vehicle.get_velocity().length()
    global collision_type
    if ego_speed <= 0.1:
        ego_fault = False
        collision_type = 'NPC'
    elif npc_col_quad == 1 or npc_col_quad == 2 or npc_col_quad == 3: 
        ego_fault = True
        collision_type = 'Ego'
    else:
        ego_fault = False
        collision_type = 'NPC'
    global sim_time
    if sim_time == True:
        logger.info("Collision detected - Ego Fault: " + str(ego_fault))
        time_collision = (datetime.now() - sim_start_time).seconds
        fitness = liability.compute_fitness_collision(ego, npc_col, ego_fault, time_collision)
        global max_fitness
        max_fitness = fitness if fitness > max_fitness else max_fitness
        logger.info("Fitness value: " + str(fitness))
    
    sim_time = False


def get_new_quad(pos):
    pos_positions = [1,2,3,4,5,6,7,8]
    if pos == 1 or pos == 8 or pos == 7:
        pos_positions.remove(1)
        pos_positions.remove(8)
        pos_positions.remove(7)
    elif pos == 3 or pos == 4 or pos == 5:
        pos_positions.remove(3)
        pos_positions.remove(4)
        pos_positions.remove(5)
    return pos_positions

def start_ego():
    ego_spawn = spawns[2]
    bp_library = world.get_blueprint_library()
    ego_bp = bp_library.find('vehicle.lincoln.mkz_2017')
    logger.info("Spawning Ego Vehicle")
    ego = Ego(world, ego_bp, ego_spawn)
    vehicles.append(ego)
    ego.update_spectator()
    ego.vehicle.set_autopilot(True)
    return ego


def start_npcs(number_npcs, ev, sol):

    npc_list = []
    spawns_positions = []
    for i in range(number_npcs):
        if sol['npc'+str(i)][0][1] not in spawns_positions:
            spawns_positions.append(sol['npc'+str(i)][0][1])
        else:
            spawns_positions.append(sol['npc'+str(i)][0][1]+1) if sol['npc'+str(i)][0][1] < 8 else spawns_positions.append(sol['npc'+str(i)][0][1]-1)
            logger.warning("Trying to spawn two NPCs in the same position")
    index = spawns_positions
    npc_spawns_indexs = []
    npc_spawns = get_npc_spawns(ev)
    for i in range(len(npc_spawns)): 
        npc_spawns_indexs.append(i) 

    for npcs in range(number_npcs):
        try:
            index_tr = get_spawn_wps(ego, index[npcs])
            npc = Player(world, npc_bp, index_tr[0].transform)
            npc_list.append(npc)
            world.wait_for_tick() # Same as with spectator, don't know why it's needed
        except Exception as e:
            logger.error(e)
    return  npc_list
    
    
def destroy_vehicles(ego, npcs):
    time.sleep(2)
    ego.destroy()
    for npc in npcs:
        npc.destroy()
    
    

# Create the connection with Carla
logger.info("Starting simulation")
client = carla.Client('localhost', 2000)
client.set_timeout(15)
logger.debug("Connected to Carla Sim")
# Load the map. For now, Town06
world = client.get_world()
world_settings(world)
map = world.get_map()
logger.info("Current map: " +str(world.get_map().name))
client.load_world(map_name)

if map.name == map_name_long:
    print("reloading world0")
    client.reload_world()

else:
    print("loading world")
    client.load_world(map_name)

logger.info("Loaded map " + map.name)

vehicles = []

spawns = world.get_map().get_spawn_points()
bp_library = world.get_blueprint_library()

number_npcs = 2 # Let's do one for now, we'll have to change the code to accommodate more NPCs
npc_bp = bp_library.find('vehicle.nissan.micra') # Fow now same type of vehicle, we can change this later
npcs_spawned = False
logger.info("Spawning {} NPCs", number_npcs)

# Generate a random solution
gen_0 = True
population = {}
for ind in range(pop_size):
    solution = {}
    current__position = random.randint(1,8)
    for i in range(number_npcs):
        s = []
        for j in range(6):
            next_position = aux.generate_next_pos(current__position)
            s.append([random.randrange(0, npc_top_speed), next_position])
            current__position = next_position
        solution.update({"npc"+str(i):s})
    population['pop_'+str(ind)] = solution
logger.info("Generated " + str(pop_size) + "starting solutions")

pop_ff = {}

# Set up the TM in synchronous mode
traffic_manager = client.get_trafficmanager()
traffic_manager.set_synchronous_mode(True)

time_between_genes = 10

csv_headers = ['scenario', 'fitness', 'collision', 'execution_time']
csv_data = []
scenario_counter = 0
sim_time = True
total_sim_time = 60
npc_position_treshold = 2
control_signal = None


for gen in range(max_gen):
    sim_start_time = datetime.now()
    min_distance = 9999
    mettc = 9999
    execution_time = 0.0
    collision = False
    max_fitness = -1111

    gene_executed = 0
    next_gene = 1

    for pop in population.copy().keys():
        target_position = [population[pop]['npc0'][0][1], population[pop]['npc1'][0][1]]
        vel_ref = [population[pop]["npc0"][0][0], population[pop]["npc1"][0][0]]
        logger.info("Moving NPC to " + str(target_position) + " positions with vel_ref " + str(vel_ref))
        # For each scenario
        sim_time = True
        sim_start_time = datetime.now()
        max_fitness = -1111
        collision_type = 'None'
        ego = start_ego()
        col_bp = world.get_blueprint_library().find('sensor.other.collision')
        col_transform = carla.Transform(carla.Location(x=-10, z=10))
        colision_sensor = world.spawn_actor(col_bp, col_transform, attach_to=ego.vehicle)
        colision_sensor.listen(on_collision)
        solution = population[pop]
        npc_list = start_npcs(number_npcs, ego, solution)

        while sim_time == True:
            sim_time_elapsed = datetime.now() - sim_start_time
            #TODO move the solution gene thingy here and calculate mettc with the starting point
            npc_in = 0
            for i in range(len(npc_list)):
                if sim_time_elapsed.seconds == (next_gene * 10):
                    gene_executed = next_gene
                    next_gene += 1
                    for j in range(len(npc_list)):
                        try:
                            target_position[j] = solution["npc"+str(j)][gene_executed][1]
                            vel_ref[j] = solution["npc"+str(j)][gene_executed][0]
                        except:
                            pass
                        logger.info("Moving NPC" + str(j) + " to position " + str(target_position[j]) + " with vel_ref " + str(vel_ref[j]))

                try:
                    waypoints = get_quadrant_wps(ego, target_position[i])

                except Exception as e:
                    target_position[i] = random.choice(get_new_quad(target_position[i]))
                    logger.info("Changing NPC"+ str(i) + "target position to: " + str(target_position[i]))
                    logger.error(str(e))
                # print(target_position[i])
                current_pos = get_npc_quadrant_pos(ego, npc_list[i])
                if current_pos == target_position[i]:
                    # print("lalala")
                    npc_neo_wp =  map.get_waypoint(npc_list[i].vehicle.get_location())
                    try:
                        neo_wp = npc_neo_wp.next(ego.vehicle.get_velocity().length()*2)[0]
                    except Exception as e:
                        logger.error(str(e))
                        try:
                            neo_wp = npc_neo_wp.next(5.0)[0]
                        except:
                            pass
                    control_signal = npc_list[i].controller.run_step(vel_ref[i],neo_wp)
                    npc_list[i].vehicle.apply_control(control_signal)

                elif current_pos != 0:
                    # print("lololo")
                    dist_npc_wp = calculate_distance(npc_list[i].vehicle.get_transform(), waypoints[0].transform)
                    if dist_npc_wp < npc_position_treshold: #Distance between the NPC and the waypoint

                        if (current_pos == 8 and target_position == 1) or (current_pos == 4 and target_position == 3):
                            # Accelerate the NPC to reach the target_position
                            vel_npc = vel_ref[i] * 2.0
                            npc_neo_wp =  map.get_waypoint(npc_list[i].vehicle.get_location())
                            neo_wp = npc_neo_wp.next(npc_list[i].vehicle.get_velocity().length())[0]
                            control_signal = npc_list[i].controller.run_step(vel_npc,neo_wp)
                            npc_list[i].vehicle.apply_control(control_signal)
                        
                        elif (current_pos == 8 and target_position == 7) or (current_pos == 4 and target_position == 5):
                            # Deccelerate NPC to reach target_position
                            npc_neo_wp =  map.get_waypoint(npc_list[i].vehicle.get_location())
                            neo_wp = npc_neo_wp.next(npc_list[i].vehicle.get_velocity().length())[0]
                            vel_npc = vel_ref[i] * 0.5
                            control_signal = npc_list[i].controller.run_step(vel_npc, neo_wp)
                            npc_list[i].vehicle.apply_control(control_signal)
                        else:
                            vel_npc = vel_ref[i]
                            try:
                                neo_wp = waypoints[0].next(npc_list[i].vehicle.get_velocity().length()*2.0)[0]
                            except Exception as e:
                                logger.error(str(e))
                                try:
                                    neo_wp = waypoints[0].next(5.0)[0]
                                except:
                                    pass
                            control_signal = npc_list[i].controller.run_step(vel_npc,neo_wp)
                            npc_list[i].vehicle.apply_control(control_signal)


                    else:
                        vel_npc = 100
                        try:
                            neo_wp = waypoints[0].next(npc_list[i].vehicle.get_velocity().length()*2.0)[0]
                        except Exception as e:
                            logger.error(str(e))
                            try:
                                neo_wp = waypoints[0]
                            except:
                                pass
                        control_signal = npc_list[i].controller.run_step(vel_npc,neo_wp)
                        npc_list[i].vehicle.apply_control(control_signal)

                elif current_pos == 0: # This means the NPC is way far away from the EV position to be considered in the grid
                    # print("lululu")
                    dist_npc_wp = calculate_distance(npc_list[i].vehicle.get_transform(), waypoints[0].transform)
                    vel_npc = vel_ref[i] if dist_npc_wp < npc_position_treshold else 100.0
                    if npc_is_ahead(ego, npc_list[i]):
                        npc_neo_wp =  map.get_waypoint(npc_list[i].vehicle.get_location())
                        try:
                            neo_wp = npc_neo_wp.next(npc_list[i].vehicle.get_velocity().length()*2.0)[0]
                        except Exception as e:
                            logger.error(str(e))
                            try:
                                neo_wp = npc_neo_wp.next(5)[0]
                            except:
                                pass
                        control_signal = npc_list[i].controller.run_step(vel_npc * 0.1 ,neo_wp)
                    else:
                        npc_neo_wp =  map.get_waypoint(npc_list[i].vehicle.get_location())
                        try:
                            neo_wp = npc_neo_wp.next(npc_list[i].vehicle.get_velocity().length()*2.0)[0]
                        except Exception as e:
                            logger.error(str(e))
                            try:
                                neo_wp = npc_neo_wp.next(5)[0]
                            except:
                                pass
                        control_signal = npc_list[i].controller.run_step(vel_npc, neo_wp)

                    npc_list[i].vehicle.apply_control(control_signal)

                ettc = liability.calculate_ettc(ego, npc_list[i])
                mettc = ettc if ettc < mettc else mettc

                fitness = liability.compute_fitness(ego, npc_list[i], sim_time_elapsed.seconds, mettc)
                if fitness > max_fitness:
                    max_fitness = fitness

            ego.update_spectator()

            if sim_time_elapsed.seconds >= total_sim_time:
                sim_time = False
                logger.info("Finishing simulation")


        logger.info("Finishing simulation, Max Fitness: " + str(max_fitness))
        if is_lis: #restart the pop_ff dictionary for LIS
            pop_ff.update({"pop_lis_"+str(pop)[-1]: solution, "fitness_lis_"+str(pop)[-1]: max_fitness})
        else:
            pop_ff.update({"pop_"+str(pop)[-1]: solution, "fitness_"+str(pop)[-1]: max_fitness})
        destroy_vehicles(ego, npc_list)
        colision_sensor.destroy()
        if is_lis:
            csv_data.append([str(gen)+'_lis_'+str(pop), max_fitness, collision_type, sim_time_elapsed.seconds])
        else:
            csv_data.append([str(gen)+'_'+str(pop), max_fitness, collision_type, sim_time_elapsed.seconds])
        scenario_counter +=1

    logger.info("Finished Generation "+ str(gen))


# Save fitness, solutions, and everyting else into something (list?) that can be accessed by the mutator            
    # print("population", pop_ff)
    with open(output_folder+"/gen_"+str(gen)+".txt", "x") as f:
        json.dump(pop_ff, f)
        f.close()

    best_ff = 0.0
    index_best = 0

    for k in range(int(len(pop_ff)/2)):
        print(k, len(pop_ff))
        if is_lis:
            if pop_ff['fitness_lis_'+str(k)] > best_ff:
                best_ff = pop_ff['fitness_lis_'+str(k)]
                index_best = k
        else:
            if pop_ff['fitness_'+str(k)] > best_ff:
                best_ff = pop_ff['fitness_'+str(k)]
                index_best = k
    if is_lis:
        best_scenarios.append([pop_ff['pop_lis_'+str(index_best)], best_ff])
    else:
        best_scenarios.append([pop_ff['pop_'+str(index_best)], best_ff])
    ### Check if no progress ###
    avg = 0
    no_progress = False

    if gen >= generation_last_restart + 5:
        
        for jey in range(gen-5, gen):
            avg += best_scenarios[jey][1]
        avg /= 5
            
        if avg >= best_scenarios[-1][1]:
            generation_last_restart = gen
            no_progress = True

    if was_restarted:
        was_restarted = False
        best_ff_after_restart = best_scenarios[-1][1]

    ### Restart if no progress
    if no_progress:
        was_restarted = True
        logger.info("No progress made, restarting the population at gen " + str(gen))
        population = {}
        for ind in range(pop_size):
            solution = {}
            current__position = random.randint(0,2)
            for i in range(number_npcs):
                s = []
                for j in range(6): # Generate six actions, one each 10 seconds
                    action = random.randint(0,2)
                    s.append([random.randrange(0, npc_top_speed), random.randint(1,8)])
                    # current__position = next_position
                solution.update({"npc"+str(i):s})
            population['pop_'+str(ind)] = solution
        logger.info("Generated " + str(pop_size) + " new solutions")

    # Here goes the local fuzzer
    # 
    elif best_scenarios[-1][1] > best_ff_after_restart and is_lis == False:
        best_ff_after_restart = best_scenarios[-1][1]
        if gen > (generation_last_restart + min_generation_lis):
            is_lis = True
            pop_before_lis = copy.deepcopy(population) # Save current population for after LIS
            population = mutator.process(pop_ff, p_mutation * 1.5, p_crossover)
            logger.info("Starting Local Iterative Search at generation "+str(gen))
            lis_counter +=1
            max_gen += lis_gens
            logger.info("New max gens: " + str(max_gen))
            pop_ff = {}
    
    elif is_lis:
        #Check if LIS is still running
        if lis_counter < lis_gens:
            lis_counter += 1
            population = mutator.process_lis(pop_ff, p_mutation * 1.5, p_crossover)
            logger.info("Starting LIS generation"+str(lis_counter))

            # check if there was a better solution in this LIS generation
            for x in range(int(len(pop_ff)/2)):
                if pop_ff['fitness_lis_'+str(x)] > best_ff_after_restart:
                    logger.info('Found better solution with FF= ' +str(pop_ff['fitness_lis_'+str(x)]))
                    aux_pop.append(pop_ff['pop_lis_'+str(x)])
        else:
            is_lis = False
            logger.info("Finishing LIS")
            population = mutator.process(pop_before_lis, p_mutation, p_crossover)
            len_pop = len(population)
            for x in range(len_pop, len_pop + len(aux_pop)):
                population.update({'pop_'+str(x):aux_pop[x-len_pop]})
            pop_ff = {}
    # If progress was made, continue the GA mutator
    else:
        population = mutator.process(pop_ff, p_mutation, p_crossover)

# Write simulation results to a CSV
with open(output_folder+"/results_sim.csv", 'x', encoding='UTF8', newline='') as c:
    writer = csv.writer(c)
    writer.writerow(csv_headers)
    writer.writerows(csv_data)

with open(output_folder+"best_scenarios.csv", 'x', encoding='UTF8', newline='') as b:
    writer_sce = csv.writer(b)
    writer_sce.writerow(['scenario', 'fitness'])
    writer_sce.writerows(best_scenarios)

logger.info("Max Fitness: " + str(max_fitness))
logger.info("Min Distance: " + str(min_distance))
logger.info("Execution Time: " + str(sim_time_elapsed.seconds))
