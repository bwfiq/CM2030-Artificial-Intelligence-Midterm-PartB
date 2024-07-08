import time
import pybullet as p
import pybullet_data
import numpy as np
import os
import creature
import genome
import population
from multiprocessing import Pool

class Simulation: 
    def __init__(self, sim_id=0):
        self.physicsClientId = p.connect(p.DIRECT)
        self.sim_id = sim_id

    def run_creature(self, cr, iterations=2400):     
        pid = self.physicsClientId
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=pid)
        p.setAdditionalSearchPath('shapes/', physicsClientId=pid)
        p.resetSimulation(physicsClientId=pid)
        p.setPhysicsEngineParameter(enableFileCaching=0, physicsClientId=pid)
        p.setGravity(0, 0, -10, physicsClientId=pid)
        
        # Create the environment
        arena_size = 20
        wall_height = 1
        wall_thickness = 0.5
        floor_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, arena_size/2, wall_thickness], physicsClientId=pid)
        floor_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, arena_size/2, wall_thickness], rgbaColor=[1, 1, 0, 1], physicsClientId=pid)
        floor_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=floor_collision_shape, baseVisualShapeIndex=floor_visual_shape, basePosition=[0, 0, -wall_thickness], physicsClientId=pid)
        wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, wall_thickness/2, wall_height/2], physicsClientId=pid)
        wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, wall_thickness/2, wall_height/2], rgbaColor=[0.7, 0.7, 0.7, 1], physicsClientId=pid)  # Gray walls
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[0, arena_size/2, wall_height/2], physicsClientId=pid)
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[0, -arena_size/2, wall_height/2], physicsClientId=pid)
        wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[wall_thickness/2, arena_size/2, wall_height/2], physicsClientId=pid)
        wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[wall_thickness/2, arena_size/2, wall_height/2], rgbaColor=[0.7, 0.7, 0.7, 1], physicsClientId=pid)  # Gray walls
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[arena_size/2, 0, wall_height/2], physicsClientId=pid)
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[-arena_size/2, 0, wall_height/2], physicsClientId=pid)
        mountain_position = (0, 0, -1)
        mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))
        mountain = p.loadURDF("gaussian_pyramid.urdf", mountain_position, mountain_orientation, useFixedBase=1, physicsClientId=pid)

        # load in the creature
        xml_file = 'temp' + str(self.sim_id) + '.urdf'
        with open(xml_file, 'w') as f:
            f.write(cr.to_xml())
        cid = p.loadURDF(xml_file, physicsClientId=pid)
        p.resetBasePositionAndOrientation(cid, [5, 5, 2.5], [0, 0, 0, 1], physicsClientId=pid)

        # Run the simulation
        for step in range(iterations):
            p.stepSimulation(physicsClientId=pid)
            
            # Move the creature's motors
            if step % 24 == 0:
                for jid in range(p.getNumJoints(cid, physicsClientId=self.physicsClientId)):
                    m = cr.get_motors()[jid]
                    p.setJointMotorControl2(cid, jid, 
                            controlMode=p.VELOCITY_CONTROL, 
                            targetVelocity=m.get_output(), 
                            force = 5, 
                            physicsClientId=self.physicsClientId)
            # Update the creature's position
            pos, _ = p.getBasePositionAndOrientation(cid, physicsClientId=pid)
            cr.update_position(pos)
        
        # delete file created
        os.remove(xml_file)
        # Update fitness
        if pos[0] > arena_size/2 or pos[0] < -arena_size/2 or pos[1] > arena_size/2 or pos[1] < -arena_size/2:
            cr.set_fitness(0)
        else:
            distance_from_peak = np.linalg.norm(np.array(pos) - np.array([0, 0, 4]))
            cr.set_fitness(1.0 / distance_from_peak)

class ThreadedSim:
    def __init__(self, pool_size):
        self.sims = [Simulation(i) for i in range(pool_size)]

    @staticmethod
    def static_run_creature(sim, cr, iterations):
        sim.run_creature(cr, iterations)
        return cr
    
    def eval_population(self, pop, iterations):
        """
        pop is a Population object
        iterations is frames in pybullet to run for at 240fps
        """
        pool_args = [] 
        start_ind = 0
        pool_size = len(self.sims)
        while start_ind < len(pop.creatures):
            this_pool_args = []
            for i in range(start_ind, start_ind + pool_size):
                if i == len(pop.creatures):# the end
                    break
                # work out the sim ind
                sim_ind = i % len(self.sims)
                this_pool_args.append([
                            self.sims[sim_ind], 
                            pop.creatures[i], 
                            iterations]   
                )
            pool_args.append(this_pool_args)
            start_ind = start_ind + pool_size

        new_creatures = []
        for pool_argset in pool_args:
            with Pool(pool_size) as p:
                # it works on a copy of the creatures, so receive them
                creatures = p.starmap(ThreadedSim.static_run_creature, pool_argset)
                # and now put those creatures back into the main 
                # self.creatures array
                new_creatures.extend(creatures)
        pop.creatures = new_creatures

class RunSimulation():
    @staticmethod
    def run_simulation(population_size = 10, 
                       gene_count = 3,
                       num_generations=10, 
                       crossover_iterations = 1,
                       point_mutations = 1,
                       point_mutation_rate = 0.5,
                       shrink_mutations = 1,
                       shrink_mutation_rate = 0.25,
                       grow_mutations = 1,
                       grow_mutation_rate = 0.5):
        start_time = time.time()
        pop = population.Population(pop_size=population_size, gene_count=gene_count)
        sim = ThreadedSim(pool_size=16)

        # Mutation Parameters
        # params = {
        #     "crossover_iterations": crossover_iterations,
        #     "point_mutations": point_mutations,
        #     "point_mutation_rate": point_mutation_rate,
        #     "shrink_mutations": shrink_mutations,
        #     "shrink_mutation_rate": shrink_mutation_rate,
        #     "grow_mutations": grow_mutations,
        #     "grow_mutation_rate": grow_mutation_rate
        # }

        # Metrics
        results = {
            "Median Fitness": [],
            "Standard Deviation of Fitness": [],
            "Median Distance Travelled": [],
            "Mutation Improvement Count": []
        }

        for iteration in range(num_generations):
            sim.eval_population(pop, 2400) # 10 seconds for each creature
            fits = [cr.get_fitness() for cr in pop.creatures]
            #print(iteration, "fittest:", np.round(np.max(fits), 3), "mean:", np.round(np.mean(fits), 3))
            
            # Metrics
            total_distance_travelled = 0
            mutation_improvement_count = 0
            # Find the two best parents
            new_creatures = []
            p1 = pop.creatures[np.argmax(fits)]
            fits[np.argmax(fits)] = 0 # remove the best parent from the list
            p2 = pop.creatures[np.argmax(fits)]
            for i in range(len(pop.creatures)):
                # Metrics
                total_distance_travelled += pop.creatures[i].get_distance_travelled() # for metrics
                if iteration > 0 and pop.creatures[i].get_fitness() > results["Median Fitness"][iteration - 1]:
                    mutation_improvement_count += 1
                
                # Mutations
                dna = None
                for _ in range(crossover_iterations):
                    dna = genome.Genome.crossover(p1.dna, p2.dna)
                for _ in range(point_mutations):
                    dna = genome.Genome.point_mutate(dna, rate=point_mutation_rate)
                for _ in range(shrink_mutations):
                    dna = genome.Genome.shrink_mutate(dna, rate=shrink_mutation_rate)
                for _ in range(grow_mutations):
                    dna = genome.Genome.grow_mutate(dna, rate=grow_mutation_rate)
                    
                # Create a new creature with the new DNA
                cr = creature.Creature(1)
                cr.update_dna(dna)
                new_creatures.append(cr)
            
            # Replace the first two creatures with the parents
            new_cr1 = creature.Creature(1)
            new_cr1.update_dna(p1.dna)
            new_creatures[0] = new_cr1
            new_cr2 = creature.Creature(1)
            new_cr2.update_dna(p2.dna)
            new_creatures[1] = new_cr2
            
            # Get the best creature from each generation and save it to file
            filename = "elite_" + str(iteration) + ".csv"
            genome.Genome.to_csv(p1.dna, filename)
            #print ("Elite saved to", filename, "with fitness", p1.get_fitness())
            
            # Measure metrics
            results["Median Fitness"].append(np.median(fits))
            results["Standard Deviation of Fitness"].append(np.std(fits))
            results["Median Distance Travelled"].append(total_distance_travelled / len(pop.creatures))
            results["Mutation Improvement Count"].append(mutation_improvement_count)
            
            # Get the new creatures ready for next simulation
            pop.creatures = new_creatures
            
        end_time = time.time()
        print("Time taken:", end_time - start_time)
        
        return results