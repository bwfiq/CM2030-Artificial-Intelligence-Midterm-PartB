import numpy as np
import genome
import creature
import population
import simulation
import time

start_time = time.time()
pop = population.Population(pop_size=10, gene_count=3)
sim = simulation.ThreadedSim(pool_size=10)
for iteration in range(100):
    # Run the simulation and get the fitnesses
    sim.eval_population(pop, 2400) # 10 seconds for each creature
    fits = [cr.get_distance_travelled() for cr in pop.creatures] # old fitness function
    #fits = [cr.get_fitness() for cr in pop.creatures]
    links = [len(cr.get_expanded_links()) for cr in pop.creatures]
    print(iteration, 
          "fittest:", np.round(np.max(fits), 3), 
          "mean:", np.round(np.mean(fits), 3), 
          "mean links", np.round(np.mean(links)), 
          "max links", np.round(np.max(links)))
    
    # Genetic algorithm   
    fit_map = population.Population.get_fitness_map(fits)
    new_creatures = []
    for i in range(len(pop.creatures)):
        p1_ind = population.Population.select_parent(fit_map)
        p2_ind = population.Population.select_parent(fit_map)
        p1 = pop.creatures[p1_ind]
        p2 = pop.creatures[p2_ind]
        # now we have the parents!
        dna = genome.Genome.crossover(p1.dna, p2.dna)
        dna = genome.Genome.point_mutate(dna, rate=0.1, amount=0.25)
        dna = genome.Genome.shrink_mutate(dna, rate=0.25)
        dna = genome.Genome.grow_mutate(dna, rate=0.1)
        cr = creature.Creature(1)
        cr.update_dna(dna)
        new_creatures.append(cr)
    
    # Get the best creature from each generation and save it to file as well as allow it to be in the next generation
    max_fit = np.max(fits)
    for cr in pop.creatures:
        if cr.get_distance_travelled() == max_fit:
            new_cr = creature.Creature(1)
            new_cr.update_dna(cr.dna)
            new_creatures[0] = new_cr
            filename = "elite_"+str(iteration)+".csv"
            genome.Genome.to_csv(cr.dna, filename)
            break

    pop.creatures = new_creatures
end_time = time.time()
print("Time taken:", end_time - start_time)