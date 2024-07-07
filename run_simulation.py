import numpy as np
import genome
import creature
import population
import simulation
import time

start_time = time.time()
pop = population.Population(pop_size=10, gene_count=3)
sim = simulation.ThreadedSim(pool_size=16)

# Mutation Parameters
crossover_iterations = 1
point_mutations = 1
point_mutation_rate = 0.1
shrink_mutations = 1
shrink_mutation_rate = 0.25
grow_mutations = 1
grow_mutation_rate = 0.1

for iteration in range(100):
    # Run the simulation and get the fitnesses
    sim.eval_population(pop, 2400) # 10 seconds for each creature
    fits = [cr.get_fitness() for cr in pop.creatures]
    #print("fits", fits)
    print(iteration, "fittest:", np.round(np.max(fits), 3), "mean:", np.round(np.mean(fits), 3))
    
    # Genetic algorithm 
    # Find the two best parents
    new_creatures = []
    p1 = pop.creatures[np.argmax(fits)]
    fits[np.argmax(fits)] = 0 # remove the best parent from the list
    p2 = pop.creatures[np.argmax(fits)]
    for i in range(len(pop.creatures)):
        for _ in range(crossover_iterations):
            dna = genome.Genome.crossover(p1.dna, p2.dna)
        for _ in range(point_mutations):
            dna = genome.Genome.point_mutate(dna, rate=point_mutation_rate)
        for _ in range(shrink_mutations):
            dna = genome.Genome.shrink_mutate(dna, rate=shrink_mutation_rate)
        for _ in range(grow_mutations):
            dna = genome.Genome.grow_mutate(dna, rate=grow_mutation_rate)
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
    
    # Get the new creatures ready for next simulation
    pop.creatures = new_creatures
end_time = time.time()
print("Time taken:", end_time - start_time)