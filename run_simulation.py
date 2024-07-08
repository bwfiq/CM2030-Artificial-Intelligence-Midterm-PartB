import time
import numpy as np
import genome
import population
import creature
import simulation

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
        sim = simulation.ThreadedSim(pool_size=10)

        # Metrics
        results = {
            "Median Fitness": [],
            "Standard Deviation of Fitness": [],
            "Median Distance Travelled": [],
            "Mutation Improvement Count": []
        }

        for iteration in range(num_generations):
            sim.eval_population(pop, 2400) # 10 seconds for each creature
            fits = np.array([cr.get_fitness() for cr in pop.creatures])
            
            # Metrics
            total_distance_travelled = sum(cr.get_distance_travelled() for cr in pop.creatures)
            mutation_improvement_count = sum(cr.get_fitness() > results["Median Fitness"][-1] for cr in pop.creatures) if results["Median Fitness"] else 0 
            
            # Find the two best parents
            new_creatures = []
            sorted_creatures = sorted(pop.creatures, key=lambda cr: cr.get_fitness(), reverse=True)
            p1, p2 = sorted_creatures[:2]
            for _ in range(len(pop.creatures)):
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
            new_creatures[0] = p1
            new_creatures[1] = p2
            
            # Get the best creature from each generation and save it to file
            genome.Genome.to_csv(p1.dna, f"elite_{iteration}.csv")
                        
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