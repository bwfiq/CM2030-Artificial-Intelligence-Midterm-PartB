import simulation
import matplotlib.pyplot as plt
import os

# Create the "results" subfolder if it doesn't exist
if not os.path.exists("results"):
    os.makedirs("results")

# Create a dict for the graphs to reference
overall_results = {
    "Median Fitness": [],
    "Standard Deviation of Fitness": [],
    "Median Distance Travelled": [],
    "Mutation Improvement Count": []
}

# Run the simulations and store the results
for simulation_count in range(5):
    print("--- Simulation", simulation_count, "---")
    results = simulation.RunSimulation.run_simulation(population_size=10, 
                                                      gene_count=3,
                                                      num_generations=100, 
                                                      crossover_iterations=1,
                                                      point_mutations=1,
                                                      point_mutation_rate=0.1,
                                                      shrink_mutations=1,
                                                      shrink_mutation_rate=0.25,
                                                      grow_mutations=1,
                                                      grow_mutation_rate=0.1)
    for key in results.keys():
        overall_results[key].append(results[key])

# Plot the median fitness over the generations and simulations and save to file
for key, value in overall_results.items():
    x = list(range(len(overall_results[key][0])))
    for i in range(len(overall_results[key])):
        plt.plot(x, value[i], label="Simulation " + str(i+1))
    plt.xlabel("Generation")
    plt.ylabel(key)
    plt.legend()
    plt.savefig("results/" + key + ".png")
    plt.clf()