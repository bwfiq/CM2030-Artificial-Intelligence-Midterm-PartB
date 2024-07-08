import os
import matplotlib.pyplot as plt
import logging
from typing import Dict, List, Any
from run_simulation import RunSimulation

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

def run_simulation_with_params(params: Dict[str, Any]) -> Dict[str, List[float]]:
    return RunSimulation.run_simulation(**params)

def plot_results(results: Dict[str, List[List[float]]], title: str) -> None:
    for key, value in results.items():
        x = list(range(len(value[0])))
        for i, val in enumerate(value):
            plt.plot(x, val, label=f"Simulation {i+1}")
        plt.title(title)
        plt.xlabel("Generation")
        plt.ylabel(key)
        plt.legend()
        plt.savefig(f"results/{title}_{key}.png")
        plt.clf()

def run_experiment_1() -> None:
    experiment_params = [
        ("Point Mutation Iterations", "point_mutations", [1, 2, 3, 4, 5]),
        ("Point Mutation Rate", "point_mutation_rate", [0.2, 0.4, 0.6, 0.8, 1.0]),
        ("Shrink Mutation Iterations", "shrink_mutations", [1, 2, 3, 4, 5]),
        ("Shrink Mutation Rate", "shrink_mutation_rate", [0.2, 0.4, 0.6, 0.8, 1.0]),
        ("Grow Mutation Iterations", "grow_mutations", [1, 2, 3, 4, 5]),
        ("Grow Mutation Rate", "grow_mutation_rate", [0.2, 0.4, 0.6, 0.8, 1.0])
    ]

    base_params = {
        "population_size": 10,
        "gene_count": 3,
        "num_generations": 100,
        "crossover_iterations": 1,
        "point_mutations": 1,
        "point_mutation_rate": 0.1,
        "shrink_mutations": 1,
        "shrink_mutation_rate": 0.25,
        "grow_mutations": 1,
        "grow_mutation_rate": 0.1
    }

    for title, param, values in experiment_params:
        for value in values:
            logging.info(f"--- {title}: {value} ---")
            base_params[param] = value
            results = run_simulation_with_params(base_params)
            median_fitness_delta = results["Median Fitness"][-1] - results["Median Fitness"][0]
            logging.info(f"Median Fitness Delta: {median_fitness_delta}")

if __name__ == "__main__":
    if not os.path.exists("results"):
        os.makedirs("results")

    # Experiment 1: Varying mutation parameters
    run_experiment_1()
    
    logging.info("Done!")
