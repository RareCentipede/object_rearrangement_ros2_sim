import numpy as np
import matplotlib.pyplot as plt

def main():
    # Mock data for testing
    x = np.linspace(0, 100, 10)

    greedy_cost = 10 + np.linspace(1, 40, 10)
    mb_cost = 10 + np.linspace(1, 20, 10)
    greedy_cost_stab = 15 + np.linspace(1, 40, 10)
    mb_cost_stab = 12 + np.linspace(1, 30, 10)

    # greedy_cost = 70 * np.ones(100) * np.linspace(1.0, 0.5, 100) + np.random.normal(-5, 5, 100)  # Adding some noise
    # mb_cost = 80 * np.ones(100) * np.linspace(1.0, 0.8, 100) + np.random.normal(-5, 5, 100)  # Adding some noise
    # greedy_cost_stab = 80 * np.ones(100) * np.linspace(1.0, 0.5, 100) + np.random.normal(-5, 5, 100)  # Adding some noise
    # mb_cost_stab = 100 * np.ones(100) * np.linspace(1.0, 0.8, 100) + np.random.normal(-5, 5, 100)  # Adding some noise

    # Create a simple plot
    plt.figure(figsize=(10, 5))
    plt.plot(x, greedy_cost, marker='o', label='Greedy', c='blue')
    plt.plot(x, mb_cost, marker='o', label='Multi-bound', c='red')
    plt.plot(x, greedy_cost_stab, marker='o', label='Greedy-stable', c='green')
    plt.plot(x, mb_cost_stab, marker='o', label='Multi-bound-stable', c='orange')
    plt.title('Mock Plot for Navigation cost vs number of blocks per planner strategy')
    plt.xlabel('Number of blocks')
    plt.ylabel('Time take to terminate (s)')
    plt.legend()
    plt.grid()
    plt.show()

    # strategies = ['Greedy', 'Multi-bound', 'Greedy-stable', 'Multi-bound-stable']
    # scenarios = ['Scenario 1', 'Scenario 2', 'Scenario 3']  # Mock scenarios
    # costs = [[60, 75, 80], [90, 70, 80], [100, 80, 70], [100, 80, 90]]  # Mock costs for each strategy

    # # Create a multi-bar plot
    # plt.figure(figsize=(10, 5))
    # x = np.arange(len(scenarios))
    # width = 0.2

    # for i, strat in enumerate(strategies):
    #     plt.bar(x + i * width, costs[i], width, label=strat)

    # plt.xlabel('Planner Strategy')
    # plt.ylabel('Average stability (%)')
    # plt.title('Mock Bar Plot for average stability of different planner strategies across scenarios')
    # plt.xticks(x + width, scenarios)
    # plt.legend()
    # plt.grid()
    # plt.show()

if __name__ == "__main__":
    main()