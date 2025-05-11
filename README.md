# RL-Braitenberg

## Project Overview

This project features Braitenberg vehicles implemented with Mbed OS, incorporating wireless communication and a state table-based reinforcement learning algorithm using probabilities. The vehicles are designed to dynamically switch between various Braitenberg vehicle behaviors, as defined in Valentino Braitenberg's seminal work, "Vehicles: Experiments in Synthetic Psychology."

## Learning Mechanism

The core of the project lies in its unique reinforcement learning approach. The vehicles "learn" by utilizing environmental darkness as an objective function. After completing a state, a vehicle assesses the change in ambient light levels. This assessment then influences the probability of the vehicle transitioning into that same state again from its preceding state – rewarding states that lead to darker environments.

## Inter-Vehicle Communication

The vehicles in this system are not isolated. They possess the capability to influence each other's behavior. With a randomly determined probability, one vehicle can communicate its upcoming state to another. This communication increases the likelihood of the receiving vehicle also transitioning into the communicated state, fostering a basic level of swarm-like interaction.

## Current Capabilities and Future Expansion

Currently, the system is configured to support two interacting vehicles. However, the underlying architecture can be modified and extended to support a larger number of vehicles. Please note that accommodating more vehicles might necessitate some changes to improve modularity and scalability.

## Key Features

- Braitenberg Vehicle Implementation: Simulates multiple Braitenberg vehicle types.
- Reinforcement Learning: Employs a probability-based state table that adapts based on environmental light (darkness seeking).
- Wireless Communication: Enables vehicles to influence each other's state transitions.
- Mbed OS Based: Developed on the Mbed OS platform.
- Scalability: Designed with the potential to support more than two vehicles with modifications.

## Getting Started

To get started with the project, follow these steps:

1. Clone the repository to your local machine.
2. Set up the Mbed OS environment.
3. Compile and upload the code to your Mbed-compatible hardware (the original code was designed for the Discovery STM32F429ZI board).
4. Power on the vehicles and observe their behavior in a controlled environment.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgments

Special thanks to [Rafael Toameh](https://github.com/Rafififi) for help on this project.
