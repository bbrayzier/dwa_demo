# Dynamic Window Approach Trajectory Planner

This project implements a trajectory planner for roverics using the Dynamic Window Approach (DWA). The DWA is a popular method for real-time trajectory planning that considers the rover's dynamics and the environment to generate feasible trajectories.

## Overview

The DWA algorithm computes possible trajectories based on the rover's current state and velocity, evaluates them, and selects the optimal trajectory to follow. This approach is particularly useful for mobile rovers navigating in dynamic environments.

## Project Structure

```
dwa_trajectory_planner
├── src
│   ├── main.py               # Entry point of the application
│   ├── planner
│   │   └── dwa.py            # Implementation of the DWA class
│   ├── utils
│   │   └── math_utils.py      # Utility functions for mathematical operations
│   └── tests
│       └── test_dwa.py       # Unit tests for the DWA class
├── requirements.txt           # Project dependencies
└── README.md                  # Project documentation
```

## Setup

To set up the project, clone the repository and install the required dependencies:

```bash
git clone <repository-url>
cd dwa_trajectory_planner
pip install -r requirements.txt
```

## Usage

To run the trajectory planner, execute the following command:

```bash
python src/main.py
```

## Examples

The project includes examples of how to use the DWA class and its methods. Refer to the `src/tests/test_dwa.py` file for unit tests that demonstrate the functionality of the DWA implementation.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request for any enhancements or bug fixes.