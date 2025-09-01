# Dynamic Window Approach Trajectory Planner

This project implements a trajectory planner for rovers using the Dynamic Window Approach (DWA). The DWA is a popular method for real-time trajectory planning that considers the rover's dynamics and the environment to generate feasible trajectories.

## Overview

The DWA algorithm computes possible trajectories based on the rover's current state and velocity, evaluates them, and selects the optimal trajectory to follow. This approach is particularly useful for mobile rovers navigating in dynamic environments.

## Project Structure

```
dwa_demo
├── src
│   ├── run_dwa_demo.py       # Entry point of the demo application
│   ├── dwa
│   │   ├── dwa_planner.py    # Implementation of the DWA trajectory planner
│   │   └── rover_data.py     # Classes related to the rover
│   ├── util
│   │   └── math.py           # Utility functions for mathematical operations
│   └── tests
│       └── test_dwa.py       # :warning: Placeholder file for unit tests
├── pyproject.toml            # Project dependencies and build configuration
└── README.md                 # Project documentation
```

## Setup

First make sure you have [uv](https://docs.astral.sh/uv/getting-started/installation/)
installed.

Then to set up the project, clone the repository and install the required
dependencies:

```bash
git clone <repository-url>
cd dwa_demo
uv sync
```

## Usage

To run the trajectory planner, execute the following command:

```bash
uv run -m src.run_dwa_demo
```

## Examples

The project includes examples of how to use the DWA class and its methods. Refer
 to the `src/tests/test_dwa.py` file for unit tests that demonstrate the
 functionality of the DWA implementation.
