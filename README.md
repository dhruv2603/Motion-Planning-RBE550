# Robot Motion Planning and Benchmarking of different Planners

### *RBE550: Motion Planning - [Worcester Polytechnic Institute](https://www.wpi.edu/), Fall 2024*

## Project Guidlines:
- There are 5 projects each one building on the previous project. 
- OMPL library is used to plan the path of the robot. 
- The projects are implemented in C++.

## Project 0: Getting Familiar with OMPL

### Goals of the project
1. Getting familiar with and using the [Open Motion Planning Library (OMPL)](https://ompl.kavrakilab.org/).
2. Understanding basic requirements for the course and becoming acquianted with the style of the course projects.

### Running the project
To run the planner in the envionment, use the following command
```bash
mkdir build && cd build
cmake ../ && make
./Point2DPlanning
```
## Project 1: Throwing Points on a Disk

### Goals of the project
1. Introduction to key operations in OMPL, specifically sampling and collision checking for a simple point robot.

### Running the project
```bash
# Build the project
mkdir -p build && cd build
cmake ../ && make
# Run the executable
./diskPlanning
# Run the visualize script
python3 visualize.py
```

## Project 2: Barking Up a Random Tree

### Goals of the project
1. Implement a simple sampling-based motion planner in OMPL to plan for a rigid body.
2. Systematically compare the planner to existing methods in the library.

### Running the project
```bash
# Build the project
mkdir -p build && cd build
cmake ../ && make
# Run the planner executable
./planRTP
# Run the visualize script
python3 visualise.py --obstacles <obs file> --path <path file>
# Run the benchmark executable
./KCBenchmark
```

## Project 3: Boxing with a Chain

### Goals of the project
1. Implement several advanced features such as custom configuration spaces, benchmarking asymptotically optimal planners, and exploring different narrow passage sampling distributions.
2. Implement a custom optimization objective.

### Running the project
```bash
# Build the project
mkdir -p build && cd build
cmake ../ && make
# Run the planner executable
./plan
# Run the visualize script
python3 visualise.py --obstacles <obs file> --path <path file>
```

## Project 4: Out of Control Planning

### Goals of the project
1. Plan motions for non-holonomic systems whose dynamics are described by an ordinary differential equation of the form $\dot{q} = f(q,u).$
2. Compute the dynamically feasible and collision free motions for these systems using RRT and KPIECE planners.
3. Learn about and implement a new planner called Reachability-Guided RRT (RG-RRT).

### Running the project
```bash
# Build the project
mkdir -p build && cd build
cmake ../ && make
# Run the planner executable
./car
./pendulum
# Run the visualize script
python3 visualise.py --obstacles <obs file> --path <path file>
```