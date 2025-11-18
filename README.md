# Frontier Exploration

We wrote an explore function that returns valid cells to explore in our stochastic occupancy map, based on 3 exploration heuristics.

When a state in the grid is within the sensing radius of the robot, the state is known. Then it's assigned a probability of whether it's occupied, which the given heuristic tells us it's only occupied if it's greater or equal to 0.5. 

If the cell isn't within the sensing radius, then its value is -1. 

## How it works 

We're using convolutions for heuristics.


Heuristics:

1. The percentage of unknown cells surrounding a cell in some window should be greater than or equal to 20% of the surrounding cells.
- Why?
If the ratio < 20%, then the region around the cell is already largely explored / well-defined, so using stochastic updates may just introduce noise rather than improve the map

2. The number of known, occupied cells surrounding a cell in some window should be 0.


3. The percentage of known, unoccupied cells surrounding a cell in some window should be greater than or equal to 30% of the surrounding cells.

