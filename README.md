# Multi-Drone Consensus and Barrier Function Master's Thesis

This repository contains the implementation and experimental evaluation framework developed
for a master's thesis on collision avoidance and coordinated control in multi-drone systems.

Videos of the Crazyflie drones can be found in /VideosOfDroneFlight

The project focuses primarily on:

- Barrier-function-based collision avoidance
- Consensus-based multi-agent coordination
- Path planning algorithms
- Real-world and simulated drone experiments

The repository includes both theoretical tests and real-world Crazyflie drone experiments.

---

# Repository Structure

```text
src/
├── Consensus/
├── RealWorldTests/
│   ├── Offline/
│   └── Online/
├── TheoreticalTests/
│   ├── Astar/
│   ├── Barrier/
│   └── RRT/
└── README.txt
VideosOfDroneFlight/
```

# Main Research Topics

This repository investigates:

- Multi-drone coordination
- Consensus-based formation control
- Collision avoidance using control barrier functions
- Online vs offline path planning
- Dynamic obstacle avoidance
- Orientation-aware safety constraints
- Comparison of:
  - A*
  - RRT*
  - Barrier methods
  - Penalty methods

---

# Experimental Goals

The experiments evaluate:

- Collision avoidance performance
- Runtime
- Path quality
- Scalability
- Robustness under dense multi-agent interactions

Metrics include:

- Flight time
- Path length
- Minimum inter-drone distance
- Runtime

---

# Notes

- Some scripts are intended specifically for testing.
- Some scripts require physical drone hardware and/or localization systems.
- When runnning the code, the generated figures are stored inside the experiment folders.
- Certain scripts may require parameter tuning depending on scenario complexity.


All experiments were conducted using Crazyflie 2.1+ from Bitcraze. 
Communication between the drones and the computer was handled using the Crazyradio 2.0 USB radio.

For positioning and tracking, we used the Lighthouse Positioning System 
from Bitcraze along with the Lighthouse Positioning Deck.