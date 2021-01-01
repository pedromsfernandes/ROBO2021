# Search and Rescue

## Pre-Requisites

* Ubuntu 16.04 / 18.04
* [ARGoS](https://www.argos-sim.info)

### Installing ARGoS

```bash
$ sudo dpkg -i argos3_simulator-3.0.0-x86_64-beta56.deb
$ sudo apt --fix-broken install
```

## Compile

```bash
# From search-and-rescue/
$ mkdir build
$ cd build
$ cmake -DCMAKE_BUILD_TYPE=Release ..
$ make
```

## Run

```bash
# From search-and-rescue/
# Run our simulation
$ argos3 -c experiments/search_and_rescue.argos
```

## Code Snippets

* [Draw ID of robots](https://github.com/ilpincy/argos3-examples/tree/master/loop_functions/id_loop_functions)
* [Draw trajectory of robots](https://github.com/ilpincy/argos3-examples/tree/master/loop_functions/trajectory_loop_functions)
* [Obstacle avoidance](https://github.com/ilpincy/argos3-examples/blob/6755b995b6982fd81e55dfe3121153841c4a7f5c/controllers/footbot_diffusion/footbot_diffusion.cpp#L64)
* [Set robot wheel speed](https://github.com/ilpincy/argos3-examples/blob/6755b995b6982fd81e55dfe3121153841c4a7f5c/controllers/footbot_foraging/footbot_foraging.cpp#L270)
* [Get current date time](https://stackoverflow.com/a/10467633)

## Directory Structure

```
.
├── controllers
|   └── searcher.cpp # Searcher robot controller
├── experiments # ARGoS launch files with arena and robots configurations
├── loop_functions 
|   ├── id_qtuser_functions.cpp # Draw ID and trajectory of robots
|   └── search_loop_functions.cpp # Save to file number of robots near target at each iteration
├── CMakeLists.txt
└── README.md
```

## Authors

* Luís Silva - up201503730@fe.up.pt
* Mariana Costa - up201604414@fe.up.pt
* Pedro Fernandes - up201603846@fe.up.pt

5th Year // ROBO // MIEIC // FEUP