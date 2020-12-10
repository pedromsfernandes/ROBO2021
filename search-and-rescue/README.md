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
$ mkdir build
$ cd build
$ cmake -DCMAKE_BUILD_TYPE=Release ..
$ make
```

## Run

```bash
# From search-and-rescue/
# Run the example
$ argos3 -c footbot_diffusion/diffusion_10.argos

# Run our simulation
$ argos3 -c src/search_and_rescue.argos
```