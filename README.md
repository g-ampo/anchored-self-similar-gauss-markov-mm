# Anchored Self-Similar 3D Gauss-Markov Mobility Model for NS-3

## Overview

The **Anchored Self-Similar 3D Gauss-Markov Mobility Model** is a custom mobility model designed for NS-3, implementing an enhanced version of the basic Gauss-Markov mobility model. The model introduces self-similar, exponentially decaying randomness to better simulate real-world movement patterns, particularly in scenarios involving drone swarms or mobile nodes operating in 3D space.

This mobility model supports the following features:
- **Self-similarity**: The model introduces exponentially decaying randomness to create smooth transitions in velocity, direction, and pitch.
- **3D mobility**: Unlike traditional 2D mobility models, this model supports movement in 3D space, making it suitable for drone swarm simulations or scenarios with nodes in vertical motion.
- **Boundaries**: Movement can be restricted within specified 3D boundaries, ensuring nodes do not drift beyond predefined regions.

### Key Parameters:
- **`Alpha`**: Controls the level of randomness. A higher value introduces less randomness, making the movement more deterministic.
- **`Bounds`**: Defines the 3D boundary box within which nodes can move.
- **`TimeStep`**: The time interval at which mobility updates occur.
- **`MeanVelocity`, `MeanDirection`, `MeanPitch`**: Specifies the average velocity, direction, and pitch for the nodes.
- **`NormalVelocity`, `NormalDirection`, `NormalPitch`**: Adds randomness to velocity, direction, and pitch, drawn from normal distributions.

## Files

### 1. **`anchored-ss-gauss-markov-mobility-model.h`**
This header file defines the `AnchoredSelfSimilarGaussMarkovMobilityModel` class, including methods for setting parameters, initializing the model, and retrieving node positions and velocities.

Key attributes:
- `m_alpha`: Controls the level of randomness in the movement.
- `m_bounds`: Specifies the 3D boundary within which the nodes can move.
- `m_velocity`: Stores the current velocity vector of the node.
- `m_position`: Stores the current position vector of the node.
- `m_rndSpeed`, `m_rndDirection`, `m_rndPitch`: Random variables used to introduce randomness in speed, direction, and pitch.

### 2. **`anchored-ss-gauss-markov-mobility-model.cc`**
This source file contains the implementation of the mobility model. It defines the `TypeId` for the mobility model and handles updates to the node's velocity and position based on the parameters.

### 3. **`wscript`**
This is the build script for the NS-3 module. It lists all the source files and ensures that the mobility model is properly integrated into NS-3.

## How the Model Works

### Equation-Based Mobility Update

The movement of nodes in this model is governed by a set of Gauss-Markov-based equations, updated at each time step. The following equations are used to update velocity, direction, and pitch:

1. **Speed Update**:
    \[
    s_n = a_s s_{(n-1)} + (1 - a_s)\bar{s} + \sqrt{(1 - a_s^2)} s_{x_{n-1}}
    \]
    - `a_s` is the randomness factor, which decays exponentially based on the previous speed.

2. **Direction Update**:
    \[
    d_n = a_d d_{(n-1)} + (1 - a_d)\bar{d} + \sqrt{(1 - a_d^2)} d_{x_{n-1}}
    \]
    - `a_d` is the randomness factor, which decays based on the previous direction and acceleration.

3. **Pitch Update**:
    \[
    p_n = a_p p_{(n-1)} + (1 - a_p)\bar{p} + \sqrt{(1 - a_p^2)} p_{x_{n-1}}
    \]
    - `a_p` is the randomness factor, which decays based on the previous pitch and acceleration.
  
### 4. **`anchored-ss-gm-mm.cc`**
This is a very simple example to test the correct build and installation of the new mobility model.
After building, place it in your scratch folder and run with `./waf --run scratch/anchored-ss-gm-mm`.

### Exponentially Decaying Randomness

In each update cycle, the randomness factors `a_s`, `a_d`, and `a_p` exponentially decay over time, creating smooth and realistic transitions in node movement. These factors are updated based on the differences between the previous and current values of speed, direction, and pitch.

### Boundary Handling

If the model is used with 3D boundaries (`Bounds`), the model ensures that the nodes remain within the specified boundaries. The node's position is updated after each time step, and any attempt to move outside the bounds is corrected by reflecting the position back into the boundary.

## Build and Installation

### Prerequisites

- **NS-3**: You need to have NS-3 installed on your system. This model has been developed for NS-3, version 3.30.1, but should be compatible with other versions as well.
- **C++ compiler**: A C++ compiler such as `g++` or `clang` is required.

### Build Instructions

1. **Clone or move the files into the `src/mobility/` folder of your NS-3 installation**:
   - Place the `.h` and `.cc` files inside the `src/mobility/model/` directory.
   - The `wscript` file should be placed inside the `src/mobility/` directory.

2. **Modify the `wscript` file** (if not already done):
   Ensure that the `anchored-ss-gauss-markov-mobility-model.cc` and `.h` files are included in the `src/mobility/wscript` file. This should already be set up, but it’s worth verifying.

   Example:
   ```python
   def build(bld):
       mobility = bld.create_ns3_module('mobility', ['network'])
       mobility.source = [
           'model/anchored-ss-gauss-markov-mobility-model.cc',
           # other model files...
       ]
       mobility_test = bld.create_ns3_module_test_library('mobility')
       mobility_test.source = [
           # test suite files...
       ]
       headers = bld(features='ns3header')
       headers.module = 'mobility'
       headers.source = [
           'model/anchored-ss-gauss-markov-mobility-model.h',
           # other header files...
       ]
