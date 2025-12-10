# Simple Electromechanical Control Systems using Deep Networks and Edge AI

## Disclaimer - Work In Progress

This is an ongoing project to apply my classroom AI knowledge to my technical discipline of electromechanical design and prototyping.

## Tasks and Status

General approach:

- Design and build the apparatus,
- Develop minimal embedded code to pass data to/from computer and develop AI models on computer
- Develop final models for purely edge-AI system, where all activities take place on the microcontroller

Tasks:

- [x] General system design and material selection
- [x] Detailed CAD design
- [x] Mechanical construction
- [ ] Electronic circuit construction
  - [x] Proof-of-concept using pre-existing boards from other projects
  - [ ] PCB fabrication for edge-AI system
- [x] Embedded development environment setup
- [x] Embedded programming for data acquisition
- [x] EDA and data validation, bug fixes
- [ ] Proof-of-concept AI model deployment on computer,
- [ ] Network architecture experimentation on computer
  - [ ] Determine suitable size, activation functions etc.
  - [ ] Determine limits for embedded code (network size and latency)
- [ ] Final embedded system code
  - [x] Hardware libraries tested for required matrix operations
  - [ ] Create matrices based on computer-based testing
  - [ ] Write C opitimizer for on-line learning

## Apparatus description

<img src="media/apparatus_CAD.png" width="300">

The apparatus was designed with two functions:

### Simple Pendulum

> Goal: Flexible, adaptive system for precise control and accurate predictions of future machine states.  

> Goal: On-line learning on edge device with minimal prior system knowledge.

In this mode, a magnet at the end of a rotating shaft is subjected to a magnetic impulse when it passes over an electromagnet.  The shaft position is tracked using a combination of an absolute encoder (~0.1-degree resolution) and time-series data interpolation, allowing for a control mode where a neural network learns the system characteristics and adjusts impulse strength to achieve specific target amplitudes.

Beyond a learning exercise, the intent is to develop a flexible control system that is rapidly able to adapt to changing system parameters, such as when the pendulum counterweight is altered, or impule strength is affected by external factors.

### Null-Force Scale (not yet implemented), 

> Goal: use active methods to improve weigh scale performance by minimizing settling time, maximizing resolution and linearity

In this mode, the same electromagnet is used to provide a controllable force to offset applied forces to a load cell.  This is a modification to standard electronic scale reading which uses a restoring force to maintain a scale in its most linear region. 


The plan is to use a neural network to supervise standard [PID](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller) auto-tuning algorithms based on [published academic results](https://www.researchgate.net/publication/242777564_Neural_Network-based_AutoTuning_for_PID_Controllers).  

## Notebook links

[EDA](EDA.ipynb) - Initial exploration into encoder and timing data being streamed from device during automatic stimulus-and-decay experiments.  This is intended to validate the data that will be seen by the network and gain a statistical understanding of variance, temporal drift or other factors to provide background and context for later observations.  If, for example, there are significant outliers or glitches present, that may help explain perodic anomalies in network training loss.

<a href="https://photos.app.goo.gl/SNn2zTws2jMJB6hWA">
  <img src="media/image.png" alt="Apparatus picture, click to see video" style="height: 300px;">
</a>

## Other Notes

These are not intended for presentation, but personal notes for future reference:

### Emdedded Environment (ESP-IDF) Setup Log (for MacOS)

1. Follow [ESP-IDF MacOS setup](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html#step-1-install-prerequisites) instructions \#1-3 (in home directory):
    ```bash
    brew install cmake ninja dfu-util
    brew install ccache
    ```
2. Install VSCode ESP-IDF extension  
3. Run Express setup (ESP-IDF Setup)
   - Python version: usr/local/bin/python
   - Other options: install v5.5.1 and use default paths
