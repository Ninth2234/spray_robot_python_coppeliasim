# Spray Robot Simulation in CoppeliaSim

This project provides a Python-based simulation of a spray robot using **CoppeliaSim** and **Python** with the **ZeroMQ Remote API**.

---

## Installation Guide  
This project has been developed using **CoppeliaSim 4.9.0 Rev2** and **Python 3.8.10**. For optimal compatibility, it's recommended to use these versions.

### 1. Clone the Repository
```sh
git clone https://github.com/Ninth2234/spray_robot_python_coppeliasim.git
cd spray_robot_python_coppeliasim
```

### 2. Set up a virtual environment
```sh
python -m venv .venv
source .venv/Scripts/activate
```

### 3. Install python dependencies
```sh
pip install -r requirements.txt
```
---

## Demo

To see the simulation in action, check out the example video below:

[![Watch Spray Robot Demo](https://img.youtube.com/vi/wToINhK9YoE/0.jpg)](https://youtu.be/wToINhK9YoE)

To quickly try out the simulation:

1. Open the **CoppeliaSim** scene `spray_robot.ttt`.
2. Run the Python script `spray_robot.py`:
 ```sh
source .venv/Scripts/activate
python spray_robot.py

```
