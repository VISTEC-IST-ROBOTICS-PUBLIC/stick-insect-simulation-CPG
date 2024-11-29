# Stick insect simulation - CPG

![stick insect workshop1](https://github.com/VISTEC-IST-ROBOTICS-PUBLIC/stick-insect-simulation-CPG/blob/main/figure/self_organized_locomotion_poster.jpg)
I have developed a simulation designed for educational purposes to enhance understanding of neural control in legged robots, building upon and improving a simulation from [the published paper](https://onlinelibrary.wiley.com/doi/10.1002/adts.202300228).
This simulation is intended solely for academic and research use and is not for commercial sale.
Any reproduction, distribution, or use of this simulation without prior permission or citation is strictly prohibited and may result in legal consequences.

Developed by: Thirawat Chuthong
______

## Software and Library
- Windown 10 or newer
- CoppeliaSim simulation version 4.6.0 or newer - [Download here](https://www.coppeliarobotics.com/)
- Python version 3.8.8 or newer - [Download here](https://www.python.org/downloads/)
- (optional) Visual studio code [Download here](https://code.visualstudio.com/)
- Numpy - [see this link](https://numpy.org/install/)


## Installation
### 1) Install programs
- CoppeliaSim
- Visual studio code

### 2) Install Python
##### Option 1: install from __Microsoft Store__
    - search "python" >> install
##### Option 2: install from __Source__
    - [download](https://www.python.org/downloads/) >> install
    - add python path [see this](https://realpython.com/add-python-to-path/)


### 3) Copy python interpreter path to simulation setting
##### Goto Command prompt
~~~
$ where python
$ <your pyhon path show here and copy it>
~~~
##### Goto usrser.txt
"C: Users > {user_name} > AppData (need to show Hidden items) > Roaming > CoppeliaSim > usrser.txt"

~~~
*** in usrser.txt ***
...
...
defaultPython = <paste the copied python path here>
...
...

*** for example: ***
defaultPython = C:Users\user_name\AppData\Local\Microsoft\WindowsApps\python.exe
~~~

### 4) Open simulation
- Windows search "CoppeliaSim"
- Open our provided scene "stick_insect_sim_CPG.ttt"
- Run this simulation and the robot should move

#### (option) My simulation is NOT running
If the robot does not move
- See the __red text__ at output shell (the bottom of the simulation) and follow the warning

_____

## How to play
![stick insect workshop](https://github.com/VISTEC-IST-ROBOTICS-PUBLIC/stick-insect-simulation-CPG/blob/main/figure/How_to_play_stick_insect_workshop.png)

### Task:
Control each joint (18 joints) of the simulated robot to move forward

### Stick insect robot in CoppeliaSim simulation with python code
#### Provided code
- 3 CPGs with different phases
- 6 foot contact sensors
- 1 body orientation sensor
#### Improve
- Intralimb and interlimb (control each joint)
- (Option) Front leg amputations
- (Option) Middle leg amputations
- (Option) Front-left and hind-right leg amputations







_______ 
### Citation:
Paper title: Self-Organized Stick Insect-Like Locomotion under Decentralized Adaptive Neural Control: From Biological Investigation to Robot Simulation
Link: https://onlinelibrary.wiley.com/doi/10.1002/adts.202300228

@article{Larsen,
    author = {Larsen, Alexander and Büscher, Thies and Chuthong, Thirawat and Pairam, Thipawan and Bethge, Hendrik and Gorb, Stanislav and Manoonpong, Poramate},
    year = {2023},
    month = {06},
    pages = {},
    title = {Self‐Organized Stick Insect‐Like Locomotion under Decentralized Adaptive Neural Control: From Biological Investigation to Robot Simulation},
    volume = {6},
    journal = {Advanced Theory and Simulations},
    doi = {10.1002/adts.202300228 }
}

____
### Developer
Developed by:
Thirawat Chuthong (Joe) <br>
thirawat.c_s21@vistec.ac.th <br>
Ph.D. student at Bio-inspired Robotics and Neural Engineering Laboratory, <br>
School of Information Science and Technology,<br>
Vidyasirimedhi Institute of Science and Technology, <br>
Wangchan Valley 555 Moo 1, Payupnai, Wangchan, 21210 Rayong, Thailand
