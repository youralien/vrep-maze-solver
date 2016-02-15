# vrep-maze-solver
the ePuck robot (2 wheel differential drive) plans a path and tries to avoid obstacles. Implemented in the VREP simulator environment.  Inspired by NUS EE4306 - Distributed Autonomous Robotic Systems Lab 2. 

# How to Run
First, enter the `PythonWorkspace` directory
```
cd PythonWorkspace/
```
Then open VREP with a continous remote API going on port 19999. On linux, it looks like
```
bash /absolute/path/to/vrep.sh /absolute/path/to/Lab2_Environment.ttt -gREMOTEAPISERVERSERVICE_19999_FALSE_FALSE 
```
Finally, start the python program
```
python Lab2Program.py
```

