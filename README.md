# robutts
### A multi agent solution to collect small debri from beaches: a simulation

![gifexample](https://github.com/elenafillo/robutts/blob/main/animation_full.gif)

## About the algorithm:
* Robots (in green) move randomly collecting debri (orange boxes) 
* Robots only move within a radius of the base
* Once they have reached a certain capacity, they change colour to red and start moving towards the base using PSO (black circle) (read below for how this works)
* If in white, they have reached maximum capacity (but they will still continue moving and picking up debri - to be improved)
* Once in dropoff radius, they drop off their boxes (which turn black) and start roaming again
* After a given number of drop-offs, base starts moving slowly - robutts move with it to next section of the beach to clean
* If a robutt goes outside of the allowed radius, stops moving and turns black 

## About returning to the base
* Robots do not have a system of geo-localization
* Base emits some type of signal (eg. UWB) that robutts can read accurately and use to calculate their distance with respect to the base
* When they need to return to the base, they do the following algorithm
```
move in random direction
if signal_now is better than signal_before, keep direction
if signal_before is better than signal_now, change direction
```

Note: code was developed from https://github.com/hogg377/evo_demo swarm simulation
