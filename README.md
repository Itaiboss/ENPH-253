# ENPH-253 Introduction to Instrument Design

### Competition Goal
- 6 week project course to design an autonomous robot from scratch that can navigate a Mario Kart themed race course while collecting points by completing laps, dodging obstacles, and going head-to-head with an opposing robot.

<img width="611" alt="Screenshot 2024-01-19 at 10 15 56 AM" src="https://github.com/Itaiboss/ENPH-253/assets/90986809/6f37d0f7-93bf-4f5d-86fb-7e45bd96a3b2">

### Our Solution 
Our approach was to find the shortest possible path along the course and completing the most laps within the 2-minute time limit. In order to do so, our robot needed to be as light and agile as possible, as well as taking advantage of shortcuts on the course.
<img width="601" alt="Screenshot 2024-01-19 at 10 16 02 AM" src="https://github.com/Itaiboss/ENPH-253/assets/90986809/9274421e-b3f7-4137-8fd1-9eadbba95fee">

| Mech | Elec | Software |
|----------------------|--------|------|
| <ul><li> Modelled, fabricated/machined and tested all mechanical components. </li><li> Implemented rack and pinion servo steering, as well as front bumper for knocking away obstacles and mitigating impact forces.<li> optimized weight for a sub 1.5 kg design to increase max speed while also keeping a robust chassis for handling continual impacts</li> </ul> | <ul><li> Designed circuits schematics for tape and IR sensor, H-bridge for motor control, as well as boards for power management and interfacing with the stm-32 microcontroller.</li><li> Designed custom PCB's for all circuits to increase space efficiency while allowing for modularity and easy repairs. </li><li> Optimized power consumption to decrease battery weight<li> Optocoupled signals to H-bridge to mitigate noise from motors</li></ul> | <ul><li> Designed finite state machine to take inputs and determine various state based actions</li> </li><li> Implemented PID control algorithms for tape and IR following</li> <li> Designed system to interface with and zero IMU to determine rotation/position charecteristics </li> <li>Designed IR frequency sampling and detection algorithm using FFTs, to differentiate and follow specific IR frequencies</ul>|

## Media 
Internal electronics
![IMG_0563](https://github.com/Itaiboss/ENPH-253/assets/90986809/25e30a97-18c5-4083-8532-34ca8bb35f51)
Outer Electronics
![IMG_0596](https://github.com/Itaiboss/ENPH-253/assets/90986809/8d14ea97-f650-418b-85a3-3fed9f29da9a)
Bottom View 
![IMG_0598](https://github.com/Itaiboss/ENPH-253/assets/90986809/96ad9327-d29e-433a-8021-72b7dd501840)


### Steering System test video
https://github.com/Itaiboss/ENPH-253/assets/90986809/176ecf7e-b65b-4f1c-b625-0f59ca0cb1a9

### Pre-comp jump testing
https://github.com/Itaiboss/ENPH-253/assets/90986809/6d90ce66-5be8-4b95-b07e-0ac00a035467

### Competition day run
https://github.com/Itaiboss/ENPH-253/assets/90986809/04df8977-5c3b-4841-8c5f-93d6f6c0546f



