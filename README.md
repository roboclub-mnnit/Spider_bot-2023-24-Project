# Spider_bot-2023-24-Project
![image](https://github.com/utkarsh1236/Image_to_text_conversion-2022-23-Project/assets/110242820/d318c1dd-fe28-4817-b11e-07643c498637)
## About
The SpiderBot Project is a venture aimed at designing and constructing a fundamental spider robot capable of efficient movement and navigation on flat surfaces. The primary objective is to develop a spider robot with the ability to autonomously traverse its environment, showcasing the integration of mechanical and electronic components for seamless functionality.

The Quadrupled Spider Robot is an innovative four-legged walking robot designed to mimic the movement patterns of spiders. This project leverages servo motors for each leg, providing precise control and coordinated movement. The integration of an Arduino microcontroller, along with the Adafruit_PWMServoDriver library, facilitates efficient control of the servo motors through PWM signal generation.


## Project Group Members
|     Name     |        Branch       | Registration No. |
|:------------:|:------------------:|:----------------:|
| Raj Kushwaha  | Electronics & Communication Engineering |    20224117     |
|   Yasir Khan   |  Mechanical Engineering  |    20226181     |
| Mohammad Kaif  |  Mechanical Engineering  |    20226085     |
| Avinash Gupta  |  Mechanical Engineering  |    20226040     |
## Project Mentors
 *Final Year
|         Name                |             Branch                        | Registration No. |
|:---------------------------|:-----------------------------------------:|:----------------:|
| Abhishek Hanotiya               | Mechanical Engineering |    20203005     |
| Aditya Jain             | Mechanical Engineering         |     20203010     |

 *Pre-Final Year
 |         Name                |             Branch                        | Registration No. |
|:---------------------------|:-----------------------------------------:|:----------------:|
| Devendra Saini             | Mechanical Engineering | 20213032          |
| Pulkit Singhal             | Mechanical Engineering | 20213110        |
| Deepti Dixit               | Chemical Engineering   | 20218038          |
 
## Tech Stack
-**Microcontroller:** Arduino

-**Programming Language:** C++

-**Libraries:** Adafruit_PWMServoDriver

-**Hardware:** Servo Motors, Chassis, Power Supply, Jumper Wires, PCA9685 Servo Motor Driver

## Working Of The Bot 
The Spider Bot’s motion and functionality are achieved through a combination of trajectory planning, inverse kinematics, and the coordinated movement of its servo motors. Below is an elaboration of its working:

#### *Initialization and Setup:
-The Spider Bot uses an Arduino Nano board as its microcontroller.

-An Adafruit PWM Servo Driver is employed to control the numerous SG90 Mini Servo Motors 
 efficiently.
 
-Twelve(12)  SG90 Mini Servo Motors are distributed across the four legs(3 per leg) and body of 
 the Spider Bot.
 
-The default, normal, and current positions of each servo motor are defined in arrays.

#### *Standing Position:
-The Spider Bot is initially positioned in a stable standing position using inverse kinematics.
-The servo motors are tuned for stability while the robot is suspended above the ground with a rope.

#### *Walking Steps:
The walking cycle involves a series of steps, and each step is described as follows:

-**Leg Movement:** The robot bends its knees, allowing one leg to swing above the ground while the other supports its weight.

-**Body Tilt:** The upper body tilts in the opposite direction to provide enough height for the swinging leg.

-**Leg Placement:** Both legs are placed on the ground to support the robot for further walking steps.

-**Leg Swing:** The free leg swings forward, and the hip center trajectory is controlled using trajectory planning equations.

-**Body Re-Centering:** Both legs come together, and the body re-centers to its normal standing position.

#### *Trajectory Planning:
-Trajectory planning involves determining the time series of successive joint angles for coordinated leg and body movements.

-Hyperbolic, elliptical, and straight-line trajectories are used for the hip center, providing controlled, stable, and agile motion.

#### *Inverse Kinematics:
-Inverse kinematics is employed to calculate the required joint angles for the robot to achieve specific positions or motions.

-Joint configurations are determined using closed-form solutions, ensuring stable motion of the Quadrupled Spider Bot.

#### *Turning and Shifting:
-The Spider Bot can execute turns and shifts using a combination of trajectory planning      and inverse kinematics.

-Specific functions(user defined), such as turnRight and centerShift, are utilized for     controlled turning and lateral movements.

#### *Walking Sequence:
The Spider Bot executes a sequence of leg movements, including lifting, swinging, and placing, to achieve forward motion. Coordinated movements of all legs are orchestrated to mimic the walking cycle.

#### *User Interaction:
-The Spider Bot can respond to user commands received through the serial interface.
-Users can adjust the position of individual servo motors in real-time to control the Spider 
 Bot’s motion.

#### *Power Supply and Components:
 The Spider Bot is powered by a battery, and its components include an Arduino Nano, Adafruit PWM Servo Driver, SG90 Mini Servo Motors, chassis, legs, and jumper wires.
 
 In summary, the Spider Bot’s intricate motion is achieved through a combination of precise trajectory planning, inverse kinematics, and coordinated control of its servo motors. The provided code defines various functions and sequences that enable the Spider Bot to exhibit lifelike movements, resembling the walking pattern of a spider.



## Real-Time applications-
### 1. Search and Rescue Operations:
   
-**Capabilities:** Climbing, versatile locomotion.

-**Implementation:** Deployment in disaster-stricken areas for exploring complex terrains,   
  locating survivors, and aiding rescue efforts.
  
### 2. Exploration in Unknown Environments:
   
-**Capabilities:** Adaptability, sensor integration.
   
-**Implementation:** Planetary exploration, underground exploration in caves or mines, 
  leveraging climbing abilities for navigation.

### 3. Surveillance and Monitoring:
 
-**Capabilities:** Stealth, maneuverability, 360-degree vision.

-**Implementation:** Security surveillance in complex environments, environmental monitoring 
  in challenging or remote areas


