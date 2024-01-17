# PROJECT TITLE: Autonomous Mover 

## Difference between Basic Robot and AI-Based Robot:

The basic difference between an AI robot and a normal robot lies in the ability of the AI robot to make decisions, learn, and adapt based on data from sensors.

## Aim Overview:

To pick up specified objects from a space, place them in a designated place, while avoiding obstacles and other objects.

## Software Requirements:

- Oracle Virtual Box
- Ubuntu
- Robot Operating System
- Python Functionality

## Hardware Requirements:

- Ras-Berry Pi-3 (Robot Brain)
- Arduino Mega 2560 (Micro Controller)

## What is AI according to this Project:

AI, in this project, refers to a program that learns from examples to recognize objects and adapt to different situations.

## What is Robot according to this Project:

A robot is a machine capable of sensing and reacting to its environment with some human or animal-like function.

## Problem Statements:

- User interaction with the robot, and the robot understanding the intent of the command.
- Identifying and differentiating objects to pick up.
- Determining how to pick up specified objects with a robot arm.
- Avoiding hazards and obstacles.
- Recognizing a box to place the object and navigating back to patrol for more objects.

## Concepts we will follow:

Our robot will have two loops:
- Inner loop or Introspective Loop.
- Outer Loop or Environment Sensing loop.
  
We will follow the OODA (Observe, Orient, Decide, Act) Loop.

## All the AI Tasks we will Perform:

1. **Task 1 (Object Classification):** Recognizing and classifying objects using an artificial neural network.
2. **Task 2 (Avoid Complex Maths):** Using genetic algorithms to let the robot learn to pick up objects on its own.
3. **Task 3 (Use NLP):** Using natural language processing to understand the intent of commands.
4. **Task 4 (Avoid Hazards and Obstacles):** Using operant conditioning for the robot to learn safe movement.
5. **Task 5 (Path Planning):** Implementing decision trees for path planning.
6. **Task 6 (Building Emotions):** Creating an engine for the robot to carry on a small conversation.
7. **Task 7 (Building Personality):** Developing an artificial personality to distinguish it from artificial intelligence.

## Use of Ras Berry Pi and Arduino:

- Ras Berry Pi acts as the main controller communicating with the operator via built-in WiFi.
- Arduino provides the interface to the robot's hardware components like motors and sensors.

## Battery Requirement:

The robot uses a 2700mAh rechargeable battery, similar to those used in quadcopters.

## OODA:

We will use the OODA loop, where tasks are time-controlled, and the lower, inner loop takes priority over the slower, outer loop.

## Control Loops:

Why we need control loops:
- To respond to commands from the control station.
- To interface with motors and sensors in the Arduino Mega.

## Way of Creating Commands:

We will use 3-letter identifiers:
- DRV -> Motor Drive
- ARM -> Robot Arm

## Concept of Heart Beat:

- Using a Heartbeat Logic to maintain positive control over the robot.
- Regularly passing messages as a heartbeat to ensure the robot is responsive.

**Note:**
- The setup involves three devices: Raspberry Pi3, Arduino, and a control program running on a PC.
- Heartbeat messages are used to measure communication performance and ensure the robot is responsive.
- It helps in understanding latency, confirming active connections, and allowing monitoring and control over a wireless network.

`