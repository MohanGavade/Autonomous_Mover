# Robot Arm Machine Learning Methods

## Two Machine Learning Methods:
1. **Genetic Algorithms**
2. **Reinforcement Learning (RL)**

## Utilizing Forward Kinematics
Calculate the hand's location using forward kinematics by summing up all arm angles and levers. This hand location serves as the reward criterion and desired condition.

## Training Steps

### 1. Freeze Arm Base
Remove one degree of freedom by freezing the arm base at the shoulder.

### 2. Define Actions and States
- **Actions:** Commands for robot motors (rotate left, rotate right, no motion).
- **States:** Different starting positions for the robot's hand.

### 3. Associate Actions and States
Teach the robot which actions work best for each state to maximize rewards.

### 4. Probability of Outcome
Learn the likelihood of positive or negative results for each state-action pair.

### 5. Training the Robot
The robot learns by trying different actions in different states and adjusting based on outcomes.

### 6. Discounting the Reward
Reward is attributed to all steps in a sequence, not just the final action.

### 7. Define Policy
Combination of state and action, guiding the robot on what action to take in a specific state.

## Robo Actions to Perform
1. Normal Arm Position (Neutral)
2. Pick up
3. High Carry
4. Drop-Off

## How the Robot Functions

### Goal Position
The robot has a target position for its hand specified by x and y coordinates.

### Learning Movements
The robot tries different movements to get close to the goal, initially with random movements.

### Scoring Movements
The robot assesses whether it got closer to the goal after each small movement.

### Remembering Movements
The robot connects starting positions, actions, and reward scores to remember successful movements.

### Neural Network Training
A neural network is trained to predict the likelihood of positive outcomes based on starting position and movement action.

### Learning Sequences
The robot learns which sequences of movements lead to positive results.

### Prediction
The trained neural network enables the robot to predict likely movements based on its current position.

## Reward Calculation

### Q-Function
The Q-function is used to calculate the expected reward for a particular action in a specific state.

### Components of Q-Function
- **Q(s, a):** Final reward for taking action 'a' in the starting state 's'.
- **reward(s, a):** Immediate reward obtained for action 'a' in state 's'.
- **g (discount function):** Encourages the robot to reach the goal faster by discounting future rewards.
- **max(Q(s', a')):** Selects the action promising the highest reward among available actions in the next state 's'.
  
### Q-Function Equation
Q(s, a) = reward(s, a) + g * max(Q(s', a'))

### Example Scenario
1. Robot is at position s and decides to move left (action a).
2. Receives immediate reward (reward(s, a)) for this action.
3. Considers all possible actions from the new position s' (max(Q(s', a'))).
4. Q-function estimates the overall expected reward (Q(s, a)) for taking action 'a' in state 's'.

## Action Space
Three motors with three options for each (do nothing, move counterclockwise, move clockwise). This results in 27 possible actions (3 x 3 x 3).

Note: Most servo motors treat positive position changes as clockwise rotation.

Each action will have three values. An action matrix represents the possible actions, where each action reduces, holds, or increases the angle of the corresponding motor.

- Example Action: [-1, 0, 1]
  - Reduces the angle of motor 1
  - Holds motor 2 in place
  - Increases the angle of motor 3
