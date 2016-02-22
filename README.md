# QLearning-NAO_plays_Agario
ROS compatible reinforcement learning algorithm based on Q-Learning that enables a NAO robot play the [Agar.io](http://agar.io/) game. The game state acquisition (cell's positions, radius, etc) is done with computer vision because the only allowed data input is the robot camera.

## Video
<a href="http://www.youtube.com/watch?feature=player_embedded&v=ihTgj4SA-ME
" target="_blank"><img src="http://img.youtube.com/vi/ihTgj4SA-ME/0.jpg" 
alt="Reinforcement learning - Nao robot plays Agar.io" width="240" height="180" border="10" /></a>


## Q-Learning
Q(x(t-1), u(t-1)) = random(\beta, exploitation, exploration) 
* exploitation: Q(x(t-1), u(t-1)) = IR + \gamma * Max[Q((x(t), U(t))]
* exploration: Q(x(t-1), u(t-1)) = IR + \gamma * Q(x(t), random(u(t) | u(t) \in U(t))
* instantaneous reward: IR = R[x(t-1), u(t-1)] + R_still_alive + R_eat_pellet + R_eat_enemy

## TODO
1. Improve game state acquisition:
  * Improve segmentation for occlusions and overlapings.
  * Improve Ray Trace algorithm.
2. Decrease memory usage.
 
