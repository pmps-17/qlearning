In the training phase, I ran the model with 10 episodes and 50 steps in each episode.


In testing Phase, I ran the model for 10 episodes with 50 steps in each episode.

In my environment the maximum reward is 100, and set the rewardThreshold to 90. Hence if any state gets reward of 100 then I consider it to be successfully mixed.

Sometimes, object is getting thrown out of container, in that case I am resetting the simulation.

--> In testing, when using the Q-Table, the number of successful mixes are 2 over 10 trails and it took 25 sec to complete the run.

--> When performing the random action, the number of successful mixes are 3 over 10 trails and it took 27 sec to complete the run.