import gymnasium as gym
import sys866_lib as ps
import math

env = gym.make("Pendulum-v1", render_mode="human")
policy = ps.Policy(consigne=math.pi, consigne_tresh=0.99, dt=0.05)   
loop = ps.EpisodeLoopPendulum(env, math.pi, policy, ps.PID(1.0, 0.0, 0.0001), max_steps=1000)
loop.run()