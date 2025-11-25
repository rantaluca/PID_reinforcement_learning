import gymnasium as gym
import sys866_lib as ps

print("Starting Pendulum-v1 with PID controller")
print("initial pid values:")

kp = input("Kp: ")
ki = input("Ki: ")
kd = input("Kd: ")

env = gym.make("Pendulum-v1", render_mode="human")
policy = ps.Policy(consigne=0.0, consigne_tresh=0.99, dt=0.01)   
loop = ps.EpisodeLoopPendulum(env, 0.0, policy, ps.PID(float(kp), float(ki), float(kd)), max_steps=1000)
loop.run()