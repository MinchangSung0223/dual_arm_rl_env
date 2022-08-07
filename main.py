import gym 
import dual_arm_env
import cv2
import time
env = gym.make("DualArmEnv-v0") 
obs = env.reset()
random_action = env.action_space.sample()
new_obs, reward, done, info = env.step(random_action)
num_steps = 1500
obs = env.reset()
for step in range(num_steps):
    # take random action, but you can also do something more intelligent
    # action = my_intelligent_agent_fn(obs) 
    action = env.action_space.sample()
    # apply the action
    obs, reward, done, info = env.step(action)
    print(reward)    
    # Render the env
    img =  env.render()
    cv2.imshow("RGB",img)
    cv2.waitKey(1)

    # Wait a bit before the next frame unless you want to see a crazy fast video
    
    # If the epsiode is up, then start another one
    if done:
        env.reset()