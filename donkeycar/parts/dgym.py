import os
import time
import gym
import donkey_gym
import donkeycar as dk

class DonkeyGymEnv(object):

    def __init__(self, sim_path, input_shape, port=9090, headless=0, env_name="donkey-generated-track-v0", sync="asynchronous"):
        os.environ['DONKEY_SIM_PATH'] = sim_path
        os.environ['DONKEY_SIM_PORT'] = str(port)
        os.environ['DONKEY_SIM_HEADLESS'] = str(headless)
        os.environ['DONKEY_SIM_SYNC'] = str(sync)

        self.env = gym.make(env_name)
        self.frame = self.env.reset()
        self.action = [0.0, 0.0]
        self.running = True
        self.input_shape = input_shape
        if self.input_shape[2] == 1:
            self.frame = dk.utils.rgb2gray(self.frame).reshape(input_shape)            
            print("self.frame.shape", self.frame.shape)

    def update(self):
        while self.running:
            frame, reward, done, info = self.env.step(self.action)

            if frame.shape != self.input_shape:
                if self.input_shape[2] == 1:
                    frame = dk.utils.rgb2gray(frame).reshape(self.input_shape)
                    
            self.frame = frame


    def run_threaded(self, steering, throttle):
        if steering is None or throttle is None:
            steering = 0.0
            throttle = 0.0
        self.action = [steering, throttle]
        return self.frame

    def shutdown(self):
        self.running = False
        time.sleep(0.2)
        self.env.close()


    
