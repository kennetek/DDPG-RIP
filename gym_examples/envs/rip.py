__credits__ = ["Kenneth Hodson"]

from typing import Optional
import gym
from gym import spaces
import random
import matlab.engine
import matplotlib.pyplot as plt
import numpy as np

name = 'eom'
obs = ['theta', 'alpha', 'theta_dot', 'alpha_dot']
work = {
    "ic_alpha0": np.pi
}


class Simulink:
    def __init__(self, model_name, controller):
        self.modelName = model_name  # The name of the Simulink Model (To be placed in the same directory)
        self.controller = controller
        # Logging the variables
        self.yHist = [0 for i in range(len(obs))]
        self.tHist = 0

        print("Starting matlab")
        self.eng = matlab.engine.start_matlab()
        print("Connected to Matlab")

    def setControlAction(self, u):
        # Helper Function to set value of control action
        self.eng.set_param('{}/u'.format(self.modelName), 'value', str(u), nargout=0)

    def getHistory(self):
        # Helper Function to get Plant Output and Time History
        return [self.out(i) for i in obs], self.eng.eval('out.tout')

    def getState(self):
        hisy, hist = self.getHistory()
        return [np.double(hisy[i][-1][0]) for i in range(len(obs))]

    def out(self, n):
        return self.eng.eval(f'out.{n}')

    def start(self):
        self.eng.clear(nargout=0)

        # Load the model
        self.eng.eval("model = '{}'".format(self.modelName), nargout=0)
        self.eng.eval("load_system(model)", nargout=0)
        for key in work:
            self.eng.workspace[key] = work[key]
        self.eng.load('constants.mat', nargout=0)

        # Initialize Control Action to 0
        self.setControlAction(0)
        print("Starting simulation...")

        # Start Simulation and then Instantly pause
        self.eng.set_param(self.modelName, 'SimulationCommand', 'start', 'SimulationCommand', 'pause', nargout=0)
        self.yHist, self.tHist = self.getHistory()

    def simulate(self):
        # Control Loop
        while self.step(0):
            continue

    def step(self, u):
        status = self.eng.get_param(self.modelName, 'SimulationStatus') != ('stopped' or 'terminating')

        # if not status:
         #   return False

        # Generate the Control action based on the past outputs
        # u = self.controller.getControlEffort(self.yHist, self.tHist)

        # Set that Control Action
        self.setControlAction(u)

        # Pause the Simulation for each timestep
        self.eng.set_param(self.modelName, 'SimulationCommand', 'continue', 'SimulationCommand', 'pause', nargout=0)

        self.yHist, self.tHist = self.getHistory()

        return True

    def stop(self):
        print("Stopping simulation...")
        self.eng.set_param(self.modelName, 'SimulationCommand', 'stop', nargout=0)

    def disconnect(self):
        self.eng.set_param(self.modelName, 'SimulationCommand', 'stop', nargout=0)
        self.eng.quit()


class Controller:

    def __init__(self):
        # Maintain a History of Variables
        self.yHist = [[] for i in range(len(obs))]
        self.tHist = []
        self.uHist = []

    def getControlEffort(self, yhist, thist):

        # Returns control action based on past outputs

        for i in range(len(obs)):
            self.yHist[i] = yhist[i]
        self.tHist = thist

        if type(self.yHist[0]) == float:
            y = self.yHist[0]
        else:
            y = self.yHist[0][-1][0]

        u = 0

        print(y)
        self.uHist.append(u)
        return u

    def plot(self):
        plt.plot(self.tHist, self.yHist[0])
        plt.plot(self.tHist, self.yHist[2])
        # plt.xlim(0, np.amax(control.tHist))
        # plt.ylim(0, np.amax(control.yHist)*1.1)
        plt.ylabel("Plant Output")
        plt.xlabel("Time(s)")
        plt.title("Plant Response")
        plt.show()


class RotaryInvertedPendulumEnv(gym.Env):
    metadata = {
        "render_modes": [],
        "render_fps": 30,
    }

    def __init__(self, render_mode: Optional[str] = None):
        self.render_mode = render_mode
        self.last_u = None
        self.count = 0
        self.max_count = 200
        self.state = [0, 0, 0, 0]

        self.max_voltage = 5
        self.max_speed = 2 * np.pi
        self.max_theta = 60 * np.pi / 180
        self.max_alpha = 2 * np.pi

        self.action_space = spaces.Box(
            low=-self.max_voltage, high=self.max_voltage, shape=(1,), dtype=np.float32
        )

        # theta, theta_dot, alpha, alpha_dot
        high = np.array([self.max_theta, self.max_speed, self.max_alpha, self.max_speed], dtype=np.float32)
        self.observation_space = spaces.Box(low=-high, high=high, dtype=np.float32)

        self.control = Controller()
        self.plant = Simulink(model_name=name, controller=self.control)
        self.plant.start()

    def step(self, u):
        th, al, thdot, aldot = self.state

        u = np.clip(u, -self.max_voltage, self.max_voltage)[0]
        self.last_u = u  # for rendering

        alpha_bal_threshold = 10 * np.pi / 180
        q11 = 10
        q22 = 15
        q33 = 0
        q44 = 1
        r = 0.1
        b = 200

        fail = abs(th) >= self.max_theta or abs(al) >= alpha_bal_threshold or abs(u) >= self.max_voltage

        # reward function
        costs = q11 * (th ** 2) + q22 * (al ** 2) + q33 * (thdot ** 2) + q44 * (aldot ** 2) + r * (u ** 2) + \
                fail*b*(self.max_count-self.count)/self.max_count
        # print(f'Costs:-{costs}')
        # print(f'Failed? {fail}')
        # print(f'Input Voltage: {u}')
        print(f'State:{self.state}')
        self.count += 1

        # run matlab step here
        terminated = self.plant.step(u)

        # set new state after step here
        self.state = self.plant.getState()

        # return observation, reward, terminated, info
        return self.state, -costs, not terminated or fail or self.count > self.max_count, {}

    def reset(self, seed=None, options=None):
        self.count = 0
        self.plant.stop()
        a0 = (20 * (random.random() - 0.5) + 180) * np.pi / 180
        work['ic_alpha0'] = a0
        self.state = [0, np.mod(a0, 2*np.pi)-np.pi, 0, 0]
        self.plant.start()
        self.last_u = None

        return self.state

    def render(self, mode="human"):
        return

    def close(self):
        self.plant.stop()
        self.plant.disconnect()
