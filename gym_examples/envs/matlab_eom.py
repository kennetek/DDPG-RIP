# source:
# https://medium.com/@soutrikbandyopadhyay/controlling-a-simulink-model-by-a-python-controller-2b67bde744ee

import matlab.engine
import matplotlib.pyplot as plt
import numpy as np

name = 'eom'
obs = ['theta', 'alpha', 'theta_dot', 'alpha_dot']
work = {
    "theta_max": 60*np.pi/180,
    "alpha0": 0.0
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
        his = self.getHistory()
        return [his[i][-1] for i in range(len(obs))]

    def out(self, n):
        return self.eng.eval(f'out.{n}')

    def start(self):
        self.eng.clear(nargout=0)

        # Load the model
        self.eng.eval("model = '{}'".format(self.modelName), nargout=0)
        self.eng.eval("load_system(model)", nargout=0)
        for key in work:
            self.eng.workspace[key] = work[key]

        # Initialize Control Action to 0
        self.setControlAction(0)
        print("Initialized Model")

        # Start Simulation and then Instantly pause
        self.eng.set_param(self.modelName, 'SimulationCommand', 'start', 'SimulationCommand', 'pause', nargout=0)
        self.yHist, self.tHist = self.getHistory()

    def simulate(self):
        # Control Loop
        while self.step():
            continue

    def step(self, u):
        status = self.eng.get_param(self.modelName, 'SimulationStatus') != ('stopped' or 'terminating')

        if not status:
            return False

        # Generate the Control action based on the past outputs
        # u = self.controller.getControlEffort(self.yHist, self.tHist)

        # Set that Control Action
        self.setControlAction(u)

        # Pause the Simulation for each timestep
        self.eng.set_param(self.modelName, 'SimulationCommand', 'continue', 'SimulationCommand', 'pause', nargout=0)

        self.yHist, self.tHist = self.getHistory()

        return True

    def stop(self):
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


"""
control = Controller()
plant = Simulink(model_name=name, controller=control)
plant.start()
plant.simulate()
plant.stop()

work['alpha0'] = 175*np.pi/180
plant.start()
plant.simulate()
plant.stop()

plant.disconnect()

control.plot()
"""


