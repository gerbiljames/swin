#!/usr/bin/env python
from subprocess import Popen
from os.path import expanduser

# set your python command here
python_command = expanduser("python")


def generate_calibration_beta_data():

    for beta in [2, 4, 6, 8, 10, 12, 14, 16, 18]:
        processes = []
        for seed in range(4):

            processes.append(Popen([python_command, "Pi-Swarm-Sim.py", "--taxis_algorithm=beta",
                                    "--beta=" + str(beta), "--robots=20", "--experiment", "--headless", "--seed=" + str(seed)]))

        [process.wait() for process in processes]


def generate_calibration_omega_data():

    for omega in [20, 25, 30, 35, 40]:
        processes = []
        for seed in range(4):

            processes.append(Popen([python_command, "Pi-Swarm-Sim.py", "--taxis_algorithm=omega",
                                    "--omega=" + str(omega), "--robots=20", "--experiment", "--headless", "--seed=" + str(seed)]))

        [process.wait() for process in processes]


def generate_beta_comparison_data():

    for group in range(1, 4):
        processes = []
        for seed in range(group * 4, group * 4 + 4):

            processes.append(Popen([python_command, "Pi-Swarm-Sim.py", "--taxis_algorithm=beta",
                                    "--beta=2", "--robots=20", "--log_advanced", "--headless", "--seed=" + str(seed)]))

        [process.wait() for process in processes]


def generate_omega_comparison_data():

    for group in range(1, 4):
        processes = []
        for seed in range(group * 4, group * 4 + 4):

            processes.append(Popen([python_command, "Pi-Swarm-Sim.py", "--taxis_algorithm=omega",
                                    "--omega=35", "--robots=20", "--log_advanced", "--headless", "--seed=" + str(seed)]))

        [process.wait() for process in processes]


generate_calibration_beta_data()
generate_calibration_omega_data()
generate_beta_comparison_data()
generate_omega_comparison_data()