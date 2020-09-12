#!/usr/bin/env python
import subprocess


if __name__ == '__main__':
    bashCommand = "sudo apt install ros-melodic-navigation"
    process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
    output, error = process.communicate()
    print(output, error)