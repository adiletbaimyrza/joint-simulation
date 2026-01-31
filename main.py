#!/usr/bin/env python3

import pygame
pygame.init()

from src.app import JointSimulationApp

if __name__ == "__main__":
    app = JointSimulationApp()
    app.run()
