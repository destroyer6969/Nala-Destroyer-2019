#!/usr/bin/env python

class Mision:
    def __init__(self):
        self.speed      = 1500
        self.waypoint   = 0
        self.stop       = 0

    def changeStop(self, stop):
        self.stop = stop

    def changeWaypoint(self, waypoint):
        self.waypoint = waypoint

    def changeSpeed(self, speed):
        self.speed = speed
