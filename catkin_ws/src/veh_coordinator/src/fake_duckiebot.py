#!/usr/bin/env python
from __future__ import print_function

import Tkinter as tk

import rospy
from duckietown_msgs.msg import ControlMode, IntersectionDetection, VehicleDetection, TrafficLightDetection, \
    CoordinationClearance, RoofLight


class FakeDuckiebot:
    def __init__(self):

        # publishing
        self.mode_pub = rospy.Publisher('mode', ControlMode, queue_size=10)
        self.mode = ControlMode.LANE_FOLLOWING

        self.intersection_pub = rospy.Publisher('intersection_detection', IntersectionDetection, queue_size=10)
        self.intersection = IntersectionDetection.NONE

        self.traffic_light_pub = rospy.Publisher('traffic_light_detection', TrafficLightDetection, queue_size=10)
        self.traffic_light = TrafficLightDetection.NA

        self.right_veh_pub = rospy.Publisher('right_vehicle_detection', VehicleDetection, queue_size=10)
        self.right_veh = VehicleDetection.NA

        self.opposite_veh_pub = rospy.Publisher('opposite_vehicle_detection', VehicleDetection, queue_size=10)
        self.opposite_veh = VehicleDetection.NA

        self.intersection_veh_pub = rospy.Publisher('intersection_vehicle_detection', VehicleDetection, queue_size=10)
        self.intersection_veh = VehicleDetection.NA

        # subscribing
        self.clearance_to_go = CoordinationClearance.NA
        self.clearance_to_go_sub = rospy.Subscriber('clearance_to_go', CoordinationClearance,
                                                    self.clearance_to_go_callback)

        self.roof_light =  RoofLight.NA
        self.clearance_to_go_sub = rospy.Subscriber('roof_light',
                                                    RoofLight, self.roof_light_callback)

        self.gui = None

        self.node = rospy.init_node('fake_duckiebot', anonymous=True)

    def clearance_to_go_callback(self, msg):
        self.clearance_to_go = msg.status

    def roof_light_callback(self, msg):
        self.roof_light = msg.color

    def set_gui(self, gui):
        self.gui = gui
        self.update_gui(gui)

    def spin(self):
        self.publish()

    def publish(self):
        self.mode_pub.publish(ControlMode(mode=self.mode))
        self.intersection_pub.publish(IntersectionDetection(type=self.intersection))
        self.traffic_light_pub.publish(TrafficLightDetection(color=self.traffic_light))

        self.right_veh_pub.publish(VehicleDetection(vehicle_detected=self.right_veh != VehicleDetection.NA,
                                                    roof_light=self.right_veh))

        self.opposite_veh_pub.publish(VehicleDetection(vehicle_detected=self.opposite_veh != VehicleDetection.NA,
                                                    roof_light=self.opposite_veh))

        self.intersection_veh_pub.publish(VehicleDetection(vehicle_detected=self.intersection_veh != VehicleDetection.NA,
                                                        roof_light=self.intersection_veh))

    def set_mode(self, mode):
        self.mode = mode
        if self.gui:
            self.update_gui(self.gui)
        self.publish()

    def set_intersection(self, status):
        self.intersection = status
        if self.gui:
            self.update_gui(self.gui)
        self.publish()

    def set_traffic_light(self, status):
        self.traffic_light = status
        if self.gui:
            self.update_gui(self.gui)
        self.publish()

    def set_right_vehicle(self, status):
        self.right_veh = status
        if self.gui:
            self.update_gui(self.gui)
        self.publish()

    def set_opposite_vehicle(self, status):
        self.opposite_veh = status
        if self.gui:
            self.update_gui(self.gui)
        self.publish()

    def set_intersection_vehicle(self, status):
        self.intersection_veh = status
        if self.gui:
            self.update_gui(self.gui)
        self.publish()

    def update_gui(self, gui):
        print("updating gui")
        if self.mode == ControlMode.LANE_FOLLOWING:
            gui.mode_var.set('Mode: LANE_FOLLOWING')
        if self.mode == ControlMode.COORDINATION_CONTROL:
            gui.mode_var.set('Mode: COORDINATION_CONTROL')
        if self.mode == ControlMode.INTERSECTION_CONTROL:
            gui.mode_var.set('Mode: INTERSECTION_CONTROL')

        if self.intersection == IntersectionDetection.NONE:
            gui.intersection_var.set('Int: NONE')
        if self.intersection == IntersectionDetection.STOP:
            gui.intersection_var.set('Int: STOP')
        if self.intersection == IntersectionDetection.TRAFFIC_LIGHT:
            gui.intersection_var.set('Int: TRAFFIC LIGHT')

        values = ['TL: NA', 'TL: NONE', 'TL: GREEN', 'TL: YELLOW', 'TL: RED']
        gui.traffic_light_var.set(values[self.traffic_light+1])

        values = ['RV: NA', 'RV: OFF', 'RV: GREEN', 'RV: YELLOW', 'RV: RED']
        gui.right_veh_var.set(values[self.right_veh+1])

        values = ['OV: NA', 'OV: OFF', 'OV: GREEN', 'OV: YELLOW', 'OV: RED']
        gui.opposite_veh_var.set(values[self.opposite_veh+1])

        values = ['IV: NA', 'IV: OFF', 'IV: GREEN', 'IV: YELLOW', 'IV: RED']
        gui.intersection_veh_var.set(values[self.intersection_veh+1])

        if self.clearance_to_go == CoordinationClearance.NA:
            gui.clearance_to_go_var.set('Clearance: NA')
        if self.clearance_to_go == CoordinationClearance.GO:
            gui.clearance_to_go_var.set('Clearance: GO')
        if self.clearance_to_go == CoordinationClearance.WAIT:
            gui.clearance_to_go_var.set('Clearance: WAIT')

        if self.roof_light == RoofLight.NA:
            gui.roof_light_var.set('Roof: NA')
        if self.roof_light == RoofLight.GREEN:
            gui.roof_light_var.set('Roof: GREEN')
        if self.roof_light == RoofLight.RED:
            gui.roof_light_var.set('Roof: RED')
        if self.roof_light == RoofLight.YELLOW:
            gui.roof_light_var.set('Roof: YELLOW')

class GUI:
    def __init__(self, duckiebot):
        self.root = tk.Tk()
        self.duckiebot = duckiebot

        self.mode_var = tk.StringVar()
        self.mode_label = tk.Label(self.root, textvariable=self.mode_var)
        self.mode_label.pack(side=tk.TOP)


        tk.Button(self.root, text='Lane Navigation',
                  command=lambda: self.duckiebot.set_mode(ControlMode.LANE_FOLLOWING)).pack(side=tk.TOP)

        tk.Button(self.root, text='Coordination',
                  command=lambda: self.duckiebot.set_mode(ControlMode.COORDINATION_CONTROL)).pack(side=tk.TOP)

        tk.Button(self.root, text='Intersection',
                  command=lambda: self.duckiebot.set_mode(ControlMode.INTERSECTION_CONTROL)).pack(side=tk.TOP)

        self.intersection_var = tk.StringVar()
        tk.Label(self.root, textvariable=self.intersection_var).pack(side=tk.TOP)

        tk.Button(self.root, text='Int: None',
                  command=lambda: self.duckiebot.set_intersection(IntersectionDetection.NONE))\
            .pack(side=tk.TOP)

        tk.Button(self.root, text='Int: Stop',
                  command=lambda: self.duckiebot.set_intersection(IntersectionDetection.STOP))\
            .pack(side=tk.TOP)

        tk.Button(self.root, text='Int: Tr. Light',
                  command=lambda: self.duckiebot.set_intersection(IntersectionDetection.TRAFFIC_LIGHT))\
            .pack(side=tk.TOP)

        self.traffic_light_var = tk.StringVar()
        tk.Label(self.root, textvariable=self.traffic_light_var).pack(side=tk.TOP)

        tk.Button(self.root, text='TL: None',
                  command=lambda: self.duckiebot.set_traffic_light(TrafficLightDetection.NA))\
            .pack(side=tk.TOP)

        tk.Button(self.root, text='TL: Green',
                  command=lambda: self.duckiebot.set_traffic_light(TrafficLightDetection.GREEN))\
            .pack(side=tk.TOP)


        tk.Button(self.root, text='TL: Yellow',
                  command=lambda: self.duckiebot.set_traffic_light(TrafficLightDetection.YELLOW))\
            .pack(side=tk.TOP)

        tk.Button(self.root, text='TL: Red',
                  command=lambda: self.duckiebot.set_traffic_light(TrafficLightDetection.RED))\
            .pack(side=tk.TOP)


        self.right_veh_var = tk.StringVar()
        tk.Label(self.root, textvariable=self.right_veh_var).pack(side=tk.TOP)

        tk.Button(self.root, text='RVeh: NA',
                  command=lambda: self.duckiebot.set_right_vehicle(VehicleDetection.NA))\
            .pack(side=tk.TOP)

        tk.Button(self.root, text='RVeh: GREEN',
                  command=lambda: self.duckiebot.set_right_vehicle(VehicleDetection.GREEN))\
            .pack(side=tk.TOP)

        tk.Button(self.root, text='RVeh: YELLOW',
                  command=lambda: self.duckiebot.set_right_vehicle(VehicleDetection.YELLOW))\
            .pack(side=tk.TOP)

        tk.Button(self.root, text='RVeh: RED',
                  command=lambda: self.duckiebot.set_right_vehicle(VehicleDetection.RED))\
            .pack(side=tk.TOP)

        self.opposite_veh_var = tk.StringVar()
        tk.Label(self.root, textvariable=self.opposite_veh_var).pack(side=tk.TOP)

        tk.Button(self.root, text='OVeh: NA',
                  command=lambda: self.duckiebot.set_opposite_vehicle(VehicleDetection.NA))\
            .pack(side=tk.TOP)

        tk.Button(self.root, text='OVeh: GREEN',
                  command=lambda: self.duckiebot.set_opposite_vehicle(VehicleDetection.GREEN))\
            .pack(side=tk.TOP)

        tk.Button(self.root, text='OVeh: YELLOW',
                  command=lambda: self.duckiebot.set_opposite_vehicle(VehicleDetection.YELLOW))\
            .pack(side=tk.TOP)

        tk.Button(self.root, text='OVeh: RED',
                  command=lambda: self.duckiebot.set_opposite_vehicle(VehicleDetection.RED))\
            .pack(side=tk.TOP)

        self.intersection_veh_var = tk.StringVar()
        tk.Label(self.root, textvariable=self.intersection_veh_var).pack(side=tk.TOP)

        tk.Button(self.root, text='Int. Veh: NA',
                  command=lambda: self.duckiebot.set_intersection_vehicle(VehicleDetection.NA))\
            .pack(side=tk.TOP)

        tk.Button(self.root, text='Int. Veh: GREEN',
                  command=lambda: self.duckiebot.set_intersection_vehicle(VehicleDetection.GREEN))\
            .pack(side=tk.TOP)

        self.clearance_to_go_var = tk.StringVar()
        tk.Label(self.root, textvariable=self.clearance_to_go_var).pack(side=tk.TOP)

        self.roof_light_var = tk.StringVar()
        tk.Label(self.root, textvariable=self.roof_light_var).pack(side=tk.TOP)

        # start infinite topic publishing loop
        self.loop_interval_ms = 500

        def loop():
            self.duckiebot.spin()
            self.root.after(self.loop_interval_ms, loop)

        self.root.after(self.loop_interval_ms, loop)

    def start(self):
        tk.mainloop()

if __name__ == '__main__':
    duckiebot = FakeDuckiebot()
    gui = GUI(duckiebot)
    duckiebot.set_gui(gui)
    gui.start()