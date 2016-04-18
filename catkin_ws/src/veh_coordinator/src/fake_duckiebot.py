#!/usr/bin/env python
from __future__ import print_function

import Tkinter as tk

import rospy
from duckietown_msgs.msg import FSMState, IntersectionDetection, VehicleDetection, TrafficLightDetection, \
    CoordinationClearance, CoordinationSignal


class FakeDuckiebot:
    def __init__(self):
        self.node = rospy.init_node('fake_duckiebot', anonymous=True)

        # publishing
        self.mode = FSMState.LANE_FOLLOWING
        self.mode_pub = rospy.Publisher('~mode', FSMState, queue_size=10)

        self.intersection_pub = rospy.Publisher('~intersection_detection', IntersectionDetection, queue_size=10)
        self.intersection = IntersectionDetection.NONE

        self.traffic_light_pub = rospy.Publisher('~traffic_light_detection', TrafficLightDetection, queue_size=10)
        self.traffic_light = TrafficLightDetection.NA

        self.right_veh_pub = rospy.Publisher('~right_vehicle_detection', VehicleDetection, queue_size=10)
        self.right_veh = VehicleDetection.NO_CAR

        self.opposite_veh_pub = rospy.Publisher('~opposite_vehicle_detection', VehicleDetection, queue_size=10)
        self.opposite_veh = VehicleDetection.NO_CAR

        # subscribing
        self.clearance_to_go = CoordinationClearance.NA
        self.clearance_to_go_sub = rospy.Subscriber('~clearance_to_go', CoordinationClearance,
                                                    self.clearance_to_go_callback)

        self.roof_light = CoordinationSignal.SIGNAL_A
        self.clearance_to_go_sub = rospy.Subscriber('~coordination_signal',
                                                    CoordinationSignal, self.roof_light_callback)

        self.gui = None


    def clearance_to_go_callback(self, msg):
        self.clearance_to_go = msg.status
        if self.gui:
            self.update_gui(self.gui)

    def roof_light_callback(self, msg):
        self.roof_light = msg.signal
        if self.gui:
            self.update_gui(self.gui)

    def set_gui(self, gui):
        self.gui = gui
        self.update_gui(gui)

    def spin(self):
        self.publish()

    def publish(self):
        self.mode_pub.publish(FSMState(state=self.mode))
        self.intersection_pub.publish(IntersectionDetection(type=self.intersection))
        self.traffic_light_pub.publish(TrafficLightDetection(color=self.traffic_light))

        self.right_veh_pub.publish(VehicleDetection(detection=self.right_veh))

        self.opposite_veh_pub.publish(VehicleDetection(detection=self.opposite_veh))

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
        if self.mode == FSMState.LANE_FOLLOWING:
            gui.mode_var.set('Mode: LANE_FOLLOWING')
        if self.mode == FSMState.COORDINATION:
            gui.mode_var.set('Mode: COORDINATION_CONTROL')
        if self.mode == FSMState.INTERSECTION_CONTROL:
            gui.mode_var.set('Mode: INTERSECTION_CONTROL')

        if self.intersection == IntersectionDetection.NONE:
            gui.intersection_var.set('Int: NONE')
        if self.intersection == IntersectionDetection.STOP:
            gui.intersection_var.set('Int: STOP')
        if self.intersection == IntersectionDetection.TRAFFIC_LIGHT:
            gui.intersection_var.set('Int: TRAFFIC LIGHT')

        values = ['TL: NA', 'TL: NONE', 'TL: GREEN', 'TL: YELLOW', 'TL: RED']
        gui.traffic_light_var.set(values[self.traffic_light+1])

        if self.right_veh == 0:
            gui.right_veh_var.set("RV: No Car")
        else:
            values = ['RV: A', 'RV: B', 'RV: C']
            gui.right_veh_var.set(values[self.right_veh-11])

        if self.opposite_veh == 0:
            gui.opposite_veh_var.set("OV: No Car")
        else:
            values = ['OV: A', 'OV: B', 'OV: C']
            gui.opposite_veh_var.set(values[self.opposite_veh-11])

        values = ['Clearance: NA', 'Clearance: WAIT', 'Clearance: GO']
        gui.clearance_to_go_var.set(values[self.clearance_to_go+1])

        values = ['Roof: A', 'Roof: B', 'Roof: C']
        gui.roof_light_var.set(values[self.roof_light-11])


class GUI:
    def __init__(self, duckiebot):
        self.root = tk.Tk()
        self.duckiebot = duckiebot

        self.mode_var = tk.StringVar()
        self.mode_label = tk.Label(self.root, textvariable=self.mode_var)
        self.mode_label.pack(side=tk.TOP)

        tk.Button(self.root, text='Lane Navigation',
                  command=lambda: self.duckiebot.set_mode(FSMState.LANE_FOLLOWING)).pack(side=tk.TOP)

        tk.Button(self.root, text='Coordination',
                  command=lambda: self.duckiebot.set_mode(FSMState.COORDINATION)).pack(side=tk.TOP)

        tk.Button(self.root, text='Intersection Nav.',
                  command=lambda: self.duckiebot.set_mode(FSMState.INTERSECTION_CONTROL)).pack(side=tk.TOP)

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
                  command=lambda: self.duckiebot.set_right_vehicle(VehicleDetection.NO_CAR))\
            .pack(side=tk.TOP)

        tk.Button(self.root, text='RVeh: A',
                  command=lambda: self.duckiebot.set_right_vehicle(VehicleDetection.SIGNAL_A))\
            .pack(side=tk.TOP)

        tk.Button(self.root, text='RVeh: B',
                  command=lambda: self.duckiebot.set_right_vehicle(VehicleDetection.SIGNAL_B))\
            .pack(side=tk.TOP)

        tk.Button(self.root, text='RVeh: C',
                  command=lambda: self.duckiebot.set_right_vehicle(VehicleDetection.SIGNAL_C))\
            .pack(side=tk.TOP)

        self.opposite_veh_var = tk.StringVar()
        tk.Label(self.root, textvariable=self.opposite_veh_var).pack(side=tk.TOP)

        tk.Button(self.root, text='OVeh: NA',
                  command=lambda: self.duckiebot.set_opposite_vehicle(VehicleDetection.NO_CAR))\
            .pack(side=tk.TOP)

        tk.Button(self.root, text='OVeh: A',
                  command=lambda: self.duckiebot.set_opposite_vehicle(VehicleDetection.SIGNAL_A))\
            .pack(side=tk.TOP)

        tk.Button(self.root, text='OVeh: B',
                  command=lambda: self.duckiebot.set_opposite_vehicle(VehicleDetection.SIGNAL_B))\
            .pack(side=tk.TOP)

        tk.Button(self.root, text='OVeh: C',
                  command=lambda: self.duckiebot.set_opposite_vehicle(VehicleDetection.SIGNAL_C))\
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