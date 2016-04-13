#!/usr/bin/env python

import rospy, sys, os, cv2, pickle
from graph import Graph
from graph_search import GraphSearchProblem
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from navigation.srv import *

class graph_search_server():
    def __init__(self):
        print 'Graph Search Service Started'

        # Inputs
        self.map_name = rospy.get_param('/map_name')

        # Loading map
        self.script_dir = os.path.dirname(__file__)
        self.map_path = self.script_dir + '/maps/' + self.map_name
        try:
	        file2 = open(self.map_path + '.pkl', 'r')
        except IOError:
	        print "Couldn't find your map:", self.map_path, ". Closing program..."
	        sys.exit(0)
	
        map_data = pickle.load(file2)
        file2.close()

        # Create graph
        self.duckietown_graph = Graph()
        edges = map_data[0]
        node_locations = map_data[1]
        for edge in edges:
	        self.duckietown_graph.add_edge(edge[0], edge[1], edge[2], edge[3])
	
        self.duckietown_graph.set_node_positions(node_locations)
        self.duckietown_problem = GraphSearchProblem(self.duckietown_graph, None, None)

        print "Map loaded successfully!\n"

        self.image_pub = rospy.Publisher("~map_graph",Image, queue_size = 1, latch=True)
        self.bridge = CvBridge()

        # Send graph through publisher
        self.duckietown_graph.draw(highlight_edges=None, map_name = self.map_name)
        cv_image = cv2.imread(self.map_path + '.png', cv2.CV_LOAD_IMAGE_COLOR)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

    def handle_graph_search(self,req):
	    # Checking if nodes exists
        if (req.source_node not in self.duckietown_graph) or (req.target_node not in self.duckietown_graph):
            print "Source or target node do not exist."
            return GraphSearchResponse([])

        # Running A*
        self.duckietown_problem.start = req.source_node
        self.duckietown_problem.goal = req.target_node
        path = self.duckietown_problem.astar_search()

        # Publish graph solution
        if path:
            self.duckietown_graph.draw(highlight_edges=path.edges(), map_name = self.map_name, highlight_nodes = [req.source_node, req.target_node])
            cv_image = cv2.imread(self.map_path + '.png', cv2.CV_LOAD_IMAGE_COLOR)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

        return GraphSearchResponse(path.actions)        

if __name__ == "__main__":	
    rospy.init_node('graph_search_server')
    gss = graph_search_server()
    print 'Starting server...\n'
    s = rospy.Service('graph_search', GraphSearch, gss.handle_graph_search)
    rospy.spin()
