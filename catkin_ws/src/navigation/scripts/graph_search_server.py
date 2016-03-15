#!/usr/bin/env python

from navigation.srv import *
import rospy

import pickle
from graph import Graph
from graph_search import GraphSearchProblem

duckietown_graph = 0

def handle_graph_search(req):
	# Running A*
	
	global duckietown_graph
	
	duckietown_problem = GraphSearchProblem(duckietown_graph, req.source_node, req.target_node)
	path = duckietown_problem.astar_search()

	# Return path
	#print "A* solution:\n%s" %(path)

	# Draw solution
	#if path and draw_solution:
	#	duckietown_graph.draw(highlight_edges=path.edges())
	print path
	return GraphSearchResponse(path.actions)

def graph_search_server():
    rospy.init_node('graph_search_server')
    s = rospy.Service('graph_search', GraphSearch, handle_graph_search)
    rospy.spin()

if __name__ == "__main__":
	global duckietown_graph
	
	print 'Graph Search Service Started'
	
	# Inputs
	map_path = 'maps/duckietown_map.pkl'
	start_state = 'I15'
	goal_state = 'I26'
	draw_solution = True

	# Loading map
	file2 = open(map_path, 'r')
	map_data = pickle.load(file2)
	file2.close()

	# Create graph
	duckietown_graph = Graph()
	edges = map_data[0]
	node_locations = map_data[1]
	for edge in edges:
		duckietown_graph.add_edge(edge[0], edge[1], edge[2], edge[3])
		
	duckietown_graph.set_node_positions(node_locations)

	graph_search_server()
