import pickle
from graph import Graph
import matplotlib.pyplot as plt

# Create edges for the map.
# Edges is a list of lists, in which each edge is in the format: [source, target, weight, action]
edges = [["I11", "I12", 1.0, 'r'],
["I11", "I14", 1.0, 's'],
["I12", "I11", 9.0, 'f'],
["I13", "I14", 1.0, 'r'],
["I13", "I18", 1.0, 'l'],
["I14", "I27", 4.0, 'f'],
["I15", "I12", 1.0, 'l'],
["I15", "I18", 1.0, 's'],
["I18", "I13", 9.0, 'f'],

["I21", "I22", 1.0, 'r'],
["I21", "I26", 1.0, 'l'],
["I22", "I37", 2.0, 'f'],
["I23", "I26", 1.0, 's'],
["I23", "I28", 1.0, 'r'],
["I26", "I15", 4.0, 'f'],
["I27", "I22", 1.0, 's'],
["I27", "I28", 1.0, 'r'],
["I28", "I47", 4.0, 'f'],

["I31", "I32", 1.0, 'r'],
["I31", "I36", 1.0, 'l'],
["I32", "I55", 6.0, 'f'],
["I33", "I36", 1.0, 's'],
["I33", "I38", 1.0, 'r'],
["I36", "I23", 2.0, 'f'],
["I37", "I32", 1.0, 's'],
["I37", "I38", 1.0, 'r'],
["I38", "I45", 1.0, 'f'],

["I41", "I42", 1.0, 'r'],
["I41", "I44", 1.0, 's'],
["I41", "I46", 1.0, 'l'],
["I42", "I57", 4.0, 'f'],
["I43", "I44", 1.0, 'r'],
["I43", "I46", 1.0, 's'],
["I43", "I48", 1.0, 'l'],
["I44", "I31", 1.0, 'f'],
["I45", "I46", 1.0, 'r'],
["I45", "I48", 1.0, 's'],
["I45", "I42", 1.0, 'l'],
["I46", "I21", 4.0, 'f'],
["I47", "I48", 1.0, 'r'],
["I47", "I42", 1.0, 's'],
["I47", "I44", 1.0, 'l'],
["I48", "I51", 8.0, 'f'],

["I51", "I54", 1.0, 's'],
["I51", "I56", 1.0, 'l'],
["I54", "I33", 6.0, 'f'],
["I55", "I56", 1.0, 'r'],
["I55", "I58", 1.0, 's'],
["I56", "I43", 4.0, 'f'],
["I57", "I58", 1.0, 'r'],
["I57", "I54", 1.0, 'l'],
["I58", "I41", 8.0, 'f']]

# Node locations (for visual representation) and heuristics calculation
node_locations = dict(I11=(4,0.75),I12=(3.75,1),I13=(3.25,1),I14=(3,0.75),I15=(3,0.25),I18=(4,0.25),
                              I21=(1,2.75),I22=(0.75,3),I23=(0.25,3),I26=(0.25,2),I27=(0.75,2),I28=(1,2.25),
                              I31=(1,5.75),I32=(0.75,6),I33=(0.25,6),I36=(0.25,5),I37=(0.75,5),I38=(1,5.25),
                              I41=(3,5.75),I42=(2.75,6),I43=(2.25,6),I44=(2,5.75),I45=(2,5.25),I46=(2.25,5),
                              I47=(2.75,5),I48=(3,5.25),
                              I51=(4,9.75),I54=(3,9.75),I55=(3,9.25),I56=(3.25,9),I57=(3.75,9),I58=(4,9.25))

# Save this into a file
afile = open(r'maps/duckietown_map.pkl', 'w+')
pickle.dump([edges, node_locations], afile)
afile.close()

# Create graph
duckietown_graph = Graph()
for edge in edges:
	duckietown_graph.add_edge(edge[0], edge[1], edge[2], edge[3])
	
duckietown_graph.set_node_positions(node_locations)

duckietown_graph.draw(save_draw=True)

