import pickle, csv
import numpy as np
from graph import Graph

class Tile():
	def __init__(self, csv_row):
		self.x = float(csv_row[0])
		self.y = float(csv_row[1])
		self.type = csv_row[2]
		self.rotation = float(csv_row[3])
		self.node1 = 0
		self.node2 = 0
		self.node3 = 0
		self.node4 = 0
		self.node5 = 0
		self.node6 = 0
		self.node7 = 0
		self.node8 = 0
	
	def create_nodes(self):
		if self.type == 'turn':
			return self.create_turn_nodes()
		elif self.type == '3way':
			return self.create_3way_nodes()
		elif self.type == '4way':	
			return self.create_4way_nodes()
		else:
			return {},[]

	def create_turn_nodes(self):
		x = self.x
		y = self.y
		theta = self.rotation
		node1_default_pos = (-0.25, 0.25)
		node1_default_dir = (-1, 0)
		node2_default_pos = (0.25,-0.25)
		node2_default_dir = (0, 1)
		node1_pos = self.rotateAndTranslate(theta,x,y,node1_default_pos)
		node2_pos = self.rotateAndTranslate(theta,x,y,node2_default_pos)
		node1_dir = self.rotate(theta, node1_default_dir)
		node2_dir = self.rotate(theta, node2_default_dir)
		self.node1 = Node(node1_pos, node1_dir)
		self.node2 = Node(node2_pos	, node2_dir)
		node_loc = {self.node1.name:self.node1.pos, self.node2.name:self.node2.pos}
		edges = []
		return node_loc, edges

	def create_3way_nodes(self):
		x = self.x
		y = self.y
		theta = self.rotation
		node1_default_pos = (0.5, 0.25)
		node1_default_dir = (-1, 0)
		node2_default_pos = (0.25, 0.5)
		node2_default_dir = (0, 1)
		node3_default_pos = (-0.25,0.5)
		node3_default_dir = (0, -1)
		node4_default_pos = (-0.5, 0.25)
		node4_default_dir = (-1, 0)
		node5_default_pos = (-0.5, -0.25)
		node5_default_dir = (1, 0)
		node6_default_pos = (0.5, -0.25)
		node6_default_dir = (1, 0)
		node1_pos = self.rotateAndTranslate(theta,x,y,node1_default_pos)
		node2_pos = self.rotateAndTranslate(theta,x,y,node2_default_pos)
		node3_pos = self.rotateAndTranslate(theta,x,y,node3_default_pos)
		node4_pos = self.rotateAndTranslate(theta,x,y,node4_default_pos)
		node5_pos = self.rotateAndTranslate(theta,x,y,node5_default_pos)
		node6_pos = self.rotateAndTranslate(theta,x,y,node6_default_pos)
		node1_dir = self.rotate(theta, node1_default_dir)
		node2_dir = self.rotate(theta, node2_default_dir)
		node3_dir = self.rotate(theta, node3_default_dir)
		node4_dir = self.rotate(theta, node4_default_dir)
		node5_dir = self.rotate(theta, node5_default_dir)
		node6_dir = self.rotate(theta, node6_default_dir)
		self.node1 = Node(node1_pos, node1_dir)
		self.node2 = Node(node2_pos, node2_dir)
		self.node3 = Node(node3_pos, node3_dir)
		self.node4 = Node(node4_pos, node4_dir)
		self.node5 = Node(node5_pos, node5_dir)
		self.node6 = Node(node6_pos, node6_dir)
		node_loc = {self.node1.name:self.node1.pos, self.node2.name:self.node2.pos,
			self.node3.name:self.node3.pos, self.node4.name:self.node4.pos,
			self.node5.name:self.node5.pos, self.node6.name:self.node6.pos}

		edges = [[self.node1.name, self.node2.name, 'r'],
        	[self.node1.name, self.node4.name, 's'],
        	[self.node3.name, self.node4.name, 'r'],
        	[self.node3.name, self.node6.name, 'l'],
        	[self.node5.name, self.node2.name, 'l'],
        	[self.node5.name, self.node6.name, 's']]

		return node_loc,edges
	def create_4way_nodes(self):
		x = self.x
		y = self.y
		theta = self.rotation
		node1_default_pos = (0.5, 0.25)
		node1_default_dir = (-1, 0)
		node2_default_pos = (0.25, 0.5)
		node2_default_dir = (0, 1)
		node3_default_pos = (-0.25,0.5)
		node3_default_dir = (0, -1)
		node4_default_pos = (-0.5, 0.25)
		node4_default_dir = (-1, 0)
		node5_default_pos = (-0.5, -0.25)
		node5_default_dir = (1, 0)
		node6_default_pos = (-0.25, -0.5)
		node6_default_dir = (0, -1)
		node7_default_pos = (0.25, -0.5)
		node7_default_dir = (0, 1)
		node8_default_pos = (0.5, -0.25)
		node8_default_dir = (1, 0)
		node1_pos = self.rotateAndTranslate(theta,x,y,node1_default_pos)
		node2_pos = self.rotateAndTranslate(theta,x,y,node2_default_pos)
		node3_pos = self.rotateAndTranslate(theta,x,y,node3_default_pos)
		node4_pos = self.rotateAndTranslate(theta,x,y,node4_default_pos)
		node5_pos = self.rotateAndTranslate(theta,x,y,node5_default_pos)
		node6_pos = self.rotateAndTranslate(theta,x,y,node6_default_pos)
		node7_pos = self.rotateAndTranslate(theta,x,y,node7_default_pos)
		node8_pos = self.rotateAndTranslate(theta,x,y,node8_default_pos)
		node1_dir = self.rotate(theta, node1_default_dir)
		node2_dir = self.rotate(theta, node2_default_dir)
		node3_dir = self.rotate(theta, node3_default_dir)
		node4_dir = self.rotate(theta, node4_default_dir)
		node5_dir = self.rotate(theta, node5_default_dir)
		node6_dir = self.rotate(theta, node6_default_dir)
		node7_dir = self.rotate(theta, node7_default_dir)
		node8_dir = self.rotate(theta, node8_default_dir)
		self.node1 = Node(node1_pos, node1_dir)
		self.node2 = Node(node2_pos, node2_dir)
		self.node3 = Node(node3_pos, node3_dir)
		self.node4 = Node(node4_pos, node4_dir)
		self.node5 = Node(node5_pos, node5_dir)
		self.node6 = Node(node6_pos, node6_dir)
		self.node7 = Node(node7_pos, node7_dir)
		self.node8 = Node(node8_pos, node8_dir)
		node_loc = {self.node1.name:self.node1.pos, self.node2.name:self.node2.pos,
			self.node3.name:self.node3.pos, self.node4.name:self.node4.pos,
			self.node5.name:self.node5.pos, self.node6.name:self.node6.pos,
			self.node7.name:self.node7.pos, self.node8.name:self.node8.pos}

		edges = [[self.node1.name, self.node2.name, 'r'],
        	[self.node1.name, self.node4.name, 's'],
        	[self.node1.name, self.node6.name, 'l'],
        	[self.node3.name, self.node4.name, 'r'],
        	[self.node3.name, self.node8.name, 'l'],
        	[self.node3.name, self.node6.name, 's'],
        	[self.node5.name, self.node2.name, 'l'],
        	[self.node5.name, self.node8.name, 's'],
        	[self.node5.name, self.node6.name, 'r'],
        	[self.node7.name, self.node8.name, 'r'],
        	[self.node7.name, self.node2.name, 's'],
        	[self.node7.name, self.node4.name, 'l']]

		return node_loc,edges

	def create_edges(self,tile_map):
		if self.type == 'turn':
			return self.create_turn_edges(tile_map)
		elif self.type == '3way':
			return self.create_3way_edges(tile_map)
		elif self.type == '4way':	
			return self.create_4way_edges(tile_map)
		else:
			return []

	def create_turn_edges(self,tile_map):
		edges = []
		edges.append(self.connect_node(self.node1,tile_map))
		edges.append(self.connect_node(self.node2,tile_map))
		return edges

	def create_3way_edges(self,tile_map):
		edges = []
		edges.append(self.connect_node(self.node2,tile_map))
		edges.append(self.connect_node(self.node4,tile_map))
		edges.append(self.connect_node(self.node6,tile_map))
		return edges
	def create_4way_edges(self,tile_map):
		edges = []
		edges.append(self.connect_node(self.node2,tile_map))
		edges.append(self.connect_node(self.node4,tile_map))
		edges.append(self.connect_node(self.node6,tile_map))
		edges.append(self.connect_node(self.node8,tile_map))
		return edges
	def connect_node(self, node,tile_map):
		next_tile_pos_x = self.x + node.direction[0]
		next_tile_pos_y = self.y + node.direction[1]
		t = self.get_tile(next_tile_pos_x,next_tile_pos_y,tile_map)
		while t.type == 'straight':
			next_tile_pos_x = next_tile_pos_x + node.direction[0]
			next_tile_pos_y = next_tile_pos_y + node.direction[1]
			t = self.get_tile(next_tile_pos_x,next_tile_pos_y,tile_map)

		if t.type == 'turn':
			if t.node1.flow(node) == 2:
				return [node.name, t.node1.name, 'f']
			elif t.node2.flow(node) == 2:
				return [node.name, t.node2.name, 'f']
		elif t.type == '3way':
			if t.node1.flow(node) == 4:
				return [node.name, t.node1.name, 'f']
			elif t.node3.flow(node) == 4:
				return [node.name, t.node3.name, 'f']
			elif t.node5.flow(node) == 4:
				return [node.name, t.node5.name, 'f']
		elif t.type == '4way':
			if t.node1.flow(node) == 4:
				return [node.name, t.node1.name, 'f']
			elif t.node3.flow(node) == 4:
				return [node.name, t.node3.name, 'f']
			elif t.node5.flow(node) == 4:
				return [node.name, t.node5.name, 'f']
			elif t.node7.flow(node) == 4:
				return [node.name, t.node7.name, 'f']

	def get_tile(self, x, y,tile_map):
		for tile in tile_map:
			if tile.x == x and tile.y == y:
				return tile


	def rotate(self,theta,vec):
		theta = np.deg2rad(theta)
		vec = np.matrix(vec).transpose()
		rotMatrix = np.matrix([[np.cos(theta), -np.sin(theta)],
			[np.sin(theta), np.cos(theta)]])
		res = np.dot(rotMatrix,vec)
		res = res.tolist()
		return (int(res[0][0]),int(res[1][0]))

	def rotateAndTranslate(self,theta,x,y,vec):
		theta = np.deg2rad(theta)
		vec = np.matrix(vec).transpose()
		vec = np.append(vec,[[1]],axis=0)
		rotMatrix = np.matrix([[np.cos(theta), -np.sin(theta), x+0.5],
			[np.sin(theta), np.cos(theta), y+0.5],
			[0,0,1]])
		
		res = np.dot(rotMatrix,vec)
		res = res[0:2,0]
		res = res.tolist()

		return (res[0][0],res[1][0])


class Node():
	n = 10
	def __init__(self, pos, direction):
		self.name = str(Node.n)
		self.pos = pos
		self.direction = direction
		Node.n = Node.n + 1

	def flow(self, node):
		return (self.direction[0]+node.direction[0])**2 + (self.direction[1]+node.direction[1])**2


class graph_creator():
	def __init__(self):
		self.node_locations = {}
		self.edges = []
		self.tile_map = []
	def add_node_locations(self,node_loc):
		self.node_locations.update(node_loc)
	   
	def add_edges(self, ed):
		for edge in ed:
			source = edge[0]
			target = edge[1]
			action = edge[2]
			manhattan_dist = abs(self.node_locations[source][0] - self.node_locations[target][0]) + abs(self.node_locations[source][1] - self.node_locations[target][1])
			self.edges.append([source, target, manhattan_dist, action])
	def save(self, name='duckietown_map.pkl'):
		afile = open(r'maps/duckietown_map.pkl', 'w+')
		pickle.dump([self.edges, self.node_locations], afile)
		afile.close()
	def draw(self):
	    # Create graph
		duckietown_graph = Graph()
		for edge in self.edges:
			duckietown_graph.add_edge(edge[0], edge[1], edge[2], edge[3])

		duckietown_graph.set_node_positions(self.node_locations)
		duckietown_graph.draw(map_name='duckietown_map')

	def build_tile_map_from_csv(self):
		with open('maps/tiles_226.csv', 'rb') as f:
			spamreader = csv.reader(f,skipinitialspace=True)
			for i,row in enumerate(spamreader):
				if i != 0:
					row_ = [element.strip() for element in row] # remove white spaces
					self.tile_map.append(Tile(row_))
		self.generate_node_locations()
		self.generate_edges()
	def generate_node_locations(self):
		for tile in self.tile_map:
			node_loc,edges = tile.create_nodes()
			self.add_node_locations(node_loc)
			self.add_edges(edges)
	def generate_edges(self):
		for tile in self.tile_map:
			edges = tile.create_edges(self.tile_map)
			self.add_edges(edges)
	def get_map_226(self):
		# Node locations (for visual representation) and heuristics calculation
		node_locations = {"11":(4,0.75),"12":(3.75,1),"13":(3.25,1),"14":(3,0.75),"15":(3,0.25),"18":(4,0.25),
                          "21":(1,2.75),"22":(0.75,3),"23":(0.25,3),"26":(0.25,2),"27":(0.75,2),"28":(1,2.25),
                          "31":(1,5.75),"32":(0.75,6),"33":(0.25,6),"36":(0.25,5),"37":(0.75,5),"38":(1,5.25),
                          "41":(3,5.75),"42":(2.75,6),"43":(2.25,6),"44":(2,5.75),"45":(2,5.25),"46":(2.25,5),
                          "47":(2.75,5),"48":(3,5.25),
                          "51":(4,9.75),"54":(3,9.75),"55":(3,9.25),"56":(3.25,9),"57":(3.75,9),"58":(4,9.25),
                          "121":(3.75,3.25),"122":(5.25,3.25),"123":(5.25,0.75),
                          "181":(5.75,0.25),"182":(5.75,3.75),"183":(3.25,3.75),
                          "141":(0.75,0.75),
                          "261":(0.25,0.25),
                          "281":(2.75,2.25),
                          "481":(4.75,5.25),"482":(4.75,8.25),"483":(5.75,8.25),"484":(5.75,9.75),
                          "581":(5.25,9.25),"582":(5.25,8.75),"583":(4.25,8.75),"584":(4.25,5.75),
                          "561":(3.25,7.75),"562":(2.25,7.75),
                          "421":(2.75,7.25),"422":(3.75,7.25),
                          "541":(0.25,9.75),
                          "321":(0.75,9.25),
                          "461":(2.25,2.75)}

        # Create edges for the map.
        # Edges is a list of lists, in which each edge is in the format: [source, target, action]
		edges = [["11", "12", 'r'],
        ["11", "14",  's'],
        ["12", "121", 'f'], ["121", "122", 'f'], ["122", "123", 'f'], ["123", "11", 'f'] , 
        ["13", "14",  'r'],
        ["13", "18",  'l'],
        ["14", "141", 'f'], ["141", "27",  'f'],
        ["15", "12",  'l'],
        ["15", "18",  's'],
        ["18", "181", 'f'], ["181", "182", 'f'],  ["182", "183", 'f'],  ["183", "13", 'f'],

        ["21", "22",  'r'],
        ["21", "26",  'l'],
        ["22", "37",  'f'],
        ["23", "26",  's'],
        ["23", "28",  'r'],
        ["26", "261", 'f'], ["261", "15", 'f'],
        ["27", "22",  's'],
        ["27", "28",  'r'],
        ["28", "281",  'f'], ["281", "47",  'f'],

        ["31", "32",  'r'],
        ["31", "36",  'l'],
        ["32", "321",  'f'], ["321", "55",  'f'],
        ["33", "36",  's'],
        ["33", "38",  'r'],
        ["36", "23",  'f'],
        ["37", "32",  's'],
        ["37", "38",  'r'],
        ["38", "45",  'f'],
     
        ["41", "42",  'r'],
        ["41", "44",  's'],
        ["41", "46",  'l'],
        ["42", "421",  'f'], ["421", "422",  'f'], ["422", "57",  'f'],
        ["43", "44",  'r'],
        ["43", "46",  's'],
        ["43", "48",  'l'],
        ["44", "31",  'f'],
        ["45", "46",  'r'],
        ["45", "48",  's'],
        ["45", "42",  'l'],
        ["46", "461",  'f'], ["461", "21",  'f'],
        ["47", "48",  'r'],
        ["47", "42",  's'],
        ["47", "44",  'l'],
        ["48", "481",  'f'], ["481", "482",  'f'],["482", "483",  'f'],["483", "484",  'f'], ["484", "51",  'f'],
     
        ["51", "54",  's'],
        ["51", "56",  'l'],
        ["54", "541",  'f'], ["541", "33",  'f'],
        ["55", "56",  'r'],
        ["55", "58",  's'],
        ["56", "561",  'f'], ["561", "562",  'f'], ["562", "43",  'f'],
        ["57", "58",  'r'],
        ["57", "54",  'l'],
        ["58", "581",  'f'], ["581", "582",  'f'], ["582", "583",  'f'], ["583", "584",  'f'], ["584", "41",  'f']]

		return node_locations, edges
 
if __name__ == "__main__":
    gc = graph_creator()
    gc.build_tile_map_from_csv()
    # Node locations (for visual representation) and heuristics calculation
    #node_locations, edges = gc.get_map_226()
    #gc.add_node_locations(node_locations)
    #gc.add_edges(edges)
    gc.save()
    gc.draw()






