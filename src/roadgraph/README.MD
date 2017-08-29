Class: MapGraph

Modifications made to MapGraph (what and why):

1, Added a member variable -HashMap to map from GeographicPoint to MapNode, since class GeographicPoint fits the signature of class method of MapGraph, but MapNode is the actual class to get children of current node by edges. 
As a result, a storage of all GeographicPoint->MapNode is needed for MapGraph class.

2, Implemented class methods addVertex, addEdge, getNumVertices, getVertices, getNumEdges and bfs based on the HashMap of GeographicPoint->MapNode 
3, Added class methods getNeighborEdgesByPoint and reconstructPath based on OOP design, which are used by bfs

Class name: MapNode

Purpose and description of class: It extends GeographicPoint to have 1 more member variable - list of edges to keep track of neibour nodes and the other properties of roads, based on OOP design
1, Added constructor 
2, Added class methods getNumberOfEdges, getEdges and addEdge


Class name: MapEdge

Purpose and description of class: It is used to keep track properties of roads, based on OOP design

1. Added constructor
2. Added getter of various member variable


Overall Design Justification (4-6 sentences):

Two classes (MapNode and MapEdge) are added in order to keep track of the children nodes of any node, as well as the property of the link between nodes, without modification of GeographicPoint class