/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.*;
import java.util.function.Consumer;
import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 *         A class which represents a graph of geographic locations Nodes in the
 *         graph are intersections between
 *
 */
public class MapGraph {
	// TODO: Add your member variables here in WEEK 3
	private HashMap<GeographicPoint, MapNode> nodes;
	private int numOfEdges = 0;
	private GeographicPoint _goal;

	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph() {
		// TODO: Implement in this constructor in WEEK 3
		nodes = new HashMap<GeographicPoint, MapNode>();
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * 
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		// TODO: Implement this method in WEEK 3
		return nodes.size();
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * 
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		// TODO: Implement this method in WEEK 3
		return nodes.keySet();
	}

	/**
	 * Get the number of road segments in the graph
	 * 
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		// TODO: Implement this method in WEEK 3

		// // for each MapNode inside the graph, sum up the number of edges
		// for (MapNode node : nodes.values()) {
		// numOfEdges += node.getNumberOfEdges();
		// }
		return numOfEdges;
	}

	/**
	 * Add a node corresponding to an intersection at a Geographic Point If the
	 * location is already in the graph or null, this method does not change the
	 * graph.
	 * 
	 * @param location
	 *            The location of the intersection
	 * @return true if a node was added, false if it was not (the node was
	 *         already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		// TODO: Implement this method in WEEK 3
		if (location == null || nodes.containsKey(location)) {
			return false;
		} else {
			MapNode node = new MapNode(location.x, location.y);
			nodes.put(location, node);
			return true;
		}
	}

	/**
	 * Adds a directed edge to the graph from pt1 to pt2. Precondition: Both
	 * GeographicPoints have already been added to the graph
	 * 
	 * @param from
	 *            The starting point of the edge
	 * @param to
	 *            The ending point of the edge
	 * @param roadName
	 *            The name of the road
	 * @param roadType
	 *            The type of the road
	 * @param length
	 *            The length of the road, in km
	 * @throws IllegalArgumentException
	 *             If the points have not already been added as nodes to the
	 *             graph, if any of the arguments is null, or if the length is
	 *             less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length)
			throws IllegalArgumentException {

		// TODO: Implement this method in WEEK 3
		if (!(nodes.containsKey(from) && nodes.containsKey(to))) {
			throw new IllegalArgumentException("cannot add edge if its nodes are not present in graph");
		}
		nodes.get(from).addEdge(nodes.get(from), nodes.get(to), roadName, roadType, length);
		numOfEdges++;
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return bfs(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 3
		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		// child maps to parent <child, parent>
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		Queue<GeographicPoint> queue = new LinkedList<GeographicPoint>();
		queue.offer(start);
		while (queue.size() > 0) {
			GeographicPoint currentNode = queue.poll();
			nodeSearched.accept(currentNode);
			if (currentNode.equals(goal)) {
				return reconstructPath(start, goal, parentMap);
			}
			if (!visited.contains(currentNode)) {
				visited.add(currentNode);
				// queue.addAll(getNeighborEdgesByPoint(currentNode));
				for (MapEdge edge : getNeighborEdgesByPoint(currentNode)) {
					MapNode next = edge.getNodeTo();
					if (!visited.contains(next)) {
						queue.offer(next);
						parentMap.put(next, currentNode);
						if (next.equals(goal)) {
							// already find the target, break since no need to
							// look further
							nodeSearched.accept(next);
							return reconstructPath(start, goal, parentMap);
						}
					}
				}
			}
		}
		return null;
	}

	private List<MapEdge> getNeighborEdgesByPoint(GeographicPoint point) {
		return nodes.get(point).getEdges();
	}

	public LinkedList<GeographicPoint> reconstructPath(GeographicPoint start, GeographicPoint goal,
			HashMap<GeographicPoint, GeographicPoint> parentMap) {
		// reconstruct the path
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		GeographicPoint curr = goal;
		while (curr != start) {
			path.addFirst(curr);
			// <child, parent>
			curr = parentMap.get(curr);
		}
		path.addFirst(start);
		return path;
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return dijkstra(start, goal, temp);
	}

	public class distanceComparator implements Comparator<MapNode> {
		@Override
		public int compare(MapNode x, MapNode y) {
			if (x.getDistanceFromStart() < y.getDistanceFromStart()) {
				return -1;
			}
			if (x.getDistanceFromStart() > y.getDistanceFromStart()) {
				return 1;
			}
			return 0;
		}
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 4
		_goal = goal;
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		// initilize starting point 0
		((MapNode) nodes.get(start)).setDistanceFromStart(0);
		// child maps to parent <child, parent>
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		distanceComparator comparator = new distanceComparator();
		PriorityQueue<MapNode> queue = new PriorityQueue<MapNode>(comparator);
		queue.offer(nodes.get(start));
		while (queue.size() > 0) {
			GeographicPoint currentNode = queue.poll();
			nodeSearched.accept(currentNode);
			if (currentNode.equals(goal)) {
				return reconstructPath(start, goal, parentMap);
			}
			if (!visited.contains(currentNode)) {
				visited.add(currentNode);
				// queue.addAll(getNeighborEdgesByPoint(currentNode));
				for (MapEdge edge : getNeighborEdgesByPoint(currentNode)) {
					MapNode next = edge.getNodeTo();
					// update distance of next
					double newDistance = ((MapNode) currentNode).getDistanceFromStart() + edge.getLength();
					if (newDistance < next.getDistanceFromStart()) {
						next.setDistanceFromStart(newDistance);
					}
					if (!visited.contains(next)) {
						queue.add(next);
						parentMap.put(next, currentNode);
						if (next.equals(goal)) {
							// already find the target, break since no need to
							// look further
							nodeSearched.accept(next);
							return reconstructPath(start, goal, parentMap);
						}
					}
				}
			}
		}
		return null;
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return aStarSearch(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 4

		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());

		return null;
	}

	public static void main(String[] args) {
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");

		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);

		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute0 = simpleTestMap.bfs(testStart, testEnd);
		System.out.println(testroute0);
		List<GeographicPoint> testroute1 = simpleTestMap.dijkstra(testStart, testEnd);
		System.out.println(testroute1);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart, testEnd);
		System.out.println(testroute2);

		// MapGraph testMap = new MapGraph();
		// GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		//
		// testStart = new GeographicPoint(32.869423, -117.220917);
		// testEnd = new GeographicPoint(32.869255, -117.216927);
		// System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar
		// should be 5");
		// testroute = testMap.dijkstra(testStart, testEnd);
		// testroute2 = testMap.aStarSearch(testStart, testEnd);
		//
		// testStart = new GeographicPoint(32.8674388, -117.2190213);
		// testEnd = new GeographicPoint(32.8697828, -117.2244506);
		// System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar
		// should be 10");
		// testroute = testMap.dijkstra(testStart, testEnd);
		// testroute2 = testMap.aStarSearch(testStart, testEnd);
		//
		// MapGraph theMap = new MapGraph();
		// System.out.print("DONE. \nLoading the map...");
		// GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		// System.out.println("DONE.");
		//
		// GeographicPoint start = new GeographicPoint(32.8648772,
		// -117.2254046);
		// GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		//
		// List<GeographicPoint> route = theMap.dijkstra(start, end);
		// List<GeographicPoint> route2 = theMap.aStarSearch(start, end);

	}

}
