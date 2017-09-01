package roadgraph;
import java.util.*;

import geography.GeographicPoint;
public class MapNode extends GeographicPoint implements Comparable<MapNode> {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private ArrayList<MapEdge> edges = new ArrayList<MapEdge>();
	public double distanceFromGoalEstimated;
	//default value for dijkstra algorithm
	public double distanceFromStart=Integer.MAX_VALUE;
	public MapNode(double latitude, double longitude) {
		super(latitude, longitude);
		// TODO Auto-generated constructor stub
	}
    public int getNumberOfEdges()
    {
    	return edges.size();
    }
    public double getDistanceFromGoal()
    {
    	return distanceFromGoalEstimated;
    	//return Math.sqrt((this.getX()-latGoal)*(this.getX()-latGoal)+(this.getY()-longGoal)*(this.getY()-longGoal));
    }
    public void setDistanceFromGoal(double dist)
    {
        distanceFromGoalEstimated=dist;
    }
    public double getDistanceFromStart()
    {
    	return distanceFromStart;
    	//return Math.sqrt((this.getX()-latGoal)*(this.getX()-latGoal)+(this.getY()-longGoal)*(this.getY()-longGoal));
    }
    public void setDistanceFromStart(double dist)
    {
    	distanceFromStart=dist;
    }
    public ArrayList<MapEdge> getEdges()
    {
    	return edges;
    }
	public void addEdge(MapNode mapNodeFrom, MapNode mapNodeTo, String roadName, String roadType, double length) {
		// TODO Auto-generated method stub
		MapEdge edge=new MapEdge(mapNodeFrom,mapNodeTo,roadName,roadType,length);
		edges.add(edge);
	}
	@Override
	public int compareTo(MapNode o) {
		// TODO Auto-generated method stub
		return 0;
	}
	
}
