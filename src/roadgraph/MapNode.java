package roadgraph;
import java.util.*;

import geography.GeographicPoint;
public class MapNode extends GeographicPoint {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private ArrayList<MapEdge> edges = new ArrayList<MapEdge>();
	public MapNode(double latitude, double longitude) {
		super(latitude, longitude);
		// TODO Auto-generated constructor stub
	}
    public int getNumberOfEdges()
    {
    	return edges.size();
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
	
}
