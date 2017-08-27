package roadgraph;

public class MapEdge {
	private final MapNode _from;
	private final MapNode _to;
	private final String _roadName;
	private final String _roadType;
	//private final double _maxSpeed;
	private final double _length;
	public MapEdge(MapNode from, MapNode to, String roadName, String roadType, double length) {
		_from = from;
		_to = to;
		_roadName = roadName;
		_roadType = roadType;
		//_maxSpeed = maxSpeed;
		_length = length;
	}
	public MapNode getNodeTo()
	{
		return _to;
	}
	public String getRoadName()
	{
		return _roadName;
	}
	public String getRoadType()
	{
		return _roadType;
	}
//	public double getMaxSpeed()
//	{
//		return _maxSpeed;
//	}
	public double getLength()
	{
		return _length;
	}
}
