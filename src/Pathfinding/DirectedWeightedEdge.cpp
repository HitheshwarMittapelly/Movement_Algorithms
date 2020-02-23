#include "DirectedWeightedEdge.h"

float AI::Pathfinding::DirectedWeightedEdge::GetCost()
{
	return cost;
}

int AI::Pathfinding::DirectedWeightedEdge::GetSource()
{
	return source;
}

int AI::Pathfinding::DirectedWeightedEdge::GetSink()
{
	return sink;
}
