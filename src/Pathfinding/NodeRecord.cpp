#include "NodeRecord.h"

bool AI::Pathfinding::operator<(const NodeRecord & i_lhs, const NodeRecord & i_rhs)
{
	return i_lhs.costSoFar < i_rhs.costSoFar;
}

bool AI::Pathfinding::operator>(const NodeRecord & i_lhs, const NodeRecord & i_rhs)
{
	return i_lhs.costSoFar > i_rhs.costSoFar;
}

bool AI::Pathfinding::operator==(const NodeRecord & i_lhs, const NodeRecord & i_rhs)
{
	return i_lhs.node == i_rhs.node /*&& i_lhs.costSoFar == i_rhs.costSoFar
		&& i_lhs.estimateToGoal == i_rhs.estimateToGoal && i_lhs.incomingEdge == i_rhs.incomingEdge*/;
}

bool AI::Pathfinding::operator!=(const NodeRecord & i_lhs, const NodeRecord & i_rhs)
{
	return i_lhs.node != i_rhs.node;
}
