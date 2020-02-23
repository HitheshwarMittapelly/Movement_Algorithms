#ifndef PATHFINDINGALGORITHMS_H
#define PATHFINDINGALGORITHMS_H
#include "DirectedGraph.h"
#include "NodeRecord.h"
#include "Path.h"
#include "Heuristic.h"

namespace AI{
	namespace Pathfinding {
		AI::Pathfinding::Path * FindPathUsingDijkstra(DirectedGraph * i_graph, int i_startNode, int i_endNode);
		AI::Pathfinding::Path * FindPathUsingAStar(DirectedGraph * i_graph, int i_startNode, int i_endNode,Heuristic * heuristics);
		
	
	}
}


#endif // !PATHFINDINGALGORITHMS_H
