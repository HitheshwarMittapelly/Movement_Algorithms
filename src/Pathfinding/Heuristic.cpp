#include "Heuristic.h"
#include <ctime>
#include <random>
int AI::Pathfinding::Heuristic::GetEstimate(int i_node)
{
	return heuristicMap[i_node];
}

AI::Pathfinding::Heuristic::Heuristic(int i_goalNode, int i_totalNodes)
{
	CalculateAndStoreHeuristics(i_goalNode,i_totalNodes);
}

AI::Pathfinding::Heuristic::Heuristic(int i_goalNode, DirectedGraph * i_graph)
{
	int totalNodes = i_graph->GetTotalNodes();
	int euclideanDistance = 0;
	for (int i = 0; i < totalNodes; i++) {
		euclideanDistance = (AI::DivisionSchemes::GetLocalizedPosition(i) - AI::DivisionSchemes::GetLocalizedPosition(i_goalNode)).GetLength();
		heuristicMap.insert(std::make_pair(i, euclideanDistance));
	}
}

void AI::Pathfinding::Heuristic::CalculateAndStoreHeuristics(int i_goalNode, int i_totalNodes)
{
	heuristicMap.insert(std::make_pair(i_goalNode, 0));
	std::srand(std::time(nullptr));
	int random;
	for (int i = 1; i <= i_totalNodes; i++) {
		if (i != i_goalNode) {
			random = std::rand() % 100;
			heuristicMap.insert(std::make_pair(i, random));
		}
		
	}
}
