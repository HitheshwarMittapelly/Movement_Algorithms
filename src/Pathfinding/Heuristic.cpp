#include "Heuristic.h"
#include <ctime>
#include <random>
int AI::Pathfinding::Heuristic::GetEstimate(int i_node)
{
	if (!useTypeOne) {
		return GetEstimateFromConstantGuess(i_node);
	}
	return heuristicMap[i_node];
}

int AI::Pathfinding::Heuristic::GetEstimateFromConstantGuess(int i_node)
{
	return constantGuessMap[i_node];
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

	InsertConstantGuessHeuristics();

}

void AI::Pathfinding::Heuristic::InsertConstantGuessHeuristics()
{
	constantGuessMap.insert(std::make_pair(1, 2000));
	constantGuessMap.insert(std::make_pair(2, 1900));
	constantGuessMap.insert(std::make_pair(3, 2500));
	constantGuessMap.insert(std::make_pair(4, 2600));
	constantGuessMap.insert(std::make_pair(5, 2100));

	constantGuessMap.insert(std::make_pair(6, 2000));
	constantGuessMap.insert(std::make_pair(7, 1700));
	constantGuessMap.insert(std::make_pair(8, 1900));
	constantGuessMap.insert(std::make_pair(9, 1200));
	constantGuessMap.insert(std::make_pair(10, 1500));

	constantGuessMap.insert(std::make_pair(11, 1100));
	constantGuessMap.insert(std::make_pair(12, 1800));
	constantGuessMap.insert(std::make_pair(13, 1500));
	constantGuessMap.insert(std::make_pair(14, 1000));
	constantGuessMap.insert(std::make_pair(15, 500));

	constantGuessMap.insert(std::make_pair(16, 800));
	constantGuessMap.insert(std::make_pair(17, 200));
	constantGuessMap.insert(std::make_pair(18, 2000));
	constantGuessMap.insert(std::make_pair(19, 1500));
	constantGuessMap.insert(std::make_pair(20, 0));
}


