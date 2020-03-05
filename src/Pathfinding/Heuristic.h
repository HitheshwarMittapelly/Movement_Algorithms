#ifndef HEURISTIC_H
#define HEURISTIC_H
#include "DirectedGraph.h"
#include "DivisionSchemeAlgorithms.h"
#include <map>
namespace AI {
	namespace Pathfinding {
		class Heuristic {
		public:
			int GetEstimate(int i_node);
			Heuristic(int i_goalNode, DirectedGraph * i_graph);
			Heuristic(int i_goalNode, int i_totalNodes);
			int GetEstimateFromConstantGuess(int i_node);
			void SetHeuristicType(bool i_val) {
				useTypeOne = i_val;
			}
		private:
			void CalculateAndStoreHeuristics(int i_goalNode, int i_totalNodes);
			void InsertConstantGuessHeuristics();
			Heuristic() = default;
			std::map<int, int> heuristicMap;
			std::map<int, int> constantGuessMap;
			bool useTypeOne = true;
		};
		
		
		
	}
}

#endif // !HEURISTIC_H

