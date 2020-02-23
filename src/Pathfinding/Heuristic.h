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
			Heuristic(int i_goalNode,int i_totalNodes);
			Heuristic(int i_goalNode, DirectedGraph * i_graph);
		private:
			void CalculateAndStoreHeuristics(int i_goalNode, int i_totalNodes);
			Heuristic() = default;
			std::map<int, int> heuristicMap;
		};
	}
}

#endif // !HEURISTIC_H

