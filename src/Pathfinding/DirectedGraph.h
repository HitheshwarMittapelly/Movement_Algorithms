#ifndef DIRECTEDGRAPH_H
#define DIRECTEDGRAPH_H
#include "DirectedWeightedEdge.h"
#include <vector>
#include <map>
namespace AI {
	namespace Pathfinding {
		class DirectedGraph {
		public:
			std::vector<DirectedWeightedEdge *> GetOutgoingEdges(int i_node);
			int GetTotalNodes();
			DirectedGraph(std::vector<DirectedWeightedEdge *> i_allEdges);
		private:
			std::map<int, std::vector<DirectedWeightedEdge *>> graph;
			DirectedGraph() = default;

		};
	}
}

#endif // !DIRECTEDGRAPH_H
