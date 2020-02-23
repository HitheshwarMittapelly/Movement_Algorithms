#include "DirectedGraph.h"

std::vector<AI::Pathfinding::DirectedWeightedEdge *> AI::Pathfinding::DirectedGraph::GetOutgoingEdges(int i_node)
{
	return graph[i_node];
}

int AI::Pathfinding::DirectedGraph::GetTotalNodes()
{
	return graph.size();
}

AI::Pathfinding::DirectedGraph::DirectedGraph(std::vector<DirectedWeightedEdge *> i_allEdges)
{
	for (auto edge : i_allEdges) {
		auto node = edge->GetSource();
		if (graph.find(node) != graph.end()) {
			auto outgoingEdges = graph[node];
			outgoingEdges.push_back(edge);
			graph[node] = outgoingEdges;
		}else {
			std::vector<DirectedWeightedEdge *> outgoingEdges;
			outgoingEdges.push_back(edge);
			graph.insert(std::make_pair(node, outgoingEdges));
		}
	}


}
