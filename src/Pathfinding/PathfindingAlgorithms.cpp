#include "PathfindingAlgorithms.h"
#include "Utilities.h"
#include <iostream>
#include <queue>

AI::Pathfinding::Path * AI::Pathfinding::FindPathUsingDijkstra(DirectedGraph * i_graph, int i_startNode, int i_endNode)
{
	NodeRecord startRecord;
	startRecord.node = i_startNode;
	startRecord.incomingEdge = NULL;
	startRecord.costSoFar = 0;

	AI::Utilities::PQueue<NodeRecord, std::vector<NodeRecord>, 
		std::greater<std::vector<NodeRecord>::value_type>> openList;
	openList.push(startRecord);
	AI::Utilities::PQueue<NodeRecord, std::vector<NodeRecord>,
		std::greater<std::vector<NodeRecord>::value_type>> closedList;
	NodeRecord current;
	while (openList.size() > 0) {
		current = openList.top();
		
		if (current.node == i_endNode) {
			break;
		}
		//std::cout << current.node << std::endl;
		auto outgoingEdges = i_graph->GetOutgoingEdges(current.node);
		for (auto edge : outgoingEdges) {
			auto endNode = edge->GetSink();
			float endNodeCost = current.costSoFar + edge->GetCost();
			NodeRecord temp;
			NodeRecord endNodeRecord;
			temp.node = endNode;
			if (closedList.find(temp) != closedList.end()) {
				continue;
			}
			else if (openList.find(temp) != openList.end()) {
				endNodeRecord = *openList.find(temp);
				if (endNodeRecord.costSoFar <= endNodeCost) {
					continue;
				}
				endNodeRecord.costSoFar = endNodeCost;
				endNodeRecord.incomingEdge = edge;
			}
			else {
				endNodeRecord.node = endNode;
				endNodeRecord.incomingEdge = NULL;
				endNodeRecord.costSoFar = endNodeCost;
				endNodeRecord.incomingEdge = edge;
			}

			if (openList.find(endNodeRecord) == openList.end()) {
				openList.push(endNodeRecord);
			}
			
		}
		openList.pop();
		closedList.push(current);
		
	}
	if (current.node != i_endNode) {
		return NULL;
	}
	else {
		std::vector<AI::Pathfinding::DirectedWeightedEdge *> path;
		while (current.node != i_startNode) {
			path.insert(path.begin(), current.incomingEdge);
			auto next = current.incomingEdge->GetSource();
			
			NodeRecord temp;
			temp.node = next;
			current = *closedList.find(temp);
		}
		std::cout << " Dijkstra Algorithm "<< std::endl;
		for (auto edge : path) {
			std::cout << edge->GetSource() << " - " <<edge->GetSink()<< std::endl;
		}
		return new Path(path);
	}


	
}

AI::Pathfinding::Path * AI::Pathfinding::FindPathUsingAStar(DirectedGraph * i_graph, int i_startNode, int i_endNode, Heuristic * heuristics)
{
	NodeRecord startRecord;
	startRecord.node = i_startNode;
	startRecord.incomingEdge = NULL;
	startRecord.costSoFar = 0;
	startRecord.estimateToGoal = heuristics->GetEstimate(i_startNode);

	AI::Utilities::PQueue<NodeRecord, std::vector<NodeRecord>,
		std::greater<std::vector<NodeRecord>::value_type>> openList;
	openList.push(startRecord);
	AI::Utilities::PQueue<NodeRecord, std::vector<NodeRecord>,
		std::greater<std::vector<NodeRecord>::value_type>> closedList;
	NodeRecord current;
	while (openList.size() > 0) {
		current = openList.top();

		if (current.node == i_endNode) {
			break;
		}
		//std::cout << current.node << std::endl;
		auto outgoingEdges = i_graph->GetOutgoingEdges(current.node);
		for (auto edge : outgoingEdges) {
			auto endNode = edge->GetSink();
			float endNodeCost = current.costSoFar + edge->GetCost();
			int endNodeHeuristic = 0;
			NodeRecord temp;
			NodeRecord endNodeRecord;
			temp.node = endNode;
			if (closedList.find(temp) != closedList.end()) {
				endNodeRecord = *closedList.find(temp);
				if (endNodeRecord.costSoFar <= endNodeCost) {
					continue;
				}
				closedList.remove(endNodeRecord);
				 endNodeHeuristic= endNodeRecord.costSoFar - endNodeCost;
			}
			else if (openList.find(temp) != openList.end()) {
				endNodeRecord = *openList.find(temp);
				if (endNodeRecord.costSoFar <= endNodeCost) {
					continue;
				}
				endNodeRecord.costSoFar = endNodeCost;
				endNodeRecord.incomingEdge = edge;
				endNodeHeuristic = endNodeRecord.costSoFar - endNodeCost;
			}
			else {
				endNodeRecord.node = endNode;
				endNodeRecord.incomingEdge = NULL;
				endNodeRecord.costSoFar = endNodeCost;
				endNodeRecord.incomingEdge = edge;
				endNodeHeuristic = heuristics->GetEstimate(endNode);
				endNodeRecord.estimateToGoal = endNodeCost + endNodeHeuristic;
			}

			if (openList.find(endNodeRecord) == openList.end()) {
				
				openList.push(endNodeRecord);
			}

		}
		openList.pop();
		closedList.push(current);

	}
	if (current.node != i_endNode) {
		return NULL;
	}
	else {
		std::vector<AI::Pathfinding::DirectedWeightedEdge *> path;
		while (current.node != i_startNode) {
			path.insert(path.begin(), current.incomingEdge);
			auto next = current.incomingEdge->GetSource();

			NodeRecord temp;
			temp.node = next;
			current = *closedList.find(temp);
		}
		std::cout << " A* Algorithm " << std::endl;
		for (auto edge : path) {
			std::cout << edge->GetSource() << " - " << edge->GetSink() << std::endl;
		}
		return new Path(path);
	}
}


