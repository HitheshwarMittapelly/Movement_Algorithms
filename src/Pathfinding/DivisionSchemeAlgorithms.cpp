#include "DivisionSchemeAlgorithms.h"
#include "../ofApp.h"
namespace {
	int numberOfRows;
	int numberOfColumns;
	std::unordered_set<int> collisionZones;
	void SetupCollisionZones(int maxNodes) {
		auto step = maxNodes / 20;
		for (int i = 0; i < maxNodes; i += step) {
			for (int j = 0; j < 5; j++) {
				auto random = rand() % ((i+1) * step);
				if (collisionZones.find(random) == collisionZones.end()) {
					collisionZones.insert(random);
				}
			}
		}
		if (collisionZones.find(40) != collisionZones.end()) {
			collisionZones.erase(40);
		}
	}
}

AI::Pathfinding::DirectedGraph * AI::DivisionSchemes::GenerateTileBasedGraph()
{
	
	numberOfRows = AI::windowHeight / AI::heightOfTile;
	numberOfColumns = AI::windowWidth / AI::widthOfTile;
	SetupCollisionZones(numberOfRows * numberOfColumns);
	std::vector<AI::Pathfinding::DirectedWeightedEdge *> allEdges;
	int source = 0;
	int sink = 0;
	int cost = 0;
	
	for (int i = 0; i < numberOfRows; i++) {
		for (int j = 0; j < numberOfColumns; j++) {
			int source = j + (i* numberOfColumns);
			int sink = 0;
			if (collisionZones.find(source) == collisionZones.end()) {
				//Left
				{
					if (j - 1 >= 0) {
						AI::Pathfinding::DirectedWeightedEdge * newEdge;
						sink = (j - 1) + (i * numberOfColumns);
						if (collisionZones.find(sink) == collisionZones.end()) {
							cost = (GetLocalizedPosition(source) - GetLocalizedPosition(sink)).GetLength();
							newEdge = new AI::Pathfinding::DirectedWeightedEdge(source, sink, cost);
							allEdges.push_back(newEdge);
						}
					
					}
				}
				//Right
				{
					if (j + 1 < numberOfColumns) {
						AI::Pathfinding::DirectedWeightedEdge * newEdge;
						sink = (j + 1) + (i * numberOfColumns);
						if (collisionZones.find(sink) == collisionZones.end()) {
							cost = (GetLocalizedPosition(source) - GetLocalizedPosition(sink)).GetLength();
							newEdge = new AI::Pathfinding::DirectedWeightedEdge(source, sink, cost);
							allEdges.push_back(newEdge);
						}
					}
				}
				//Up
				{
					if (i - 1 >= 0) {
						AI::Pathfinding::DirectedWeightedEdge * newEdge;
						sink = (j)+((i - 1)* numberOfColumns);
						if (collisionZones.find(sink) == collisionZones.end()) {
							cost = (GetLocalizedPosition(source) - GetLocalizedPosition(sink)).GetLength();
							newEdge = new AI::Pathfinding::DirectedWeightedEdge(source, sink, cost);
							allEdges.push_back(newEdge);
						}
					}
				}
				//Down
				{
					if (i + 1 < numberOfRows) {
						AI::Pathfinding::DirectedWeightedEdge * newEdge;
						sink = (j)+((i + 1) * numberOfColumns);
						if (collisionZones.find(sink) == collisionZones.end()) {
							cost = (GetLocalizedPosition(source) - GetLocalizedPosition(sink)).GetLength();
							newEdge = new AI::Pathfinding::DirectedWeightedEdge(source, sink, cost);
							allEdges.push_back(newEdge);
						}
					}
				}
			}
			
		}
	}
	
	return new AI::Pathfinding::DirectedGraph(allEdges);
}

std::unordered_set<int> AI::DivisionSchemes::GetCollisionZones()
{
	return collisionZones;
}

int AI::DivisionSchemes::GetQuantizedNode(AI::Math::sVector2D i_position)
{
	int column = i_position.x / AI::widthOfTile;
	int row = i_position.y / AI::heightOfTile;
	return column + (row * numberOfColumns);
}

AI::Math::sVector2D AI::DivisionSchemes::GetLocalizedPosition(int i_node)
{
	int row = i_node / numberOfColumns;
	int column = i_node % numberOfColumns;
	float x = (column + 0.5) * AI::widthOfTile;
	float y = (row + 0.5) * AI::heightOfTile;
	return AI::Math::sVector2D(x, y);

}
