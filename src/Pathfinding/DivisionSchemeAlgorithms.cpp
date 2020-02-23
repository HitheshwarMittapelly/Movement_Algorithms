#include "DivisionSchemeAlgorithms.h"
#include "../ofApp.h"
namespace {
	int heightOfTile = 50;
	int widthOfTile = 50;
	int numberOfRows;
	int numberOfColumns;
}

AI::Pathfinding::DirectedGraph * AI::DivisionSchemes::GenerateTileBasedGraph()
{
	numberOfRows = AI::windowHeight / heightOfTile;
	numberOfColumns = AI::windowWidth / widthOfTile;
	std::vector<AI::Pathfinding::DirectedWeightedEdge *> allEdges;
	int source = 0;
	int sink = 0;
	int cost = 0;
	
	for (int i = 0; i < numberOfRows; i++) {
		for (int j = 0; j < numberOfColumns; j++) {
			int source = j + (i* numberOfColumns);
			int sink = 0;
			//Left
			{
				if (j - 1 >= 0) {
					AI::Pathfinding::DirectedWeightedEdge * newEdge;
					sink = (j - 1) + (i * numberOfColumns);
					cost = (GetLocalizedPosition(source) - GetLocalizedPosition(sink)).GetLength();
					newEdge = new AI::Pathfinding::DirectedWeightedEdge(source, sink, cost);
					allEdges.push_back(newEdge);
				}
			}
			//Right
			{
				if (j + 1 < numberOfColumns) {
					AI::Pathfinding::DirectedWeightedEdge * newEdge;
					sink = (j + 1) + (i * numberOfColumns);
					cost = (GetLocalizedPosition(source) - GetLocalizedPosition(sink)).GetLength();
					newEdge = new AI::Pathfinding::DirectedWeightedEdge(source, sink, cost);
					allEdges.push_back(newEdge);
				}
			}
			//Up
			{
				if (i - 1 >= 0) {
					AI::Pathfinding::DirectedWeightedEdge * newEdge;
					sink = (j) + ((i -1)* numberOfColumns);
					cost = (GetLocalizedPosition(source) - GetLocalizedPosition(sink)).GetLength();
					newEdge = new AI::Pathfinding::DirectedWeightedEdge(source, sink, cost);
					allEdges.push_back(newEdge);
				}
			}
			//Down
			{
				if (i + 1 < numberOfRows) {
					AI::Pathfinding::DirectedWeightedEdge * newEdge;
					sink = (j) + ((i + 1) * numberOfColumns);
					cost = (GetLocalizedPosition(source) - GetLocalizedPosition(sink)).GetLength();
					newEdge = new AI::Pathfinding::DirectedWeightedEdge(source, sink, cost);
					allEdges.push_back(newEdge);
				}
			}
		}
	}
	
	return new AI::Pathfinding::DirectedGraph(allEdges);
}

int AI::DivisionSchemes::GetQuantizedNode(AI::Math::sVector2D i_position)
{
	int column = i_position.x / widthOfTile;
	int row = i_position.y / heightOfTile;
	return column + (row * numberOfColumns);
}

AI::Math::sVector2D AI::DivisionSchemes::GetLocalizedPosition(int i_node)
{
	int row = i_node / numberOfColumns;
	int column = i_node % numberOfColumns;
	float x = (column + 0.5) * widthOfTile;
	float y = (row + 0.5) * heightOfTile;
	return AI::Math::sVector2D(x, y);

}
