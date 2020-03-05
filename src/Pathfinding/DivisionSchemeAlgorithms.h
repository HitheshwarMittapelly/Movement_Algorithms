#ifndef DIVISIONSCHEMEALGORITHMS_H
#define DIVISIONSCHEMEALGORITHMS_H
#include "DirectedGraph.h"
#include "../Movement/sVector2D.h"
#include <unordered_set>
namespace AI {
	namespace DivisionSchemes {
		AI::Pathfinding::DirectedGraph * GenerateTileBasedGraph();
		std::unordered_set<int> GetCollisionZones();
		int GetQuantizedNode(AI::Math::sVector2D i_position);
		AI::Math::sVector2D GetLocalizedPosition(int i_node);
	
	}
}
#endif // !DIVISIONSCHEMEALGORITHMS_H
