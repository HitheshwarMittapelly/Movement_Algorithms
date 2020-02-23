#ifndef DIVISIONSCHEMEALGORITHMS_H
#define DIVISIONSCHEMEALGORITHMS_H
#include "DirectedGraph.h"
#include "../Movement/sVector2D.h"
namespace AI {
	namespace DivisionSchemes {
		AI::Pathfinding::DirectedGraph * GenerateTileBasedGraph();
		int GetQuantizedNode(AI::Math::sVector2D i_position);
		AI::Math::sVector2D GetLocalizedPosition(int i_node);

		
	}
}
#endif // !DIVISIONSCHEMEALGORITHMS_H
