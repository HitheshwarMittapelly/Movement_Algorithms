#ifndef NODERECORD_H
#define NODERECORD_H
#include "DirectedWeightedEdge.h"

namespace AI {
	namespace Pathfinding {
		struct NodeRecord {
			int node;
			DirectedWeightedEdge * incomingEdge;
			float costSoFar;
			float estimateToGoal;

			friend bool operator < (const NodeRecord& i_lhs, const NodeRecord& i_rhs);
			friend bool operator > (const NodeRecord& i_lhs, const NodeRecord& i_rhs);
			friend bool operator == (const NodeRecord& i_lhs, const NodeRecord& i_rhs);
			friend bool operator != (const NodeRecord& i_lhs, const NodeRecord& i_rhs);
		};
	}
}


#endif // !NODERECORD_H

