#ifndef PATH_H
#define PATH_H
#include "DirectedWeightedEdge.h"
#include <vector>
namespace AI {
	namespace Pathfinding {
		class Path {
		public:
			std::vector<DirectedWeightedEdge *> path;
			Path(std::vector<DirectedWeightedEdge *> i_path) :path(i_path) {

			}
		private:
			Path() = default;
		};
	}
}

#endif // !PATH_H

