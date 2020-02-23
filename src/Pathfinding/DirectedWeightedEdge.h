#ifndef DIRECTEDWEIGHTED_H
#define DIRECTEDWEIGHTED_H
namespace AI {
	namespace Pathfinding {
		class DirectedWeightedEdge {
		public:
			float GetCost();
			int GetSource();
			int GetSink();
			DirectedWeightedEdge(int i_source, int i_sink, float i_cost) : 
				source(i_source),sink(i_sink), cost(i_cost){

			}
		private:
			float cost;
			int source;
			int sink;
			DirectedWeightedEdge() = default;
		};
	}
}

#endif
