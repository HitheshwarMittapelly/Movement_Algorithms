#ifndef UTILITIES_H
#define UTILITIES_H
#include "DirectedWeightedEdge.h"
#include "NodeRecord.h"
#include <vector>
#include <queue>
namespace AI {
	namespace Utilities {
		std::vector<AI::Pathfinding::DirectedWeightedEdge *> ReadFile(std::string i_path);

		template<
			class T,
			class Container = std::vector<T>,
			class Compare = std::less<typename Container::value_type>
		>class PQueue : public std::priority_queue<T, Container, Compare> {
		public:
			
			typedef typename
				std::priority_queue<T, Container, Compare>
				::container_type::const_iterator const_iterator;
			const_iterator begin() {
				return this->c.cbegin();
			}
			const_iterator end() {
				return this->c.cend();
			}
			const_iterator find(const T&i_val) const{
				auto first = this->c.cbegin();
				auto last = this->c.cend();
				while (first != last) {
					if (*first == i_val) {
						return first;
					}
					++first;
				}
				return last;
			}
			bool remove(const T&i_val) {
				auto first = this->c.cbegin();
				auto last = this->c.cend();
				while (first != last) {
					if (*first == i_val) {
						this->c.erase(first);
						return true;
					}
					++first;
				}
				return false;
			}
		};
	}
}

#endif // !UTILITIES_H

