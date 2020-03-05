#ifndef UTILITIES_H
#define UTILITIES_H
#include "DirectedWeightedEdge.h"
#include "NodeRecord.h"
#include <vector>
#include <unordered_set>
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
			std::unordered_set<int> elementsSet;
			const_iterator begin() {
				return this->c.cbegin();
			}
			const_iterator end() {
				return this->c.cend();
			}
			bool contains(const int i_node) {
				if (elementsSet.find(i_node) != elementsSet.end()) {
					return true;
				}
				return false;
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
			bool remove(int i_node,const T&i_val) {
				bool contains = this->contains(i_node);
				if (contains) {
					auto first = this->c.cbegin();
					auto last = this->c.cend();
					while (first != last) {
						if (*first == i_val) {
							this->c.erase(first);
							elementsSet.erase(i_node);
							return true;
						}
						++first;
					}
				}
							
				return false;
			}
			void removeNode(int i_node) {
				if (contains(i_node)) {
					elementsSet.erase(i_node);
				}
			}
			void PushNode(int i_node, const value_type& _Val)
			{	
				elementsSet.insert(i_node);
				this->push(_Val);
			}

		};
	}
}

#endif // !UTILITIES_H

