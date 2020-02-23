#include "Utilities.h"
#include "DirectedWeightedEdge.h"
#include <iostream>
#include <fstream>
#include <boost/filesystem/path.hpp>
#include "utils/ofUtils.h"
std::vector<AI::Pathfinding::DirectedWeightedEdge *> AI::Utilities::ReadFile(std::string i_path)
{
	std::ifstream fin (ofToDataPath(i_path).c_str());
	std::string str;
	std::vector<AI::Pathfinding::DirectedWeightedEdge *> allEdges;
	std::string copyString;
	float cost;
	int source, sink;
	int delimiterIndex = 0;
	while (std::getline(fin, str)) {
		
		if (str[0] != '%') {
			
			AI::Pathfinding::DirectedWeightedEdge * directedEdge;

			copyString = str;
			delimiterIndex = copyString.find(" ");
			source = atoi(copyString.substr(0, delimiterIndex).c_str());
			copyString = copyString.erase(0, delimiterIndex + 1);

			delimiterIndex = copyString.find(" ");
			sink = atoi(copyString.substr(0, delimiterIndex).c_str());
			copyString = copyString.erase(0, delimiterIndex + 1);

			delimiterIndex = copyString.find(" ");
			cost = atof(copyString.substr(0, delimiterIndex).c_str());
			copyString = copyString.erase(0, delimiterIndex + 1);

			directedEdge = new AI::Pathfinding::DirectedWeightedEdge(source, sink, cost);
			allEdges.push_back(directedEdge);
		}
		
	}
	std::cout <<"Total edges read " << allEdges.size() << std::endl;
	return allEdges;
	
}


