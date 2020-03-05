#include "ofApp.h"
#include <iostream>
#include <cmath>
#include "Pathfinding/Utilities.h"
#include "Pathfinding/PathfindingAlgorithms.h"
#include "Pathfinding/Heuristic.h"
#include "Pathfinding/DivisionSchemeAlgorithms.h"
#include <math.h>

//--------------------------------------------------------------
void ofApp::setup() {
	counter = 0;
	ofSetCircleResolution(50);
	ofBackground(255, 255, 255);
	bSmooth = false;
	ofSetWindowTitle("Pathfinding Algorithms");

	ofSetFrameRate(60);

	runDijkstraButtononGraphOne.addListener(this, &ofApp::StartDijkstraOnSmallGraph);
	runAStarOnGraphOneButton.addListener(this, &ofApp::StartAStarOnSmallGraph);
	runDijkstraOnLargeGraphButton.addListener(this, &ofApp::StartDijkstraOnLargeGraph);
	runAStarOnLargeGraphButton.addListener(this, &ofApp::StartAStarOnLargeGraph);
	runPathFollowButton.addListener(this, &ofApp::SetupForPathFollow);
	useHeuristicTypeOne.addListener(this, &ofApp::SetHeuristicType);

	panel.setup();
	panel.setDefaultHeight(25);
	panel.setDefaultWidth(500);

	panel.add(runDijkstraButtononGraphOne.setup("Run Dijkstra on Handwritten Graph"));
	panel.add(runAStarOnGraphOneButton.setup("Run A Star on Handwritten Graph"));
	panel.add(runDijkstraOnLargeGraphButton.setup("Run Dijkstra on Large Graph"));
	panel.add(runAStarOnLargeGraphButton.setup("Run A Star on Large Graph"));
	panel.add(runPathFollowButton.setup("Run Path Follow"));
	panel.add(useHeuristicTypeOne.setup("Use Hueristic Type 1",true,500,25));
	panel.add(startNodeIF.setup("StartNode:", "type here"));
	panel.add(endNodeIF.setup("EndNode:", "type here"));
}


void ofApp::StartDijkstraOnSmallGraph()
{
	ReadGraphFiles(false);
	int startNode = 1;
	int endNode = 20;
	std::cout << "Start Node : " << startNode << "	end node : " << endNode << std::endl;
	std::cout << "Path is as follows" << std::endl;
	AI::Pathfinding::FindPathUsingDijkstra(handwrittenGraph, startNode, endNode);
	currentAlgorithm = AI::GraphAlgorithms::GraphAlgos::DIJKSTRA_ONE;
}

void ofApp::StartAStarOnSmallGraph()
{
	ReadGraphFiles(false);
	int startNode = 1;
	int endNode = 20;
	auto heuristic = new AI::Pathfinding::Heuristic(endNode, 20);
	heuristic->SetHeuristicType(b_useHeuristicTypeOne);
	std::cout << "Start Node : " << startNode << "   end node : " << endNode << std::endl;
	std::cout << "Path is as follows" << std::endl;
	AI::Pathfinding::FindPathUsingAStar(handwrittenGraph,
		startNode, endNode,heuristic );
	currentAlgorithm = AI::GraphAlgorithms::GraphAlgos::ASTAR_ONE;
}

void ofApp::StartDijkstraOnLargeGraph()
{
	ReadGraphFiles(true);
	std::string whatever = startNodeIF.getParameter().toString();
	int startNode = std::atoi(whatever.c_str());
	whatever = endNodeIF.getParameter().toString();
	int endNode = std::atoi(whatever.c_str());
	startNode = max(0, startNode);
	startNode = min(startNode, downloadedLargeGraph->GetTotalNodes());

	endNode = max(0, endNode);
	endNode = min(endNode, downloadedLargeGraph->GetTotalNodes());
	if (endNode == 0) {
		endNode = 35;
	}
	std::cout << "Start Node : " << startNode << "	end node : " << endNode << std::endl;
	std::cout << "Path is as follows" << std::endl;
	auto path = AI::Pathfinding::FindPathUsingDijkstra(downloadedLargeGraph, startNode, endNode);
	if (path == NULL) {
		std::cout << "No Path Found" << std::endl;
	}
	currentAlgorithm = AI::GraphAlgorithms::GraphAlgos::DIJKSTRA_LARGE;
}

void ofApp::StartAStarOnLargeGraph()
{
	ReadGraphFiles(true);
	std::string whatever = startNodeIF.getParameter().toString();
	int startNode = std::atoi(whatever.c_str());
	whatever = endNodeIF.getParameter().toString();
	int endNode = std::atoi(whatever.c_str());
	startNode = max(0, startNode);
	startNode = min(startNode, downloadedLargeGraph->GetTotalNodes());

	endNode = max(0, endNode);
	endNode = min(endNode, downloadedLargeGraph->GetTotalNodes());
	if (endNode == 0) {
		endNode = 35;
	}
	std::cout << "Start Node : " << startNode << "	end node : " << endNode << std::endl;
	std::cout << "Path is as follows" << std::endl;

	auto path = AI::Pathfinding::FindPathUsingAStar(downloadedLargeGraph,
		startNode, endNode, new AI::Pathfinding::Heuristic(endNode, downloadedLargeGraph->GetTotalNodes()));
	if (path == NULL) {
		std::cout << "No Path Found" << std::endl;
	}
	currentAlgorithm = AI::GraphAlgorithms::GraphAlgos::ASTAR_LARGE;
}

//--------------------------------------------------------------


void ofApp::update() {
	if (currentAlgorithm == AI::GraphAlgorithms::GraphAlgos::PATH_FOLLOW) {
		if (startPathFollow) {
			UpdateForPathFollow();
		}
	}
	
	
}

void ofApp::UpdateForPathFollow()
{
	const auto frameRate = ofGetLastFrameTime();

	smoothingRadius = 15;
	if ((boid->GetPositionToDraw() - target.position).GetLength() < smoothingRadius) {
		SetTargetForNextPathIndex();
	}
	targetRadius = 2;
	timeToTarget = 1.0f;
	auto steering = boid->GetKinematicSeekSteering(target, maxSpeed);
	boid->UpdateKinematic(steering, frameRate, false);
	

}

//--------------------------------------------------------------
void ofApp::draw() {
	if (currentAlgorithm == AI::GraphAlgorithms::GraphAlgos::PATH_FOLLOW) {
		DrawPathFollow();
	}
	else if (currentAlgorithm == AI::GraphAlgorithms::GraphAlgos::NONE) {
		panel.draw();
		ofDrawBitmapString("Press T while in path follow to toggle the visibility of the grid",AI::windowWidth/2, AI::windowHeight/2);
	}
	
}

void ofApp::DrawPathFollow()
{
	ofColor black(1, 1, 1);
	DrawBoid(boid, false, black);
	ofSetColor(black);
	if (isShowingGrid) {
		ofSetLineWidth(0.5);
		auto pos = 0;
		//Lines Along X Axis
		for (int i = 0; i < AI::windowWidth / AI::widthOfTile; i++) {
			pos = i * AI::widthOfTile;
			ofDrawLine(pos, 0, pos, AI::windowHeight);
		}
		//Lines Along Y Axis
		for (int i = 0; i < AI::windowHeight / AI::heightOfTile; i++) {
			pos = i * AI::heightOfTile;
			ofDrawLine(0, pos, AI::windowWidth, pos);
		}
	}
	

	//Collision Zones
	auto collisionZones = AI::DivisionSchemes::GetCollisionZones();
	auto position = AI::Math::ZeroVector;
	for(auto collisionZone : collisionZones) {
		position = AI::DivisionSchemes::GetLocalizedPosition(collisionZone);
		ofFill();
		ofDrawRectangle(position.x - (0.5f * AI::widthOfTile), position.y - (0.5f * AI::heightOfTile), AI::widthOfTile, AI::heightOfTile);
	}
	
}

void ofApp::ReadGraphFiles(bool i_isLarge)
{
	if (i_isLarge) {
		auto edges = AI::Utilities::ReadFile("Graphs/out.opsahl-usairport");

		downloadedLargeGraph = new AI::Pathfinding::DirectedGraph(edges);
	}
	else {
		auto edgesForHandwrittenGraph = AI::Utilities::ReadFile("Graphs/Hometown.opsahl-usairport");
		handwrittenGraph = new AI::Pathfinding::DirectedGraph(edgesForHandwrittenGraph);
	}
}

	

void ofApp::SetupForPathFollow()
{
	downloadedLargeGraph = AI::DivisionSchemes::GenerateTileBasedGraph();
	boid = new AI::Agents::Boid(AI::Math::sVector2D(32,108));
	currentAlgorithm = AI::GraphAlgorithms::GraphAlgos::PATH_FOLLOW;
	bWander = false;
}

void ofApp::CalculatePathForPathFollow(AI::Math::sVector2D position)
{
	int startNode = AI::DivisionSchemes::GetQuantizedNode(boid->GetPositionToDraw());
	int endNode = AI::DivisionSchemes::GetQuantizedNode(position);

	path = AI::Pathfinding::FindPathUsingAStar(
		downloadedLargeGraph, startNode, endNode,
		new AI::Pathfinding::Heuristic(endNode, downloadedLargeGraph));
	if (path != NULL) {
		pathIndex = -1;
		std::cout << " Path follow start from position " << boid->GetPositionToDraw().x << " --- " << boid->GetPositionToDraw().y << std::endl;
		SetTargetForNextPathIndex();
		startPathFollow = true;
	}
	
}

void ofApp::SetTargetForNextPathIndex()
{
	pathIndex++;
	if (pathIndex < path->path.size()) {
		auto edge = path->path[pathIndex];
		auto sink = edge->GetSink();
		auto targetPosition = AI::DivisionSchemes::GetLocalizedPosition(sink);
		std::cout << targetPosition.x << " -- " << targetPosition.y << std::endl;
		target.position = targetPosition;
	}
	else {
		startPathFollow = false;
	}
}

void ofApp::SetupHandWrittenGraph()
{
	ReadGraphFiles(false);
	int startNode = 1;
	int endNode = 20;
	AI::Pathfinding::FindPathUsingDijkstra(handwrittenGraph, startNode, endNode);
	AI::Pathfinding::FindPathUsingAStar(handwrittenGraph,
		startNode, endNode, new AI::Pathfinding::Heuristic(endNode, 20));
	/*AI::Pathfinding::FindPathUsingDijkstra(directedGraph, startNode, endNode);
	AI::Pathfinding::FindPathUsingAStar(
		directedGraph, startNode, endNode,
		new AI::Pathfinding::Heuristic(endNode,directedGraph->GetTotalNodes()));

	boid = new AI::Agents::Boid(AI::Math::sVector2D(50, 100));*/
}

void ofApp::SetHeuristicType(const void * sender, bool &pressed)
{
	b_useHeuristicTypeOne = pressed;
}


//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	int desiredKey = 116;
	if (key == desiredKey ) {
		isShowingGrid = !isShowingGrid; 
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
	if (currentAlgorithm == AI::GraphAlgorithms::GraphAlgos::PATH_FOLLOW) {
		auto position = AI::Math::sVector2D(x, y);
		CalculatePathForPathFollow(position);
	}
	
	
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

void ofApp::DrawBoid(AI::Agents::Boid * boid, bool bRotation, ofColor i_color)
{
	ofSetColor(i_color);
	float radius = 10;

	AI::Math::sVector2D position = boid->GetPositionToDraw();
	float x = position.x;
	float y = position.y;
	ofFill();
	ofDrawCircle(x, y, radius);

	double angle = boid->GetOrientationToDraw(bRotation);

	float x1 = x + (radius)* cos(angle);
	float x2 = x - (radius)* cos(angle);
	float y1 = y + (radius)* sin(angle);
	float y2 = y - (radius)* sin(angle);
	float x3 = x + (3 * radius * sin(angle));
	float y3 = y - (3 * radius * cos(angle));
	ofFill();
	ofDrawTriangle(x1, y1, x2, y2, x3, y3);
	DrawBreadCrumbs(boid);
}

void ofApp::DrawFlock()
{
	auto allBoids = flock->GetAllBoids();
	ofColor black(1, 1, 1);
	ofColor blue(0, 0, 255);
	auto leader = flock->GetFlockLeader();
	for (int i = 0; i < allBoids.size(); i++) {
		ofColor finalColor;
		if (i == leader) {
			finalColor = blue;
			DrawBoid(allBoids.at(i), true, finalColor);
		}
		else {
			finalColor = black;
			DrawBoid(allBoids.at(i), false, finalColor);
		}
		
	}

}

void ofApp::DrawBreadCrumbs(AI::Agents::Boid * boid)
{
	for (int i = 0; i < boid->lastFivePositions.size(); i++) {
		ofColor black;
		float alpha = (boid->lastFivePositions.size() - i - 1) * 1.0f / (1.0f *boid->lastFivePositions.size());
		alpha *= 220;

		black.set(1, 1, 1, alpha);
		ofSetColor(black);
		float radius = 10;

		AI::Math::sVector2D position = boid->lastFivePositions.at(i);
		float x = position.x;
		float y = position.y;
		//std::cout << alpha << " -- " << x << " -- " << y << std::endl;
		ofFill();
		ofDrawCircle(x, y, radius);



		double angle = boid->lastFiveOrientations.at(i);
		if (bWander) {
			angle += AI::Math::pi / 2;
		}

		float x1 = x + (radius)* cos(angle);
		float x2 = x - (radius)* cos(angle);
		float y1 = y + (radius)* sin(angle);
		float y2 = y - (radius)* sin(angle);
		float x3 = x + (3 * radius * sin(angle));
		float y3 = y - (3 * radius * cos(angle));
		ofFill();
		ofDrawTriangle(x1, y1, x2, y2, x3, y3);
	}
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}


//for (auto edge : outgoingEdges) {
//	auto endNode = edge->GetSink();
//	float endNodeCost = current.costSoFar + edge->GetCost();
//	int endNodeHeuristic = 0;
//	NodeRecord temp;
//	NodeRecord endNodeRecord;
//	temp.node = endNode;
//	if (closedList.contains(temp.node)) {
//		endNodeRecord = *closedList.find(temp);
//		if (endNodeRecord.costSoFar <= endNodeCost) {
//			continue;
//		}
//		closedList.remove(endNodeRecord.node, endNodeRecord);
//		endNodeHeuristic = endNodeRecord.costSoFar - endNodeCost;
//	}
//	else if (openList.contains(temp.node)) {
//		endNodeRecord = *openList.find(temp);
//		if (endNodeRecord.costSoFar <= endNodeCost) {
//			continue;
//		}
//		endNodeRecord.costSoFar = endNodeCost;
//		endNodeRecord.incomingEdge = edge;
//		endNodeHeuristic = endNodeRecord.costSoFar - endNodeCost;
//	}
//	else {
//		endNodeRecord.node = endNode;
//		endNodeRecord.incomingEdge = NULL;
//		endNodeRecord.costSoFar = endNodeCost;
//		endNodeRecord.incomingEdge = edge;
//		endNodeHeuristic = heuristics->GetEstimate(endNode);
//		endNodeRecord.estimateToGoal = endNodeCost + endNodeHeuristic;
//	}
//
//	if (!openList.contains(endNodeRecord.node)) {
//
//		openList.PushNode(endNodeRecord.node, endNodeRecord);
//	}
//
//}
