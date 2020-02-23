#include "ofApp.h"
#include <iostream>
#include <cmath>
#include "Pathfinding/Utilities.h"
#include "Pathfinding/PathfindingAlgorithms.h"
#include "Pathfinding/Heuristic.h"
#include "Pathfinding/DivisionSchemeAlgorithms.h"

//--------------------------------------------------------------
void ofApp::setup() {
	counter = 0;
	ofSetCircleResolution(50);
	ofBackground(255, 255, 255);
	bSmooth = false;
	ofSetWindowTitle("Pathfinding Algorithms");

	ofSetFrameRate(60);

	SetupForPathFollow();
	//SetupHandWrittenGraph();
	
}
//--------------------------------------------------------------
void ofApp::update() {
	if (startPathFollow) {
		UpdateForPathFollow();
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
	DrawPathFollow();
}

void ofApp::DrawPathFollow()
{
	ofColor black(1, 1, 1);
	DrawBoid(boid, false, black);
}

void ofApp::ReadGraphFiles()
{
	auto edges = AI::Utilities::ReadFile("Graphs/out.opsahl-usairport");
	directedGraph = new AI::Pathfinding::DirectedGraph(edges);
}

void ofApp::SetupForPathFollow()
{
	directedGraph = AI::DivisionSchemes::GenerateTileBasedGraph();
	boid = new AI::Agents::Boid(AI::Math::sVector2D(50, 100));

	bWander = false;
}

void ofApp::CalculatePathForPathFollow(AI::Math::sVector2D position)
{
	int startNode = AI::DivisionSchemes::GetQuantizedNode(boid->GetPositionToDraw());
	int endNode = AI::DivisionSchemes::GetQuantizedNode(position);

	path = AI::Pathfinding::FindPathUsingAStar(
		directedGraph, startNode, endNode,
		new AI::Pathfinding::Heuristic(endNode, directedGraph));
	pathIndex = -1;
	std::cout << " Path follow start from position " << boid->GetPositionToDraw().x << " --- " << boid->GetPositionToDraw().y << std::endl;
	SetTargetForNextPathIndex();
	startPathFollow = true;
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
	ReadGraphFiles();
	int startNode = 222;
	int endNode = 777;
	AI::Pathfinding::FindPathUsingDijkstra(directedGraph, startNode, endNode);
	AI::Pathfinding::FindPathUsingAStar(
		directedGraph, startNode, endNode,
		new AI::Pathfinding::Heuristic(endNode,directedGraph->GetTotalNodes()));

	boid = new AI::Agents::Boid(AI::Math::sVector2D(50, 100));
}


//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

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
	auto position = AI::Math::sVector2D(x, y);
	CalculatePathForPathFollow(position);

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
