#pragma once

#include "ofMain.h"
#include "ofxGui/src/ofxGui.h";
#include "Movement/Boid.h"
#include "Movement/Flock.h"
#include "Pathfinding/DirectedGraph.h"
#include "Pathfinding/Path.h"
namespace AI {
	
	const float windowWidth = 1280;
	const float windowHeight = 720;
	const int heightOfTile = 48;
	const int widthOfTile = 64;
	
	namespace Behaviors {
		enum Behaviors {
			BASIC_MOTION,

			KINEMATIC_SEEK,
			KINEMATIC_WANDER,
			KINEMATIC_ARRIVE,

			DYNAMIC_SEEK,
			DYNAMIC_WANDER,
			DYNAMIC_ARRIVE,
			DYNAMIC_ALIGN,
			DYNAMIC_FACE,
			DYNAMIC_EVADE,
			DYNAMIC_LOOK_WHERE_YOU_GOING,
			DYNAMIC_SEPARATION,
			DYNAMIC_VELOCITYMATCH,
			DYNAMIC_FLEE,

			FLOCKING,
			NONE,

		};
	}

	namespace GraphAlgorithms {
		enum GraphAlgos {
			ASTAR_ONE,
			ASTAR_LARGE,
			DIJKSTRA_ONE,
			DIJKSTRA_LARGE,
			PATH_FOLLOW,
			NONE,
		};
	}
	
}
class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void UpdateForPathFollow();
		void draw();
		void DrawPathFollow();
		void ReadGraphFiles(bool i_isLarge);
		void SetupForPathFollow();
		void CalculatePathForPathFollow(AI::Math::sVector2D position);
		void SetTargetForNextPathIndex();
		void SetupHandWrittenGraph();
		void SetHeuristicType(const void * sender, bool &pressed);
		void SetupForMovementAlgorithms();
		void UpdateForMovementAlgorithms();
		void DrawForMovementAlgorithms();
		void StartDijkstraOnSmallGraph();
		void StartAStarOnSmallGraph();
		void StartDijkstraOnLargeGraph();
		void StartAStarOnLargeGraph();
		void StartBehavior(AI::Behaviors::Behaviors i_behaviorType);
		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		
		void DrawBoid(AI::Agents::Boid * boid, bool i_bRotation, ofColor i_color);
		void DrawFlock();
		void DrawBreadCrumbs(AI::Agents::Boid * boid);
		float 	counter;
		bool	bSmooth;
		bool bWander = false;
		AI::Agents::Boid * boid;
		AI::Movement::sKinematicData target;
		float maxSpeed = 35.0f;
		float maxLinearAcceleration = 15.0f;
		float targetRadius = 15.0f;
		float slowRadius = 35.0f;
		float timeToTarget = 2.0f;
		const float personalRadius = 200.0f;

		float maxRotation = 0.26f;
		float maxAngularAcceleration = 0.12f;
		float targetAngle = 0.2f;
		float slowAngle = 0.6f;

		float wanderOffset = 100.0f;
		float wanderRadius = 25.0f;
		float wanderRate = 0.5f;

		bool startPathFollow = false;
		bool isShowingGrid = true;
		bool b_useHeuristicTypeOne = true;
		int pathIndex = 0;
		float smoothingRadius = 2.0f;

		AI::Agents::Boid * targetBoid;
		AI::Agents::Flock * flock;
		std::vector<AI::Agents::Boid *> targetBoids;
		std::vector<AI::Movement::sKinematicData> targetKinematics;
		AI::Behaviors::Behaviors currentBehavior = AI::Behaviors::Behaviors::BASIC_MOTION;

		AI::GraphAlgorithms::GraphAlgos currentAlgorithm = AI::GraphAlgorithms::GraphAlgos::NONE;

		AI::Pathfinding::DirectedGraph * downloadedLargeGraph;
		AI::Pathfinding::DirectedGraph * handwrittenGraph;
		AI::Pathfinding::Path * path;

		ofxPanel panel;
		ofxButton runDijkstraButtononGraphOne;
		ofxButton runAStarOnGraphOneButton;
		ofxButton runDijkstraOnLargeGraphButton;
		ofxButton runAStarOnLargeGraphButton;
		ofxButton runPathFollowButton;
		ofxToggle useHeuristicTypeOne;

		ofxTextField startNodeIF;
		ofxTextField endNodeIF;


};

