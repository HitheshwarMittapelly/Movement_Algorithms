#include "ofApp.h"
#include <iostream>
#include <cmath>



//--------------------------------------------------------------
void ofApp::setup() {
	counter = 0;
	ofSetCircleResolution(50);
	ofBackground(255, 255, 255);
	bSmooth = false;
	ofSetWindowTitle("Movement Algorithms");

	ofSetFrameRate(60);
	boid = new AI::Agents::Boid(AI::Math::sVector2D(50, 100));
	
	bWander = false;

	currentBehavior = AI::Behaviors::Behaviors::NONE;
	
	basicMotionButton.addListener(this, &ofApp::StartBasicMotion);
	seekButton.addListener(this, &ofApp::StartSeekBehavior);
	wanderButton.addListener(this, &ofApp::StartWanderBehavior);
	flockingButton.addListener(this, &ofApp::StartFlockBehavior);

	panel.setup();
	panel.add(basicMotionButton.setup("Basic Motion"));
	panel.add(seekButton.setup("Seek Steering"));
	panel.add(wanderButton.setup("Wander Steering"));
	panel.add(flockingButton.setup("Flocking Behavior"));
	

	//target.velocity = AI::Math::sVector2D(20, 10);
	

	//StartBehavior(AI::Behaviors::Behaviors::DYNAMIC_WANDER);
	
}
void ofApp::StartBasicMotion()
{
	StartBehavior(AI::Behaviors::Behaviors::BASIC_MOTION);
}

void ofApp::StartSeekBehavior()
{
	StartBehavior(AI::Behaviors::Behaviors::KINEMATIC_SEEK);

}

void ofApp::StartWanderBehavior()
{
	StartBehavior(AI::Behaviors::Behaviors::KINEMATIC_WANDER);

}

void ofApp::StartFlockBehavior()
{
	StartBehavior(AI::Behaviors::Behaviors::FLOCKING);

}


void ofApp::StartBehavior(AI::Behaviors::Behaviors i_behaviorType)
{
	auto behaviorChosen = i_behaviorType;
	target.position = AI::Math::sVector2D(250, 250);
	target.orientation = 0.0f;  //10.14f
	boid->lastFivePositions.clear();
	boid->lastFiveOrientations.clear();
	switch (behaviorChosen) {
	case AI::Behaviors::Behaviors::FLOCKING: {
		auto numberOfBoids = 10;
		flock = new AI::Agents::Flock(numberOfBoids);
		currentBehavior = behaviorChosen;
		break;
	}case AI::Behaviors::Behaviors::DYNAMIC_ALIGN: {
		target.orientation = 10.14f;  //10.14f
		currentBehavior = behaviorChosen;
		break;
	}case AI::Behaviors::Behaviors::DYNAMIC_ARRIVE: {
		targetRadius = 20.0f;
		slowRadius = 100.0f;
		target.position = AI::Math::sVector2D(350, 350);
		currentBehavior = behaviorChosen;
		break;
	}
	case AI::Behaviors::Behaviors::DYNAMIC_SEPARATION: {
		for (int i = 0; i < 5; i++) {
			auto boid = new AI::Agents::Boid(AI::Math::sVector2D(i * (rand() % 400), i * (rand() % 500)));
			targetBoids.push_back(boid);
		}
		currentBehavior = behaviorChosen;
		break;
	}
	case AI::Behaviors::Behaviors::BASIC_MOTION: {

		boid->SetBasicMotion();
		currentBehavior = behaviorChosen;
		break;
	}
	case AI::Behaviors::Behaviors::DYNAMIC_VELOCITYMATCH: {
		target.velocity = AI::Math::sVector2D(20, 10);
		currentBehavior = behaviorChosen;
		break;
	}
	default:currentBehavior = behaviorChosen;

	}
	targetBoid = new AI::Agents::Boid(target);
}

//--------------------------------------------------------------
void ofApp::update() {
	counter += 0.033f;
	const auto frameRate = ofGetLastFrameTime();
	switch (currentBehavior) {
		case AI::Behaviors::Behaviors::BASIC_MOTION: {
			boid->UpdateForBasicMotion(frameRate);
			break;
		}case AI::Behaviors::Behaviors::KINEMATIC_SEEK: {
			auto steering = boid->GetKinematicSeekSteering(target, maxSpeed);
			boid->UpdateKinematic(steering, frameRate,bWander);
			break;
		}case AI::Behaviors::Behaviors::KINEMATIC_ARRIVE: {
			maxSpeed = 35;
			targetRadius = 5;
			timeToTarget = 5;
			auto steering = boid->GetKinematicArriveSteering(target, maxSpeed,targetRadius,timeToTarget);
			boid->UpdateKinematic(steering, frameRate, bWander);
			break;
		}
		case AI::Behaviors::Behaviors::KINEMATIC_WANDER: {
			float maxRotationForWander = 1.25f;
			maxSpeed = 100;
			auto steering = boid->GetKinematicWanderSteering(maxRotationForWander, maxSpeed);
			boid->UpdateKinematic(steering, frameRate, bWander);
			bWander = true;
			break;
		}case AI::Behaviors::Behaviors::DYNAMIC_ARRIVE: {
			targetRadius = 10;
			slowRadius = 50;
			maxLinearAcceleration = 15;
			maxSpeed = 35;
			auto steering = boid->GetDynamicArriveSteering(target, maxLinearAcceleration, maxSpeed, targetRadius, slowRadius, timeToTarget);
			auto steeringAngular = boid->GetDynamicLookWhereYouAreGoingSteering(maxAngularAcceleration, maxRotation, targetAngle, slowAngle, 1.0f);
			steering.angular = steeringAngular.angular;
			boid->UpdateDynamic(steering, frameRate,maxSpeed,maxRotation);
			break;
		}case AI::Behaviors::Behaviors::DYNAMIC_ALIGN: {
			
			auto steering = boid->GetDynamicAlignSteering(target, maxAngularAcceleration, maxRotation, targetAngle, slowAngle, timeToTarget);
			
			boid->UpdateDynamic(steering, frameRate,maxSpeed,maxRotation);
			break;
		}case AI::Behaviors::Behaviors::DYNAMIC_VELOCITYMATCH: {
			auto steering = boid->GetDynamicVelocityMatchSteering(target, maxLinearAcceleration, maxSpeed, timeToTarget);
			auto steeringAngular = boid->GetDynamicLookWhereYouAreGoingSteering(maxAngularAcceleration, maxRotation, targetAngle, slowAngle, 1.0f);
			steering.angular = steeringAngular.angular;
			boid->UpdateDynamic(steering, frameRate,maxSpeed,maxRotation);

			AI::Movement::sSteeringKinematic targetSteering;
			targetSteering.velocity = AI::Math::sVector2D(20, 10);
			targetBoid->UpdateKinematic(targetSteering, frameRate,false);
			break;
		}case AI::Behaviors::Behaviors::DYNAMIC_SEEK: {
			maxLinearAcceleration = 15;
			maxSpeed = 35;
			auto steering = boid->GetDynamicSeekSteering(target, maxLinearAcceleration);
			auto steeringAngular = boid->GetDynamicLookWhereYouAreGoingSteering(maxAngularAcceleration, maxRotation, targetAngle, slowAngle, 1.0f);
			steering.angular = steeringAngular.angular;
			boid->UpdateDynamic(steering, frameRate, maxSpeed, maxRotation);
			break;
		}
		case AI::Behaviors::Behaviors::DYNAMIC_FLEE: {
			auto steering = boid->GetDynamicFleeSteering(target, maxLinearAcceleration);
			auto steeringAngular = boid->GetDynamicLookWhereYouAreGoingSteering(maxAngularAcceleration, maxRotation, targetAngle, slowAngle, 1.0f);
			steering.angular = steeringAngular.angular;
			boid->UpdateDynamic(steering, frameRate, maxSpeed, maxRotation);
			break;
		}
		case AI::Behaviors::Behaviors::DYNAMIC_EVADE: {
			auto targetSteering = targetBoid->GetDynamicSeekSteering(boid->GetKinematicData(), maxLinearAcceleration);
			targetBoid->UpdateDynamic(targetSteering, frameRate, maxSpeed, maxRotation);
			auto steering = boid->GetDynamicEvadeSteering(targetBoid->GetKinematicData(), maxLinearAcceleration,personalRadius);
			auto steeringAngular = boid->GetDynamicLookWhereYouAreGoingSteering(maxAngularAcceleration, maxRotation, targetAngle, slowAngle, 1.0f);
			steering.angular = steeringAngular.angular;
			boid->UpdateDynamic(steering, frameRate, maxSpeed, maxRotation);
			break;
		}
		case AI::Behaviors::Behaviors::DYNAMIC_SEPARATION: {
			targetKinematics.clear();

			for (int i = 0; i < targetBoids.size(); i++) {

				auto targetSteering = targetBoids.at(i)->GetDynamicSeekSteering(boid->GetKinematicData(), maxLinearAcceleration);
				targetBoids.at(i)->UpdateDynamic(targetSteering, frameRate, maxSpeed, maxRotation);
				targetKinematics.push_back(targetBoids.at(i)->GetKinematicData());
			}

			std::cout << targetKinematics.size() << std::endl;
			auto steering = boid->GetDynamicSeparationSteering(targetKinematics, maxLinearAcceleration, personalRadius); 
			auto steeringAngular = boid->GetDynamicLookWhereYouAreGoingSteering(maxAngularAcceleration, maxRotation, targetAngle, slowAngle, 1.0f);
			steering.angular = steeringAngular.angular;
			boid->UpdateDynamic(steering, frameRate, maxSpeed, maxRotation);
			break;
		}
		case AI::Behaviors::Behaviors::DYNAMIC_LOOK_WHERE_YOU_GOING: {
			auto steeringLinear = boid->GetDynamicSeekSteering(target, maxLinearAcceleration);
			auto steeringAngular = boid->GetDynamicLookWhereYouAreGoingSteering(maxAngularAcceleration, maxRotation, targetAngle, slowAngle, 1.0f);
			auto arbitrarySteering = steeringLinear;
			arbitrarySteering.angular = steeringAngular.angular;

			boid->UpdateDynamic(arbitrarySteering, frameRate, maxSpeed, maxRotation);
			break;
		}
		case AI::Behaviors::Behaviors::DYNAMIC_WANDER: {
			maxLinearAcceleration = 35;
			maxSpeed = 35;
			maxAngularAcceleration = 0.5f;
			wanderOffset = 1500;
			wanderRadius = 5000;
			wanderRate = -5;
			maxRotation = 0.15;
			auto steering = boid->GetDynamicWanderSteering(maxLinearAcceleration, maxAngularAcceleration, wanderOffset, wanderRadius, wanderRate, maxRotation, targetAngle, slowAngle);
			boid->UpdateDynamic(steering, frameRate, maxSpeed, maxRotation);
			bWander = true;
			break;
		}case AI::Behaviors::Behaviors::DYNAMIC_FACE: {
			auto steering = boid->GetDynamicFaceSteering(target, maxAngularAcceleration, maxRotation, targetAngle, slowAngle, timeToTarget);
			boid->UpdateDynamic(steering, frameRate, maxSpeed, maxRotation);
			break;
		}case AI::Behaviors::Behaviors::FLOCKING: {
			maxLinearAcceleration = 50;
			maxSpeed = 30;
			maxAngularAcceleration = 0.1f;
			
			maxRotation = 0.15f;
			flock->UpdateFlock(frameRate, maxLinearAcceleration, maxAngularAcceleration, maxSpeed, maxRotation);
			auto allBoids = flock->GetAllBoids();
			for (int i = 0; i < allBoids.size(); i++) {
				auto currentBoid = allBoids.at(i);
				currentBoid->HandleBoundaries(AI::windowWidth, AI::windowHeight);
			}
			break;
		}
		

	}
	
	
	//Toroidal world
	if (currentBehavior != AI::Behaviors::Behaviors::NONE) {
		boid->HandleBoundaries(AI::windowWidth, AI::windowHeight);
		targetBoid->HandleBoundaries(AI::windowWidth, AI::windowHeight);
	}
	
}

//--------------------------------------------------------------
void ofApp::draw() {
	ofColor black(1, 1, 1);
	ofColor red(255, 0, 0);
	
	switch (currentBehavior) {
	case AI::Behaviors::Behaviors::NONE: {
		panel.draw();
		break;
	}
	case AI::Behaviors::Behaviors::FLOCKING:
		DrawFlock();
		break;
	case AI::Behaviors::Behaviors::DYNAMIC_SEPARATION: {
		for (int i = 0; i < targetBoids.size(); i++) {

			DrawBoid(targetBoids.at(i), false, red);
		}
		DrawBoid(boid, bWander, black);
		DrawBreadCrumbs(boid);
		break;
	}
	case AI::Behaviors::Behaviors::BASIC_MOTION: {
		DrawBoid(boid, bWander, black);
		DrawBreadCrumbs(boid);
		break;
	}
	default:
		DrawBoid(boid, bWander, black);
		DrawBoid(targetBoid, false, red);
		DrawBreadCrumbs(boid);
		break;
	}
	
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
	target.position = AI::Math::sVector2D(x, y);
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
