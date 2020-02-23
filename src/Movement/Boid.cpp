#include "Boid.h"
#include<iostream>
#include <random>
AI::Math::sVector2D AI::Agents::Boid::GetPositionToDraw()
{
	return sKinematics.position;
}

float AI::Agents::Boid::GetOrientationToDraw(bool i_wander)
{
	return i_wander?sKinematics.orientation + AI::Math::pi: sKinematics.orientation + AI::Math::pi/2;
}



void AI::Agents::Boid::UpdateKinematic(AI::Movement::sSteeringKinematic i_steering, float i_dt,bool bWander)
{
	sKinematics.UpdateKinematic(i_steering, i_dt);
	if (!bWander) {
		sKinematics.OrientToDirection();
	}
	StoreLastFiveLocations();
	
}

void AI::Agents::Boid::UpdateDynamic(Movement::sSteeringDynamic i_steering, float i_dt, const float i_maxSpeed, const float i_maxRotation)
{
	sKinematics.UpdateDynamic(i_steering, i_dt,i_maxSpeed,i_maxRotation);
	StoreLastFiveLocations();
}



void AI::Agents::Boid::UpdateForBasicMotion(float i_dt)
{
	float moveVelocity = 50.0f;
	Math::sVector2D velocity = sKinematics.velocity;
	
	AI::Movement::sSteeringKinematic steering;
	steering.velocity = velocity;

	sKinematics.UpdateKinematic(steering, i_dt);
	const auto x = sKinematics.position.x;
	const auto y = sKinematics.position.y;
	if (x > 490 && y == 490) {
		sKinematics.position = Math::sVector2D(490, 490);
		sKinematics.velocity = Math::sVector2D(0, -moveVelocity);
		sKinematics.OrientToDirection();
	}
	else if (x == 490 && y <= 10) {
		sKinematics.position = Math::sVector2D(490, 10);
		sKinematics.velocity = Math::sVector2D(-moveVelocity, 0.0f);
		sKinematics.OrientToDirection();
	}
	else if (x <= 10 && y == 10) {
		sKinematics.position = Math::sVector2D(10, 10);
		sKinematics.velocity = Math::sVector2D(0.0f, moveVelocity);
		sKinematics.OrientToDirection();
	}
	else if (x == 10 && y > 490) {
		sKinematics.position = Math::sVector2D(10, 490);
		sKinematics.velocity = Math::sVector2D();
		sKinematics.OrientToDirection();
	}
	StoreLastFiveLocations();
} 

void AI::Agents::Boid::StoreLastFiveLocations()
{
	if (lastFivePositions.size() == 0) {
		lastFivePositions.insert(lastFivePositions.begin(), sKinematics.position);
		lastFiveOrientations.insert(lastFiveOrientations.begin(), sKinematics.orientation + (Math::pi / 2));
	}
	else if (lastFivePositions.size() > 0 && lastFivePositions.size() < 20) {
		auto lastPosition = lastFivePositions.at(0);
		auto currentPosition = sKinematics.position - lastPosition;
		if (currentPosition.GetLength() > 50) {
			lastFivePositions.insert(lastFivePositions.begin(), sKinematics.position);
			lastFiveOrientations.insert(lastFiveOrientations.begin(), sKinematics.orientation + (Math::pi / 2));
		}
	}
	else {
		auto lastPosition = lastFivePositions.at(0);
		auto currentPosition = sKinematics.position - lastPosition;
		if (currentPosition.GetLength() > 50) {
			lastFivePositions.pop_back();
			lastFivePositions.insert(lastFivePositions.begin(), sKinematics.position);
			lastFiveOrientations.insert(lastFiveOrientations.begin(), sKinematics.orientation + (Math::pi / 2));
		}
		
	}
	
}



void AI::Agents::Boid::SetBasicMotion()
{
	sKinematics.position = Math::sVector2D(10, 490);
	sKinematics.velocity = Math::sVector2D(50, 0);
}

void AI::Agents::Boid::HandleBoundaries(const float i_windowWidth, const float i_windowHeight)
{
	float x = sKinematics.position.x;
	float y = sKinematics.position.y;
	if (x > i_windowWidth) {
		x -=  i_windowWidth;
	}
	else if (x < 0.0f) {
		x += i_windowWidth ;
	}
	if (y > i_windowHeight) {
		y -= i_windowHeight;
	}
	else if (y < 0.0f) {
		y += i_windowHeight;
	}
	sKinematics.position = Math::sVector2D(x, y);
}

AI::Movement::sKinematicData AI::Agents::Boid::GetKinematicData()
{
	return sKinematics;
}

AI::Agents::Boid::Boid(Math::sVector2D i_position)
{
	sKinematics.position = i_position;
}

AI::Agents::Boid::Boid(Math::sVector2D i_position, float i_mass)
{
	sKinematics.position = i_position;
	mass = i_mass;
}

AI::Movement::sSteeringKinematic AI::Agents::Boid::GetKinematicSeekSteering(Movement::sKinematicData i_target, const float i_maxSpeed)
{
	Movement::sSteeringKinematic steering;
	steering.rotation = 0.0f;
	steering.velocity = i_target.position - sKinematics.position;
	steering.velocity = steering.velocity.GetNormalizedVector();
	steering.velocity *= i_maxSpeed;
	//sKinematics.orientation = sKinematics.GetOrientationFromDirection(steering.velocity);

	return steering;

}

AI::Movement::sSteeringKinematic AI::Agents::Boid::GetKinematicArriveSteering(Movement::sKinematicData i_target, const float i_maxSpeed, const float i_targetRadius, const float i_timeToTarget)
{
	Movement::sSteeringKinematic steering;
	Math::sVector2D distanceVector = i_target.position - sKinematics.position;
	if (distanceVector.GetLength() < i_targetRadius) {
		steering.velocity = Math::ZeroVector;
		
	}
	else {
		steering.velocity = (distanceVector) / i_timeToTarget;
		sKinematics.orientation = sKinematics.GetOrientationFromDirection(steering.velocity);
	}
	if (steering.velocity.GetLength() > i_maxSpeed) {
		steering.velocity.Normalize();
		steering.velocity *= i_maxSpeed;
		
	}
	
	return steering;

}
namespace {
	float GetRandomBinom() {
		float random = (rand()* 1.0f) / ((RAND_MAX) * 1.0f);
		float anotherRandom = (rand()* 1.0f) / ((RAND_MAX) * 1.0f);
		return random - anotherRandom;
	}
}
AI::Movement::sSteeringKinematic AI::Agents::Boid::GetKinematicWanderSteering(const float i_maxRotation, const float i_maxSpeed)
{
	Movement::sSteeringKinematic steering;
	auto scale = GetRandomBinom();
	
	steering.rotation = scale * i_maxRotation;
	steering.velocity = sKinematics.GetUnitVectorAlongOrientation(sKinematics.orientation) * i_maxSpeed;

	return steering;

}

AI::Movement::sSteeringDynamic AI::Agents::Boid::GetDynamicArriveSteering(Movement::sKinematicData i_target, const float i_maxLinearAcceleration, const float i_maxSpeed, const float i_targetRadius, const float i_slowRadius, const float i_timeToTarget)
{
	Movement::sSteeringDynamic steering;
	Math::sVector2D distance = i_target.position - sKinematics.position;
	float factor = 0.0f;
	float distanceInFloat = distance.GetLength();
	if (distanceInFloat > i_slowRadius) {
		factor = 1.0f;
	}
	else if (distanceInFloat < i_slowRadius && distanceInFloat > i_targetRadius) {
		factor = distanceInFloat / i_slowRadius;
	}
	else if (distanceInFloat < i_targetRadius) {
		factor = 0.0f;
	}

	float magnitudeV = i_maxSpeed * factor;
	steering.linear = distance.GetNormalizedVector() * magnitudeV - sKinematics.velocity;
	steering.linear /= i_timeToTarget;
	if (steering.linear.GetLength() > i_maxLinearAcceleration) {
		steering.linear = distance.GetNormalizedVector() * i_maxLinearAcceleration;
	}
	return steering;
}

AI::Movement::sSteeringDynamic AI::Agents::Boid::GetDynamicAlignSteering(Movement::sKinematicData i_target, const float i_maxAngularAcceleration, const float i_maxRotation, const float i_targetAngle, const float i_slowAngle, const float i_timeToTarget)
{
	Movement::sSteeringDynamic steering;
	float factor = 0.0f;
	float targetRotation = i_target.orientation - sKinematics.orientation;
	
	while (targetRotation > Math::pi || targetRotation < -Math::pi) {
		if (targetRotation < -Math::pi) {
			targetRotation += (2 * Math::pi);
		}
		else if (targetRotation > Math::pi) {
			targetRotation -= (2 * Math::pi);
		}
	}
	
	float rotationSize = abs(targetRotation);
	if (rotationSize > i_slowAngle) {
		factor = 1.0f;
	}
	else if (rotationSize < i_slowAngle && rotationSize > i_targetAngle) {
		factor = rotationSize / i_slowAngle;
	}
	else if (rotationSize < i_targetAngle) {
		factor = 0.0f;
	}
	float direction = targetRotation / rotationSize;
	float magnitudeRotation = factor * i_maxRotation ;
	steering.angular =  (direction * magnitudeRotation) - sKinematics.rotation;
	steering.angular /= i_timeToTarget;
	if (abs(steering.angular) > i_maxAngularAcceleration) {
		steering.angular /=abs(steering.angular);
		steering.angular *= i_maxAngularAcceleration;
	}
	//std::cout << steering.angular << std::endl;
	steering.linear = Math::ZeroVector;
	return steering;


}

AI::Movement::sSteeringDynamic AI::Agents::Boid::GetDynamicVelocityMatchSteering(Movement::sKinematicData i_target, const float i_maxLinearAcceleration, const float i_maxSpeed, const float i_timeToTarget)
{
	Movement::sSteeringDynamic steering;
	float magnitudeV = (i_target.velocity).GetLength() * i_maxSpeed;
	
	steering.linear = i_target.velocity -sKinematics.velocity;
	steering.linear /= i_timeToTarget;
	
	if (steering.linear.GetLength() > i_maxLinearAcceleration) {
		steering.linear.Normalize();
		steering.linear *= i_maxLinearAcceleration;
	}
	//std::cout << sKinematics.velocity.x << std::endl;
	steering.angular = 0.0f;
	return steering;
}

AI::Movement::sSteeringDynamic AI::Agents::Boid::GetDynamicEvadeSteering(Movement::sKinematicData i_target, const float i_maxLinearAcceleration, const float i_personalRadius)
{
	Movement::sSteeringDynamic steering;
	Math::sVector2D distance = sKinematics.position - i_target.position;
	float distanceInFloat = distance.GetLength();
	if (distanceInFloat < i_personalRadius) {
		/*float repulsion = Math::decayCoefficient / (distanceInFloat * distanceInFloat);
		
		repulsion = repulsion > i_maxLinearAcceleration ? i_maxLinearAcceleration : repulsion;
		steering.linear = distance.GetNormalizedVector() * repulsion;*/
		return GetDynamicFleeSteering(i_target, i_maxLinearAcceleration);

	}
	else {
		steering.linear = Math::ZeroVector;
	}
	//std::cout << steering.linear.x << " -- "<< sKinematics.velocity.x<<std::endl;
	return steering;
}

AI::Movement::sSteeringDynamic AI::Agents::Boid::GetDynamicSeparationSteering(std::vector<Movement::sKinematicData> i_targets, const float i_maxLinearAcceleration, const float i_personalRadius)
{
	Movement::sSteeringDynamic steering;
	for (int i = 0; i < i_targets.size(); i++) {
		steering.linear += GetDynamicEvadeSteering(i_targets.at(i), i_maxLinearAcceleration, i_personalRadius).linear;
	}
	if (steering.linear.GetLength() > i_maxLinearAcceleration) {
		steering.linear.Normalize();
		steering.linear *= i_maxLinearAcceleration;
	}
	return steering;

}

AI::Movement::sSteeringDynamic AI::Agents::Boid::GetDynamicFleeSteering(Movement::sKinematicData i_target, const float i_maxLinearAcceleration)
{
	Movement::sSteeringDynamic steering;
	steering.linear = sKinematics.position - i_target.position;
	steering.linear.Normalize();
	steering.linear *= i_maxLinearAcceleration;
	steering.angular = 0.0f;
	return steering;
}

AI::Movement::sSteeringDynamic AI::Agents::Boid::GetDynamicSeekSteering(Movement::sKinematicData i_target, const float i_maxLinearAcceleration)
{
	Movement::sSteeringDynamic steering;
	steering.linear = i_target.position - sKinematics.position;
	steering.linear.Normalize();
	steering.linear *= i_maxLinearAcceleration;
	steering.angular = 0.0f;
	return steering;
}

AI::Movement::sSteeringDynamic AI::Agents::Boid::GetDynamicWanderSteering(const float i_maxLinearAcceleration, const float i_maxAngularAcceleration, const float i_wanderOffset, const float i_wanderRadius, const float i_wanderRate, const float i_maxRotation, const float i_targetAngle, const float i_slowAngle)
{
	
	auto wanderOrientation = sKinematics.wanderOrientation;
	wanderOrientation += GetRandomBinom() * i_wanderRate;
	sKinematics.wanderOrientation = wanderOrientation;
	auto targetOrientation = wanderOrientation + sKinematics.orientation;

	auto target = sKinematics.position + (sKinematics.GetUnitVectorAlongOrientation(sKinematics.orientation)* i_wanderOffset );

	target += sKinematics.GetUnitVectorAlongOrientation(targetOrientation) * i_wanderRadius;
	Movement::sKinematicData targetForFace;
	targetForFace.position = target;


	auto steering = GetDynamicFaceSteering(targetForFace, i_maxAngularAcceleration, i_maxRotation, i_targetAngle, i_slowAngle, 1.0f);

	steering.linear =sKinematics.GetUnitVectorAlongOrientation(sKinematics.orientation)*  i_maxLinearAcceleration;
	return steering;
}



AI::Movement::sSteeringDynamic AI::Agents::Boid::GetDynamicFaceSteering(Movement::sKinematicData i_target, const float i_maxAngularAcceleration, const float i_maxRotation, const float i_targetAngle, const float i_slowAngle, const float i_timeToTarget)
{
	Movement::sSteeringDynamic steering;
	auto direction = i_target.position - sKinematics.position;
	if (direction.GetLength() == 0.0) {
		return steering;
	}
	auto target = i_target;
	target.orientation = std::atan2f(direction.y, direction.x);
	return GetDynamicAlignSteering(target, i_maxAngularAcceleration, i_maxRotation, i_targetAngle, i_slowAngle, i_timeToTarget);
}

AI::Movement::sSteeringDynamic AI::Agents::Boid::GetDynamicLookWhereYouAreGoingSteering(const float i_maxAngularAcceleration, const float i_maxRotation, const float i_targetAngle, const float i_slowAngle, const float i_timeToTarget)
{
	Movement::sSteeringDynamic steering;
	if (sKinematics.velocity.GetLength() == 0.0f) {
		return steering;
	}
	Movement::sKinematicData targetKinematic;
	targetKinematic.orientation = std::atan2(sKinematics.velocity.y, sKinematics.velocity.x);
	return GetDynamicAlignSteering(targetKinematic, i_maxAngularAcceleration, i_maxRotation, i_targetAngle, i_slowAngle, i_timeToTarget);
	
}




