#include "Flock.h"
#include <algorithm>


void AI::Agents::Flock::SetupFlock(int numberOfBoids)
{
	auto leaderBoid = new AI::Agents::Boid(AI::Math::sVector2D(flockStartPositionX, flockStartPositionY), 1000);
	allBoids.insert(allBoids.begin(), leaderBoid);
	for (int i = 1; i < numberOfBoids; i++) {

		AI::Math::sVector2D randomPos(flockStartPositionX + (rand()%50 - 50), flockStartPositionY + (rand() % 50 - 50));
		auto boid = new AI::Agents::Boid(randomPos, 125 + (rand() % 25 - 25));
		allBoids.push_back(boid);
	}
}

void AI::Agents::Flock::UpdateFlock(float i_dt, const float i_maxAcceleration, const float i_maxAngularAcceleration, const float  i_maxSpeed, const float i_maxRotation)
{
	wanderOffset = 1500;
	wanderRadius = 5000;
	wanderRate = -5;
	seekAndLookWeight = 0.8f;
	separationWeight = 1.0f;
	velocityMatchWeight = 0.6f;
	
	/*auto steering = leaderBoid->GetKinematicWanderSteering(0.15, 20);
	leaderBoid->UpdateKinematic(steering, i_dt);*/
	

	auto comPosition = Math::ZeroVector;
	auto comVelocity = Math::ZeroVector;
	auto totalMass = 0.0f;
	allBoidsKinematics.clear();
	for (int i = 0; i < allBoids.size(); i++) {
		auto currentBoid = allBoids.at(i);
		auto currentBoidMass = currentBoid->GetMass();
		comPosition += (currentBoid->GetKinematicData().position * currentBoidMass);
		comPosition += (currentBoid->GetKinematicData().velocity * currentBoidMass);
		totalMass += currentBoidMass;
		allBoidsKinematics.push_back(currentBoid->GetKinematicData());
	}
	comPosition /= totalMass;
	comVelocity /= totalMass;

	Movement::sKinematicData comKinematics;
	comKinematics.position = comPosition;
	comKinematics.velocity = comVelocity;

	auto leaderBoid = allBoids.at(flockLeader);
	auto steering = leaderBoid->GetDynamicWanderSteering(i_maxAcceleration, i_maxAngularAcceleration, wanderOffset, wanderRadius, wanderRate, i_maxRotation, targetAngle, slowAngle);



	auto allKinematicsExceptThis = allBoidsKinematics;
	std::vector<Movement::sKinematicData>::iterator position = std::find(allKinematicsExceptThis.begin(), allKinematicsExceptThis.end(), allKinematicsExceptThis.at(0));

	allKinematicsExceptThis.erase(position);

	auto separationSteering = allBoids.at(0)->GetDynamicSeparationSteering(allKinematicsExceptThis, i_maxAcceleration, personalRadius);
	steering.linear = steering.linear * 0.8f;
	steering.linear += separationSteering.linear * 1.0f;
	steering.angular = steering.angular * 1.0f;
	steering.angular += separationSteering.angular * 1.0f;
	leaderBoid->UpdateDynamic(steering, i_dt, 25, i_maxRotation);


	for (int i = 1; i < allBoids.size(); i++) {
		auto currentBoid = allBoids.at(i);
		auto speed = i_maxSpeed + (std::rand() % 10 - 10);
		auto seekSteering = currentBoid->GetDynamicArriveSteering(comKinematics, i_maxAcceleration, speed, targetRadius, slowRadius, 2.0f);
		auto faceSteering = currentBoid->GetDynamicLookWhereYouAreGoingSteering(i_maxAngularAcceleration, i_maxRotation, targetAngle, slowAngle, 1.0f);
		seekSteering.angular = faceSteering.angular;
		auto allKinematicsExceptThis = allBoidsKinematics;
		std::vector<Movement::sKinematicData>::iterator position = std::find(allKinematicsExceptThis.begin(), allKinematicsExceptThis.end(), currentBoid->GetKinematicData());
		
		allKinematicsExceptThis.erase(position);
		allKinematicsExceptThis.insert(allKinematicsExceptThis.begin(), allBoids.at(0)->GetKinematicData());
		auto separationSteering = currentBoid->GetDynamicSeparationSteering(allKinematicsExceptThis, i_maxAcceleration, personalRadius);
		auto velMatchSteering = currentBoid->GetDynamicVelocityMatchSteering(comKinematics, i_maxAcceleration, speed, 1.0f);

		Movement::sSteeringDynamic finalSteering;
		finalSteering.linear = ( seekSteering.linear* seekAndLookWeight) + (separationSteering.linear * separationWeight) + (velMatchSteering.linear * velocityMatchWeight);
		finalSteering.angular = (seekSteering.angular* seekAndLookWeight) + (separationSteering.angular * separationWeight) + (velMatchSteering.angular * velocityMatchWeight);
		currentBoid->UpdateDynamic(finalSteering, i_dt, speed, i_maxRotation);
	}
	

}

std::vector<AI::Agents::Boid*> AI::Agents::Flock::GetAllBoids()
{
	return allBoids;
}

int AI::Agents::Flock::GetFlockLeader()
{
	return 0;
}
