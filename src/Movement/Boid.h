#ifndef BOID_H
#define BOID_H
#include "sKinematicData.h"
#include <vector>
namespace AI {
	namespace Math {
		const float decayCoefficient = 10.0f;
	}

	namespace Agents {
		class Boid {
		public:
			Math::sVector2D GetPositionToDraw();
			float GetOrientationToDraw(bool i_wander);
			void SetVelocity(float i_velocity) {
				sKinematics.velocity += i_velocity;
				sKinematics.OrientToDirection();
				
				
			}
			AI::Movement::sSteeringKinematic GetKinematicSeekSteering(Movement::sKinematicData i_target, const float i_maxSpeed);
			AI::Movement::sSteeringKinematic GetKinematicArriveSteering(Movement::sKinematicData i_target, const float i_maxSpeed,const float i_targetRadius, const float i_timeToTarget);
			AI::Movement::sSteeringKinematic GetKinematicWanderSteering(const float i_maxRotation, const float i_maxSpeed);

			AI::Movement::sSteeringDynamic GetDynamicArriveSteering(Movement::sKinematicData i_target, const float i_maxLinearAcceleration, const float i_maxSpeed, const float i_targetRadius, const float i_slowRadius, const float i_timeToTarget);
			AI::Movement::sSteeringDynamic GetDynamicAlignSteering(Movement::sKinematicData i_target, const float i_maxAngularAcceleration, const float i_maxRotation, const float i_targetAngle, const float i_slowAngle, const float i_timeToTarget);
			AI::Movement::sSteeringDynamic GetDynamicVelocityMatchSteering(Movement::sKinematicData i_target, const float i_maxLinearAcceleration, const float i_maxSpeed, const float i_timeToTarget);
			AI::Movement::sSteeringDynamic GetDynamicEvadeSteering(Movement::sKinematicData i_target, const float i_maxLinearAcceleration,const float i_personalRadius);
			AI::Movement::sSteeringDynamic GetDynamicSeparationSteering(std::vector<Movement::sKinematicData> i_targets, const float i_maxLinearAcceleration,const float i_personalRadius);
			AI::Movement::sSteeringDynamic GetDynamicFleeSteering(Movement::sKinematicData i_target, const float i_maxLinearAcceleration);
			AI::Movement::sSteeringDynamic GetDynamicSeekSteering(Movement::sKinematicData i_target, const float i_maxLinearAcceleration);
			AI::Movement::sSteeringDynamic GetDynamicWanderSteering(const float i_maxLinearAcceleration,const float i_maxAngularAcceleration, const float i_wanderOffset, const float i_wanderRadius,const float i_wanderRate, const float i_maxRotation, const float i_targetAngle, const float i_slowAngle);
			AI::Movement::sSteeringDynamic GetDynamicFaceSteering(Movement::sKinematicData i_target, const float i_maxAngularAcceleration, const float i_maxRotation, const float i_targetAngle, const float i_slowAngle, const float i_timeToTarget);
			AI::Movement::sSteeringDynamic GetDynamicLookWhereYouAreGoingSteering(const float i_maxAngularAcceleration, const float i_maxRotation, const float i_targetAngle, const float i_slowAngle, const float i_timeToTarget);

			
			void UpdateKinematic(Movement::sSteeringKinematic i_steering,float i_dt,bool bWander);
			void UpdateDynamic(Movement::sSteeringDynamic i_steering, float i_dt,const float i_maxSpeed, const float i_maxRotation);
			void UpdateForBasicMotion(float i_dt);
			void StoreLastFiveLocations();
			void SetBasicMotion();
			float GetMass() {
				return mass;
			}
			void HandleBoundaries(const float i_windowWidth, const float i_windowHeight);
			Movement::sKinematicData GetKinematicData();
			Boid() = default;
			Boid(Movement::sKinematicData i_kinematic)
			:sKinematics(i_kinematic){
			}
			
			Boid(Math::sVector2D i_position);
			Boid(Math::sVector2D i_position, float i_mass);
			std::vector<AI::Math::sVector2D> lastFivePositions;
			std::vector<float> lastFiveOrientations;
		private:
			Movement::sKinematicData sKinematics;
			float mass = 100.0f;
			//Steering Behaviors

		};
	}
	
}


#endif // !BOID_H
