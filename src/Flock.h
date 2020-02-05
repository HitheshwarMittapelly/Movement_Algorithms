#ifndef FLOCK_H
#define FLOCK_H
#include "Boid.h"
namespace AI {
	namespace Agents {
		class Flock {
		public:
			void SetupFlock(int numberOfBoids);
			void UpdateFlock(float i_dt, const float i_maxAcceleration, const float i_maxAngularAcceleration, const float  i_maxSpeed, const float i_maxRotation);
			std::vector<Boid *> GetAllBoids();
			int GetFlockLeader();
			Flock() = default;
			Flock(int numberOfBoids) {
				SetupFlock(numberOfBoids);
			}
		private:
			int numberOfBoids = 0;
			std::vector<Boid *> allBoids;
			std::vector<AI::Movement::sKinematicData> allBoidsKinematics;
			int flockLeader = 0;
			float wanderOffset = 100.0f;
			float wanderRadius = 25.0f;
			float wanderRate = 0.5f;
			const float targetAngle = 0.2f;
			const float slowAngle = 0.6f;
			const float targetRadius = 30.0f;
			const float slowRadius = 75.0f;

			float seekAndLookWeight = 0.8;
			float separationWeight = 1.0f;
			float velocityMatchWeight = 0.6f;

			const float personalRadius = 90.0f;

			const float flockStartPositionX = 400.0f;
			const float flockStartPositionY = 200.0f;
		};
	}

}


#endif // !FLOCK_H

