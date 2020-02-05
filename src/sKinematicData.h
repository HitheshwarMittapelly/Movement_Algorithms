#ifndef KINEMATICDATA_H
#define KINEMATICDATA_H
#include "sVector2D.h"
#include "sSteering.h"
#include <cmath>
#include <iostream>
namespace AI {
	namespace Movement {
		struct sKinematicData {
			Math::sVector2D position;
			float orientation= 0.0f;
			
			Math::sVector2D velocity;
			float rotation = 0.0f;
			float wanderOrientation = 0.0f;

			void UpdateKinematic(Movement::sSteeringKinematic i_steering, const float i_dt); 
			void UpdateDynamic(Movement::sSteeringDynamic i_steering, const float i_dt, const float i_maxSpeed, const float i_maxRotation); 

			void OrientToDirection();

			float GetOrientationFromDirection(Math::sVector2D i_direction);

			Math::sVector2D GetUnitVectorAlongOrientation(const float i_orientation); 

			bool operator ==(const sKinematicData i_other) const;
		};
	}
}



#endif // !KINEMATICDATA_H

