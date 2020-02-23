#include "sKinematicData.h"

bool AI::Movement::sKinematicData::operator==(const sKinematicData i_other) const
{
	return i_other.position == position & i_other.orientation == orientation & i_other.velocity == velocity & i_other.rotation == rotation;
}

void AI::Movement::sKinematicData::UpdateDynamic(AI::Movement::sSteeringDynamic i_steering, const float i_dt, const float i_maxSpeed, const float i_maxRotation)
{
	position += (velocity * i_dt);
	orientation += (rotation * i_dt);

	velocity += (i_steering.linear * i_dt);
	rotation += (i_steering.angular * i_dt);
	if (velocity.GetLength() > i_maxSpeed) {
		velocity = velocity.GetNormalizedVector() * i_maxSpeed;
	}
	if (abs(rotation) > i_maxRotation) {
		rotation /= abs(rotation);
		rotation *= i_maxRotation;
	}
	//std::cout << "velocity "<< velocity.x << " -- " << velocity.y << std::endl;
}

void AI::Movement::sKinematicData::OrientToDirection()
{
	if (velocity == AI::Math::ZeroVector) {
		return;
	}
	const auto orientationInDirection = GetOrientationFromDirection(velocity);
	orientation = orientationInDirection;
}

float AI::Movement::sKinematicData::GetOrientationFromDirection(AI::Math::sVector2D i_direction)
{
	if (i_direction.x == 0 && i_direction.y == 0) {
		return 0.0f;
	}
	const auto orientationInDirection = std::atan2f(i_direction.y, i_direction.x);
	return orientationInDirection;
}

AI::Math::sVector2D AI::Movement::sKinematicData::GetUnitVectorAlongOrientation(const float i_orientation)
{
	return AI::Math::sVector2D(-std::sin(i_orientation), std::cos(i_orientation));
}

void AI::Movement::sKinematicData::UpdateKinematic(Movement::sSteeringKinematic i_steering, const float i_dt)
{
	position += (velocity * i_dt);
	orientation += (rotation * i_dt);

	velocity = i_steering.velocity;
	rotation = i_steering.rotation;

}
