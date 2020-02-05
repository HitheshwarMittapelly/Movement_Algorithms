#ifndef STEERING_H
#define STEERING_H
#include "sVector2D.h"

namespace AI {
	namespace Movement {
		struct sSteeringKinematic {
			Math::sVector2D velocity;
			float rotation = 0.0f;
		};

		struct sSteeringDynamic {
			Math::sVector2D linear;
			float angular = 0.0f;
		};
	}
}



#endif // !STEERING_H
