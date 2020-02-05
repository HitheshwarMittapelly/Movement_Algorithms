#ifndef SVECTOR2D_H
#define SVECTOR2D_H

#include <cmath>
namespace AI {
	namespace Math {
		const double pi = std::acos(-1);
		struct sVector2D {
			float x = 0.0f, y = 0.0f;

			sVector2D operator +(const sVector2D i_anotherVector) const;
			sVector2D& operator +=(const sVector2D i_anotherVector) ;

			sVector2D operator +(const float i_float) const;
			sVector2D& operator +=(const float i_float) ;

			sVector2D operator -(const sVector2D i_anotherVector) const;
			sVector2D& operator -=(const sVector2D i_anotherVector) ;
			sVector2D operator -() const;

			sVector2D operator -(const float i_float) const;
			sVector2D& operator -=(const float i_float) ;

			sVector2D operator *(const float i_float) const;
			sVector2D& operator *=(const float i_float) ;

			sVector2D operator /(const float i_float) const;
			sVector2D& operator /=(const float i_float) ;

			float GetLength() const;
			float Normalize();
			sVector2D GetNormalizedVector() const;

			bool operator ==(const sVector2D i_Vector) const;
			bool operator !=(const sVector2D i_Vector) const;

			sVector2D() = default;
			sVector2D(const float i_x, const float i_y);

			float DotProduct(const sVector2D i_vector);

		};

		const sVector2D ZeroVector = sVector2D(0.0f, 0.0f);
		const sVector2D UnitVector = sVector2D(1.0f, 1.0f);
	}

	
}


#endif
