#include "sVector2D.h"
#include <cmath>
#include <iostream>
using namespace AI;
using namespace Math;

namespace {
	constexpr auto s_epsilon = 1.0e-9f;
}
sVector2D AI::Math::sVector2D::operator+(const sVector2D i_anotherVector) const
{
	return sVector2D( x + i_anotherVector.x, y + i_anotherVector.y);
}

AI::Math::sVector2D & AI::Math::sVector2D::operator+=(const sVector2D i_anotherVector) 
{
	x += i_anotherVector.x;
	y += i_anotherVector.y;
	return *this;
}

sVector2D AI::Math::sVector2D::operator+(const float i_float) const
{
	return sVector2D(x + i_float , y + i_float);
}

sVector2D & AI::Math::sVector2D::operator+=(const float i_float) 
{
	x += i_float;
	y += i_float;
	return *this;
}

sVector2D AI::Math::sVector2D::operator-(const sVector2D i_anotherVector) const
{
	return sVector2D(x- i_anotherVector.x, y-i_anotherVector.y);
}

sVector2D & AI::Math::sVector2D::operator-=(const sVector2D i_anotherVector) 
{
	x -= i_anotherVector.x;
	y -= i_anotherVector.y;
	return *this;
}

sVector2D AI::Math::sVector2D::operator-() const
{
	return sVector2D(-x,-y);
}

sVector2D AI::Math::sVector2D::operator-(const float i_float) const
{
	return sVector2D(x- i_float,y-i_float);
}

sVector2D & AI::Math::sVector2D::operator-=(const float i_float) 
{
	x -= i_float;
	y -= i_float;
	return *this;
}

sVector2D AI::Math::sVector2D::operator*(const float i_float) const
{
	return sVector2D(x * i_float, y* i_float);
}

sVector2D & AI::Math::sVector2D::operator*=(const float i_float) 
{
	x *= i_float;
	y *= i_float;
	return *this;
}

sVector2D AI::Math::sVector2D::operator/(const float i_float) const
{
	return sVector2D(x / i_float, y /i_float);
}

sVector2D & AI::Math::sVector2D::operator/=(const float i_float) 
{
	x /= i_float;
	y /= i_float;
	return *this;
}

float AI::Math::sVector2D::GetLength() const
{
	const auto squaredLength = (x * x) + (y * y);
	if (squaredLength < 0.0f) {
		std::cout << "Some error occured in vector math" << std::endl;
		return 0.0f;
	}
	return std::sqrt(squaredLength);
}

float AI::Math::sVector2D::Normalize()
{
	const auto length = GetLength();
	if (length < s_epsilon) {
		return 0.0f;
	}
	operator /= (length);
	return length;
}

sVector2D AI::Math::sVector2D::GetNormalizedVector() const
{
	const auto length = GetLength();
	if (length < s_epsilon) {
		//std::cout << "This shouldn't happen dude" << std::endl;
		return sVector2D();
	}
	return sVector2D(x/length,y/length);
}

bool AI::Math::sVector2D::operator==(const sVector2D i_Vector) const
{
	return (x == i_Vector.x) & (y == i_Vector.y);
}

bool AI::Math::sVector2D::operator!=(const sVector2D i_Vector) const
{
	return (x != i_Vector.x) | (y != i_Vector.y);
}

AI::Math::sVector2D::sVector2D(const float i_x, const float i_y):
	x(i_x),y(i_y)
{
}

float AI::Math::sVector2D::DotProduct(const sVector2D i_vector)
{
	return (x* i_vector.x) + (y * i_vector.y);
}
