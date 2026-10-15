// Minimal 3D vector / 3x3 matrix math used by the soft body solver.
// Kept free of any TouchDesigner dependency so the solver can be tested standalone.

#pragma once

#include <cmath>

namespace sb
{

struct Vec3
{
	float x = 0.0f, y = 0.0f, z = 0.0f;

	Vec3() = default;
	Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

	Vec3	operator+(const Vec3& o) const { return { x + o.x, y + o.y, z + o.z }; }
	Vec3	operator-(const Vec3& o) const { return { x - o.x, y - o.y, z - o.z }; }
	Vec3	operator-() const { return { -x, -y, -z }; }
	Vec3	operator*(float s) const { return { x * s, y * s, z * s }; }
	Vec3	operator/(float s) const { return { x / s, y / s, z / s }; }
	Vec3&	operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
	Vec3&	operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
	Vec3&	operator*=(float s) { x *= s; y *= s; z *= s; return *this; }

	float	operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
	float&	operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }
};

inline Vec3		operator*(float s, const Vec3& v) { return v * s; }
inline float	dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3		cross(const Vec3& a, const Vec3& b)
{
	return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}
inline float	length2(const Vec3& v) { return dot(v, v); }
inline float	length(const Vec3& v) { return std::sqrt(dot(v, v)); }
inline Vec3		normalize(const Vec3& v)
{
	float l = length(v);
	return l > 1e-12f ? v / l : Vec3(0.0f, 0.0f, 0.0f);
}
inline Vec3		vmin(const Vec3& a, const Vec3& b)
{
	return { a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y, a.z < b.z ? a.z : b.z };
}
inline Vec3		vmax(const Vec3& a, const Vec3& b)
{
	return { a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y, a.z > b.z ? a.z : b.z };
}

// Row-major 3x3 matrix
struct Mat3
{
	float m[3][3] = { {0,0,0},{0,0,0},{0,0,0} };

	static Mat3 identity()
	{
		Mat3 r;
		r.m[0][0] = r.m[1][1] = r.m[2][2] = 1.0f;
		return r;
	}

	Vec3	col(int c) const { return { m[0][c], m[1][c], m[2][c] }; }
	void	setCol(int c, const Vec3& v) { m[0][c] = v.x; m[1][c] = v.y; m[2][c] = v.z; }

	Vec3	operator*(const Vec3& v) const
	{
		return {
			m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
			m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
			m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
		};
	}

	// this += a * b^T * s
	void	addOuter(const Vec3& a, const Vec3& b, float s)
	{
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
				m[i][j] += a[i] * b[j] * s;
	}
};

}
