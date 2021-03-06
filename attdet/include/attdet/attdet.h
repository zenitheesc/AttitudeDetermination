#if !defined(_ATT_DET_H_)
#define _ATT_DET_H_
#undef ALGLIN_PRECISION
#define ALGLIN_PRECISION (1E-6)
#include <alglin/alglin.hpp>
#include <alglin/array.hpp>
#include <array>
#include <initializer_list>
namespace attdet {
struct Sensor {
	/**
	 * @brief Construct a new Sensor object
	 *
	 * @param measure_  Body frame measurement
	 * @param reference_ Inertial frame value
	 * @param weight_ Relative Weight of the Sensor in QUEST
	 */
	constexpr Sensor() = default;
	Sensor(const Vec3 &measure_, const Vec3 &reference_, double weight_)
	  : measure(measure_), reference(reference_), weight(weight_) {}
	Vec3 measure{};
	Vec3 reference{};
	double weight{};
};

Matrix3 block_matrix(const Vec3 &a, const Vec3 &b, const Vec3 &c);

enum class Rotations { X, Y, Z, None };
Quat quest(const std::initializer_list<Sensor> &sensors);

Matrix3 triad( Sensor const& sensor, Sensor const& sensor2) ;
Vec3 DCM2Euler(const Matrix3 &A);
Vec3 Quat2Euler(const Quat &q);
}// namespace attdet

#endif// _ATT_DET_H_
