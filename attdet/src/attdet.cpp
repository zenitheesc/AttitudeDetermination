// The MIT License (MIT)

// Copyright (c) 2021 Grupo Zenith Aerospace

//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
//  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.
/**
 * @file attdet.cpp
 * @author Leonardo Celente (@leocelente)
 * @brief Biblioteca de Determinação de Atitude
 * @version 0.1
 * @date Jan 2021
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "alglin/alglin.hpp"
#include <algorithm>
#include <attdet/attdet.h>
#include <numeric>

#define QUEST_ALT 0

namespace attdet {

Matrix3 block_matrix(const Vec3 &a, const Vec3 &b, const Vec3 &c) {
	Matrix3 out{};
	out[0] = static_cast<alglin::array<double, 3>>(a);
	out[1] = static_cast<alglin::array<double, 3>>(b);
	out[2] = static_cast<alglin::array<double, 3>>(c);
	return out;
}

/**
 * @brief QUEST algorithm implementation.
 * Computes the attitude given sensor body and inertial values.
 *
 * @param sensors List of Sensor() with at least 2 Sensors
 * @return Quat  Attitude as Unit Quaternion
 */
#if !QUEST_ALT
Quat quest(const std::initializer_list<Sensor> &sensors) {
	if (sensors.size() < 2) { return {}; }
	alglin::array<Quat, 4> candidateQuats{};
	alglin::array<double, 4> distanceToSing{};
	std::initializer_list<Rotations> rotations{
		Rotations::X, Rotations::Y, Rotations::Z,
         Rotations::None
	};

	double lambda{};
	Matrix3 B_{};
	for (const auto &sensor : sensors) {
		B_ =
		  B_
		  + (sensor.weight * alglin::outer(sensor.measure, sensor.reference));
		lambda += sensor.weight;
	}
	for (const auto rot : rotations) {
		Matrix3 B{ B_ };

		auto rotate = [&B](int n, int m) -> void {
			for (int i = 0; i < 3; ++i) {
				B[i][n] = -B[i][n];
				B[i][m] = -B[i][m];
			}
		};

		switch (rot) {
			case Rotations::X:
				rotate(1, 2);
				break;
			case Rotations::Y:
				rotate(0, 2);
				break;
			case Rotations::Z:
				rotate(0, 1);
				break;
			default:
				break;
		}

		const Matrix3 S = B + alglin::transpose(B);
		const auto sigma = alglin::trace(B);

		const Vec3 Z(
		  { (B[1][2] - B[2][1]), (B[2][0] - B[0][2]), (B[0][1] - B[1][0]) });

		const auto k = alglin::trace(alglin::fast_adjugate(S));
		const auto delta = alglin::det(S);
		const auto ZT = alglin::transpose(Z);
		const auto a = (sigma * sigma) - k;
		const auto b = sigma * sigma + (Z * ZT)[0][0];
		const auto c = delta + (Z * S * ZT)[0][0];
		const auto d = (Z * (S * S) * ZT)[0][0];

		auto f = [a, b, c, d, sigma](const double t) {
			return (((1 * t * t) - (a + b)) * t - c) * t
				   + (a * b + c * sigma - d);
		};
		auto df = [a, b, c](const double t) {
			return (4 * t * t - 2 * (a + b)) * t - c;
		};

		lambda -= f(lambda) / df(lambda);

		const auto identity = alglin::eye<double, 3>();
		const Matrix3 Y = ((lambda + sigma) * identity) - S;
		const Vec3 crp_ = alglin::transpose(alglin::inverse(Y) * ZT);
		const auto w = 1. / (std::sqrt((crp_ * alglin::transpose(crp_))[0][0]));
		const Quat q({ w * crp_[0], w * crp_[1], w * crp_[2], w });
		const auto dY = alglin::det(Y);

		switch (rot) {
			case Rotations::X:
				candidateQuats[0] = { q[3], -q[2], q[1], -q[0] };
				distanceToSing[0] = dY;
				break;
			case Rotations::Y:
				candidateQuats[1] = { q[2], q[3], -q[0], -q[1] };
				distanceToSing[1] = dY;
				break;
			case Rotations::Z:
				candidateQuats[2] = { -q[1], q[0], q[3], -q[2] };
				distanceToSing[2] = dY;
				break;
			default:
			case Rotations::None:
				candidateQuats[3] = { q[0], q[1], q[2], q[3] };
				distanceToSing[3] = dY;
				break;
		}
	}
	double d{};
	int idx{};
	for (int i = 0; i < 4; i++) {
		if (distanceToSing[i] > d) {
			d = distanceToSing[i];
			idx = i;
		}
	}
	auto selected = candidateQuats[idx];
	return alglin::normalize(selected);
}
#else
/**
 * @brief QUEST algorithm implementation.
 * Computes the attitude given sensor body and inertial values.
 * "original-like" implementation
 *
 * @param sensors List of Sensor()
 * @return Quat  Attitude as Unit Quaternion
 */
namespace {
	Quat quest_unsafe(const Matrix3 &B, double lambda) {
		const Matrix3 S = B + alglin::transpose(B);
		const auto sigma = alglin::trace(B);
		const Vec3 Z(
		  { (B[1][2] - B[2][1]), (B[2][0] - B[0][2]), (B[0][1] - B[1][0]) });

		const auto k = alglin::trace(alglin::fast_adjugate(S));
		const auto delta = alglin::det(S);
		const auto ZT = alglin::transpose(Z);
		const auto a = (sigma * sigma) - k;
		const auto b = sigma * sigma + (Z * ZT)[0][0];
		const auto c = delta + (Z * S * ZT)[0][0];
		const auto d = (Z * (S * S) * ZT)[0][0];

		auto f = [a, b, c, d, sigma](const double t) {
			return (((1 * t * t) - (a + b)) * t - c) * t
				   + (a * b + c * sigma - d);
		};
		auto df = [a, b, c](const double t) {
			return (4 * t * t - 2 * (a + b)) * t - c;
		};

		lambda -= f(lambda) / df(lambda);

		const auto identity = alglin::eye<double, 3>();
		const Matrix3 Y = ((lambda + sigma) * identity) - S;

		const Vec3 crp_ = alglin::transpose(alglin::inverse(Y) * ZT);

		const auto w = 1 / (std::sqrt((crp_ * alglin::transpose(crp_))[0][0]));
		const Quat q({ w * crp_[0], w * crp_[1], w * crp_[2], w });
		return alglin::normalize(q) * alglin::det(Y);
	}
}// namespace
Quat quest(const std::initializer_list<Sensor> &sensors) {

	const double lambda = std::accumulate(sensors.begin(),
	  sensors.end(),
	  0.f,
	  [](const double prev, const Sensor &s) { return prev + s.weight; });

	Matrix3 B{};
	std::for_each(sensors.begin(), sensors.end(), [&B](const Sensor& sensor) {
		B =
		  B + (sensor.weight * alglin::outer(sensor.measure, sensor.reference));
	});

	auto rotate = [](Matrix3 &M, int n, int m) -> void {
		for (int i = 0; i < 3; ++i) {
			M[i][n] = -M[i][n];
			M[i][m] = -M[i][m];
		}
	};

	alglin::array<Quat, 4> candidateQuats{};

	candidateQuats[0] = quest_unsafe(B, lambda);

	rotate(B, 1, 2);
	const auto qX = quest_unsafe(B, lambda);
	candidateQuats[1] = { qX[3], -qX[2], qX[1], -qX[0] };

	rotate(B, 0, 1);
	const auto qY = quest_unsafe(B, lambda);
	candidateQuats[2] = { qY[2], qY[3], -qY[0], -qY[1] };

	rotate(B, 1, 2);
	const auto qZ = quest_unsafe(B, lambda);
	candidateQuats[3] = { -qZ[1], qZ[0], qZ[3], -qZ[2] };

	const auto selected = [&candidateQuats]() -> Quat {
		const Quat x = *std::max_element(candidateQuats.begin(),
		  candidateQuats.end(),
		  [](const Quat &a, const Quat &b) { return a * a < b * b; });
		return alglin::normalize(x);
	}();

	return selected;
}

#endif

Matrix3 triad(const Sensor &sensor1, const Sensor &sensor2) {

	auto t_1b = sensor1.measure;
	auto t_2b = alglin::cross(sensor1.measure, sensor2.measure);
	auto t_3b = alglin::cross(t_1b, t_2b);
	auto t_1i = sensor1.reference;
	auto t_2i = alglin::cross(t_1i, sensor2.reference);
	auto t_3i = alglin::cross(t_1i, t_2i);
	auto BbarT = block_matrix(t_1b, t_2b, t_3b);
	auto NT = block_matrix(t_1i, t_2i, t_3i);
	auto DCM = BbarT * (alglin::transpose(NT));
	return DCM;
}

Vec3 DCM2Euler(const Matrix3 &A) {
	constexpr auto r = static_cast<double>(180.0 / 3.141592);
	const auto theta = std::asin(A[0][2]);
	const auto psi = std::acos(A[0][0] / std::cos(theta));
	const auto phi = std::asin(-A[1][2] / (std::cos(theta)));
	return { r * phi, r * theta, r * psi };
}

Vec3 Quat2Euler(const Quat &q) {
	constexpr auto r = static_cast<double>(180.0 / 3.141592);
	const auto phi = std::atan2(2.0f * (q[3] * q[0] - q[1] * q[2]),
	  1.f - 2.0f * (q[0] * q[0] + q[1] * q[1]));
	const auto tetha = std::asin(2.0f * (q[3] * q[1] + q[2] * q[0]));
	const auto psi = std::atan2(2.0f * (q[3] * q[2] - q[0] * q[1]),
	  1.f - 2.0f * (q[1] * q[1] + q[2] * q[2]));

	return { r * phi, r * tetha, r * psi };
}

}// namespace attdet
