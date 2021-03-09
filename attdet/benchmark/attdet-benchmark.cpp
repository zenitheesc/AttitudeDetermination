#include "alglin/alglin.hpp"
#include "attdet/attdet.h"
#include <algorithm>
#include <array>
#include <benchmark/benchmark.h>
#include <random>
#include <vector>

namespace {
attdet::Sensor gen_sensor() {
	using T = double;
	constexpr T tpi = 3.1415926535897932384 + 1E-4;
	auto DCM = [](T phi, T theta, T psi) -> Matrix3 {
		auto s = [](T x) -> T { return std::sin(x); };
		auto c = [](T x) -> T { return std::cos(x); };
		return { { c(theta) * c(psi),
				   s(phi) * s(theta) * c(psi) - c(phi) * s(psi),
				   s(phi) * s(psi) + c(phi) * s(theta) * c(psi) },
			{ c(theta) * s(psi),
			  s(phi) * s(theta) * s(psi) - c(phi) * c(psi),
			  c(phi) * s(theta) * s(psi) - s(phi) * c(psi) },
			{ -s(theta), s(phi) * c(theta), c(phi) * c(theta) } };
	};
	std::random_device rd;
	std::mt19937 g(rd());
	std::uniform_real_distribution<double> angles(-tpi, tpi);
	std::uniform_real_distribution<double> vecs(-1, 1);
	auto M = DCM(angles(g), angles(g), angles(g));
	Vec3 v({ vecs(g), vecs(g), vecs(g) });
	v = alglin::normalize(v);
	return { M * v, v, 0.5 };
}
};// namespace

static void BM_QUEST(benchmark::State &state) {
	constexpr auto shelf = 10000;
	std::vector<std::array<attdet::Sensor, 2>> sensors(shelf);
	auto gen = []() {
		return std::array<attdet::Sensor, 2>{ gen_sensor(), gen_sensor() };
	};
	sensors.reserve(shelf);
	std::generate(sensors.begin(), sensors.end(), gen);
	Quat q;
	benchmark::DoNotOptimize(q);
	for (auto _ : state) {
		q = attdet::quest({ sensors[state.iterations() % shelf][0],
		  sensors[state.iterations() % shelf][1] });
	}
}
BENCHMARK(BM_QUEST);


static void BM_TRIAD(benchmark::State &state) {
	constexpr auto shelf = 10000;
	std::vector<std::array<attdet::Sensor, 2>> sensors(shelf);
	auto gen = []() {
		return std::array<attdet::Sensor, 2>{ gen_sensor(), gen_sensor() };
	};
	sensors.reserve(shelf);
	std::generate(sensors.begin(), sensors.end(), gen);
	Matrix3 DCM;
	benchmark::DoNotOptimize(DCM);
	for (auto _ : state) {
		DCM = attdet::triad(sensors[state.iterations() % shelf][0],
		  sensors[state.iterations() % shelf][1]);
	}
}
BENCHMARK(BM_TRIAD);

// Run the benchmark
BENCHMARK_MAIN();
