#include "alglin/alglin.hpp"
#include "attdet/attdet.h"
#include <algorithm>
#include <benchmark/benchmark.h>
#include <random>
#include <vector>

static void BM_QUEST(benchmark::State &state) {
	// Perform setup here
	std::random_device rd;
	std::mt19937 g(rd());

	auto gen_sensor = [&g]() -> attdet::Sensor {
		using T = double;
		constexpr T tpi = 3.1415926535897932384;
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
		std::uniform_real_distribution<double> angles(-tpi, tpi);
		std::uniform_real_distribution<double> vecs(-1, 1);
		auto M = DCM(angles(g), angles(g), angles(g));
		Vec3 v({ vecs(g), vecs(g), vecs(g) });
		return { M * v, v, 0.5 };
	};
	constexpr auto shelf = 10000;
	std::vector<std::array<attdet::Sensor, 2>> sensors(shelf);
	auto gen = [&gen_sensor]() {
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
// Register the function as a benchmark
BENCHMARK(BM_QUEST);
// Run the benchmark
BENCHMARK_MAIN();
