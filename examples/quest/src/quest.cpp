#include "quest.h"
#include <attdet/attdet.h>
#include <iomanip>
#include <iostream>

int main() {
	using namespace attdet;
	Sensor sun_sensor({ 0.925417, -0.163176, -0.342020 }, { 1., 0., 0. }, .5);
	Sensor acc_sensor({ -0.37852, -0.440970, -0.813798 }, { 0., 0., -1. }, .5);

	const auto attitude = quest({ sun_sensor, acc_sensor });

	const auto angles = Quat2Euler(attitude);

	std::cout << std::fixed << std::setprecision(8);
	std::cout << "q:\t" << attitude << '\n';
	std::cout << std::fixed << std::setprecision(4);
	std::cout << "Angles:\t" << angles << '\n';
}
