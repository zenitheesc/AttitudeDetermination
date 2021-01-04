#include <iostream>
#include <algorithm>
#include <numeric>
#include <alglin.hpp>
/**
 * @brief QUEST algorithm implementation.
 * Computes the attitude given sensor body and inertial values.
 *
 * @param sensors List of Sensor()
 * @return Quat  Attitude as Unit Quaternion
 */
Quat quest(const Matrix3 &B, float lambda) {
    const Matrix3 S = B + alglin::transpose(B);
    const auto sigma = alglin::trace(B);
    const Vec3 Z(
        {(B[1][2] - B[2][1]), (B[2][0] - B[0][2]), (B[0][1] - B[1][0])});

    const auto k = alglin::trace(alglin::adjugate(S));
    const auto delta = alglin::det(S);
    const auto ZT = alglin::transpose(Z);
    const auto a = (sigma * sigma) - k;
    const auto b = sigma * sigma + (Z * ZT)[0][0];
    const auto c = delta + (Z * S * ZT)[0][0];
    const auto d = (Z * (S * S) * ZT)[0][0];

    auto f = [a, b, c, d, sigma](const auto t) {
      return (((1 * t * t) - (a + b)) * t - c) * t + (a * b + c * sigma - d);
    };
    auto df = [a, b, c](const auto t) {
      return (4 * t * t - 2 * (a + b)) * t - c;
    };

    lambda -= f(lambda) / df(lambda);

    constexpr auto identity = alglin::eye<float, 3>();
    const Matrix3 Y = ((lambda + sigma) * identity) - S;

    const Vec3 crp_ = alglin::transpose(alglin::inverse(Y) * ZT);

    const auto w = 1 / (std::sqrt((crp_ * alglin::transpose(crp_))[0][0]));
    const Quat q({w * crp_[0], w * crp_[1], w * crp_[2], w});
    const auto dY = alglin::det(Y);
  return (alglin::normalize(q) * dY);
}

int main(int argc, char const *argv[]) {
  Sensor sun_sensor({0.925417, -0.163176, -0.342020}, {1., 0., 0.}, .5);
  Sensor acc_sensor({-0.378522, -0.440970, -0.813798}, {0., 0., -1.}, .5);
    
  Matrix3 B_{};
  float lambda{};
  for (const auto &sensor : {sun_sensor, acc_sensor}) {
    B_ = B_ + (sensor.weight * alglin::outer(sensor.measure, sensor.reference));
    lambda += sensor.weight;
  }
  std::array<Quat, 4> candidateQuats{};
   candidateQuats[0] = quest(B_, lambda);

   auto BX = B_;
   for (size_t i = 0; i < 3; ++i) {
        BX[i][1] = -B_[i][1];
        BX[i][2] = -B_[i][2];
    }
  
  const auto qX = quest(BX, lambda);
  candidateQuats[1] = {qX[3], -qX[2], qX[1], -qX[0]};

   auto BY = B_;
   for (size_t i = 0; i < 3; ++i) {
        BY[i][0] = -B_[i][0];
        BY[i][2] = -B_[i][2];
    }
  const auto qY = quest(BY, lambda);
      candidateQuats[2] = {qY[2], qY[3], -qY[0], -qY[1]};

   auto BZ = B_;
   for (size_t i = 0; i < 3; ++i) {
        BZ[i][0] = -B_[i][0];
        BZ[i][1] = -B_[i][1];
    }
  const auto qZ = quest(BZ, lambda);
  candidateQuats[3] = {-qY[1], qY[0], qY[3], -qY[2]};



  const auto selected = [&candidateQuats]() -> Quat {
    const auto qmax = std::max_element(candidateQuats.begin(), candidateQuats.end(), [](const auto& a, const auto& b){
        return a*a > b*b;
    });
    return alglin::normalize(*qmax);
  }();
    
  auto euler = [](const Quat& q) {
    const auto r = 180.0f / 3.141592f;
    auto phi = r * std::atan2(2.0f * (q[3] * q[0] - q[1] * q[2]),
                              1 - 2.0f * (q[0] * q[0] + q[1] * q[1]));
    auto tetha = r * std::asin(2.0f * (q[3] * q[1] + q[2] * q[0]));
    auto psi = r * std::atan2(2.0f * (q[3] * q[2] - q[0] * q[1]),
                              1 - 2.0f * (q[1] * q[1] + q[2] * q[2]));

    return std::move(Vec3({phi, tetha, psi}));
  };
  const auto angles = euler(selected);

  // std::cout << std::fixed << std::setprecision(8);
//   std::cout << "q:\t" << selected << '\n';
  // std::cout << std::fixed << std::setprecision(3);
  std::cout << "Angles:\t" << angles << '\n';
return angles[0] + angles[1] + angles[2];


}
