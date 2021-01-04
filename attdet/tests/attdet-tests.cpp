#include <attdet.h>
#include <catch2/catch.hpp>

using namespace attdet;

TEST_CASE("QUEST") {
  Sensor sensor0({0.925417, -0.163176, -0.342020}, {1., 0., 0.}, .5);
  Sensor sensor1({-0.37852, -0.440970, -0.813798}, {0., 0., -1.}, .5);
  SECTION("Com dados normais") {
    Quat q = quest({sensor0, sensor1});
    const Vec3 a = Quat2Euler(q);

    // TODO: Melhorar testes com fp
    REQUIRE(abs(a[0] - 30.) < 1E-5);
    REQUIRE(abs(a[1] + 20.) < 1E-5);
    REQUIRE(abs(a[2] - 10.) < 1E-5);
  }
  SECTION("QUEST trivial") {
    sensor0.reference = {0.925417, -0.163176, -0.342020};
    sensor1.reference = {-0.378522, -0.440970, -0.813798};
    sensor0.measure = {0.925418, -0.163177, -0.34201};
    sensor1.measure = {-0.378521, -0.44096, -0.813797};
    Quat q = quest({sensor0, sensor1});

    REQUIRE((q[0] - 0) < 1E-5);
    REQUIRE((q[1] - 0) < 1E-5);
    REQUIRE((q[2] - 0) < 1E-5);
    REQUIRE((q[3] - 1) < 1E-5);
  }
  SECTION("QUEST em Singularidades") {

    sensor0.reference = {1., 1E-13, 0.};
    sensor1.reference = {1E-13, 0., -1.};

    sensor0.measure = {1., 1E-13, 0.};
    sensor1.measure = {1E-13, 0., 1.};
    Quat qX = quest({sensor0, sensor1});

    REQUIRE(qX == Quat{1., 0., 0., 0.});

    sensor0.measure = {-1., 1E-10, 0.};
    sensor1.measure = {1E-10, 0., 1.};
    Quat qY = quest({sensor0, sensor1});

    REQUIRE(qY == Quat{0., 1., 0., 0.});

    sensor0.measure = {-1., 1E-10, 0.};
    sensor1.measure = {1E-10, 0., -1.};
    Quat qZ = quest({sensor0, sensor1});

    REQUIRE(qZ == Quat{0., 0., 1., 0.});
  }
  SECTION("QUEST trivial") {
    sensor0.reference = {0.925417, -0.163176, -0.342020};
    sensor1.reference = {-0.378522, -0.440970, -0.813798};
    sensor0.measure = {0.925418, -0.163177, -0.34201};
    sensor1.measure = {-0.378521, -0.44096, -0.813797};
    Quat q = quest({sensor0, sensor1});

    REQUIRE((q[0] - 0) < 1E-5);
    REQUIRE((q[1] - 0) < 1E-5);
    REQUIRE((q[2] - 0) < 1E-5);
    REQUIRE((q[3] - 1) < 1E-5);
  }
}

TEST_CASE("Block Matrix Construction") {
  Vec3 a({1., 3., 4.});
  Vec3 b({0., 0., 0.});
  Vec3 c = 2. * a;
  Matrix3 A = block_matrix(a, b, c);
  Matrix3 expect({{1., 3., 4.}, {0., 0., 0.}, {2., 6., 8.}});
  REQUIRE(A == expect);
}

TEST_CASE("TRIAD") {
  Sensor sensor0({0.925417, -0.163176, -0.342020}, {1., 0., 0.}, .5);
  Sensor sensor1({-0.37852, -0.440970, -0.813798}, {0., 0., -1.}, .5);
  sensor0.measure = sensor0.reference;
  sensor1.measure =  sensor1.reference;
  Matrix3 A = triad({sensor0, sensor1});
  REQUIRE(A == alglin::eye<double, 3>());
}
