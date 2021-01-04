#include <alglin.hpp>

#include <catch2/catch.hpp>
TEST_CASE("A * A^-1  = I") {

  Matrix3 A({{1, 2, 3}, {3, 2, 1}, {3, 1, 3}});
  auto I = alglin::eye<double, 3>();
  REQUIRE(A * alglin::inverse(A) == I);
}

TEST_CASE("transpose of vector") {

  Vec3 v({1, 2, 3});
  alglin::GenericMatrix<double, 3, 1> T({{1}, {2}, {3}});
  REQUIRE(alglin::transpose(v) == T);

  auto X = v * alglin::transpose(v);
  REQUIRE(X[0][0] == 14.0);
}

TEST_CASE("Matrix Adjugate") {

  Matrix3 A({{1, 2, 3}, {3, 2, 1}, {1, 1, 1}});
  Matrix3 B({{1, -2, 1}, {1, -2, 1}, {-4, 8, -4}});
  REQUIRE(alglin::adjugate(A) == B);
}

TEST_CASE("Matrix Inverse ") {

  Matrix3 A({{1, 2, 3}, {4, 2, 1}, {5, 4, 2}});
  Matrix3 B({{0, 8, -4}, {-3, -13, 11}, {6, 6, -6}});
  B = (1. / 12) * B;
  REQUIRE(alglin::inverse(A) == B);

  alglin::SquareMatrix<double, 2> C({{1, 2}, {3, 4}});
  alglin::SquareMatrix<double, 2> D({{4, -2}, {-3, 1}});
  D = (1. / (-2)) * D;
  REQUIRE(alglin::inverse(C) == D);
}

TEST_CASE("Matrix Multiplication") {

  Matrix3 A({{1, 2, 3}, {10, 2024, 17}, {9, 8, 7}});
  Matrix3 B({{9, 5, 1}, {14, 254, 11}, {8, 8, 4}});
  Matrix3 C({{61, 537, 35}, {28562, 514282, 22342}, {249, 2133, 125}});
  REQUIRE((A * B) == C);
}

TEST_CASE("Matrix Trace") {

  Matrix3 A({{1, 2, 3}, {10, 2024, 17}, {9, 8, 7}});
  double w{2032};
  REQUIRE(alglin::trace(A) == w);
}

TEST_CASE("Matrix Determinant") {

  Matrix3 A({{1, 2, 3}, {10, 2024, 17}, {9, 8, 7}});
  double w{-40210};
  REQUIRE(alglin::det(A) == w);
}

TEST_CASE("Vector Cross Product") {

  Vec3 u({1.4, 0., 1.});
  Vec3 v({0.333, 2., 7.});
  Vec3 w({-2, -9.467, 2.8});
  REQUIRE((alglin::cross(u, v)) == w);
}

TEST_CASE("Vector Outer Product") {

  Vec3 u({1.4, 0., 1.});
  Vec3 v({0.333, 2., 7.});
  Matrix3 A({{0.4662, 2.8, 9.8}, {0, 0, 0}, {0.333, 2, 7}});
  REQUIRE(alglin::outer(u, v) == A);
}

TEST_CASE("Vector Inner Product") {

  Vec3 u({1., 0., 1.});
  Vec3 v({2., 3., 7.});
  REQUIRE((u * v) == 9.0);
}

TEST_CASE("Matrix Vector Product") {

  Matrix3 A({{1, 2, 3}, {3, 2, 1}, {5, 4, 2}});
  Vec3 v({3., 6., 7.});
  Vec3 w({36., 28., 53});
  REQUIRE((A * v) == w);
}

TEST_CASE("Vector Normalized") {

  Vec3 u({1.4, 0., 1.});
  Vec3 w({0.813733471206735, 0., 0.581238193719096});
  REQUIRE(alglin::normalize(u) == w);
}

TEST_CASE("Adjugate of uninvertable matrix") {

  Matrix3 A({{1, 0, 0}, {0, 0, 0}, {0, 0, -1}});
  Matrix3 B({{0, 0, 0}, {0, -1, 0}, {0, 0, 0}});
  REQUIRE(alglin::adjugate(A) == B);
}

TEST_CASE("Matrix Sum") {

  Matrix3 A({{1, 0, 5}, {0, 2, 0}, {3, 0, -1}});
  Matrix3 B({{1, 0, 5}, {0, 2, 0}, {3, 0, -1}});

  REQUIRE((A + B) == 2. * B);
}

TEST_CASE("Ax a xB") {

  Matrix3 A({{1, 2, 3}, {1, 2, 3}, {1, 2, 3}});
  Matrix3 B({{2, 3, 4}, {2, 3, 4}, {2, 3, 4}});
  REQUIRE(A * (2.) * B == (2. * A) * B);
  REQUIRE(A * (2.) * B == A * (2. * B));
}

TEST_CASE("Vector from InitList of lvalue") {

  double q[4] = {1, 1, 1, 1};
  Quat v = {q[0], q[1], q[2], q[3]};
  Quat w({1, 1, 1, 1});
  REQUIRE(v == w);
}



