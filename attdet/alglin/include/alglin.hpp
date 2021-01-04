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

#ifndef ALGLIN_H_
#define ALGLIN_H_
#include <array>
#include <cmath>
// Convenience
#include <initializer_list>
#include <ostream>

#if !defined(ALGLIN_PRECISION)
// Used in 'operator==' in floating-point comparison
#define ALGLIN_PRECISION (1E-14)
#endif // ALGLIN_PRECISION

// Is constexpr if C++17 or higher
#if __cplusplus >= 201703L
#define CONSTEXPR_17 constexpr
#else
#define CONSTEXPR_17
#endif
/***
  **     ___    __      __    _         __
  **    /   |  / /___ _/ /   (_)___    / /_  ____  ____
  **   / /| | / / __ `/ /   / / __ \  / __ \/ __ \/ __ \
  **  / ___ |/ / /_/ / /___/ / / / / / / / / /_/ / /_/ /
  ** /_/  |_/_/\__, /_____/_/_/ /_(_)_/ /_/ .___/ .___/
  **          /____/                     /_/   /_/
//#                                  by Zenith Aerospace
***/
/***
 * @file alglin.hpp
 * @brief Biblioteca de Algebra Linear
 * Organização: Zenith Aerospace @zenitheesc
 * @author Autor: Leonardo Celente (@leocelente)
 * @date Jan 2021
 * @version 0.1
 *?
 *? Description:
 *?  Biblioteca para auxiliar na implementação de algoritmos
 *?  deterministicos de Determinação de Atitude. Em especial
 *?  o algoritmo QUEST de Malcolm D. Shuster.
 *?
 *? Testing:
 *?  Testes da biblioteca devem estar no target 'alglin-tests' no CMake
 *
 *  Compilação:
 *   C++17 é necessário para 'constepxr operator[]' no array 'elements', logo
 *    a biblioteca só será constexpr (*) .
 *   Se no futuro mudar o container de std::array checar move optimizations
 *
 *  std::abs
 *  std::sqrt
 *  std::swap
 *  std::array
 *
 *  std::initializer_list
 *  std::ostream
 ***/

namespace alglin {

/**
  ____                      _      __  __       _        _
 / ___| ___ _ __   ___ _ __(_) ___|  \/  | __ _| |_ _ __(_)_  __
| |  _ / _ \ '_ \ / _ \ '__| |/ __| |\/| |/ _` | __| '__| \ \/ /
| |_| |  __/ | | |  __/ |  | | (__| |  | | (_| | |_| |  | |>  <
 \____|\___|_| |_|\___|_|  |_|\___|_|  |_|\__,_|\__|_|  |_/_/\_\
 **/
/**
 * @brief GenericMatrix representa uma Matrix de N linhas por M colunas.
 * Base para as classes SquareMatrix e Vector
 */
template <typename T, int N, int M> struct GenericMatrix {
protected:
  std::array<std::array<T, M>, N> elements{};

public:
  // Somente usada nos testes
  CONSTEXPR_17
  GenericMatrix(std::initializer_list<std::initializer_list<T>> &&l) {
    auto row_it = this->elements.begin();
    for (auto &&i : l) {
      auto col_it = (*row_it).begin();
      for (auto &&j : i) {
        *col_it++ = j;
      }
      row_it++;
    }
  }

  CONSTEXPR_17 GenericMatrix() = default;

  CONSTEXPR_17 GenericMatrix<T, N, M>
  operator+(const GenericMatrix<T, N, M> &rhs) const {
    GenericMatrix<T, N, M> out{};
    for (int i = 0; i < N; ++i) {
      for (int j = 0; j < M; j++) {
        out[i][j] = elements[i][j] + rhs[i][j];
      }
    }
    return out;
  }
  CONSTEXPR_17 GenericMatrix<T, N, M>
  operator-(const GenericMatrix<T, N, M> &rhs) const {
    return (*this + (static_cast<T>(-1) * rhs));
  }
  CONSTEXPR_17 std::array<T, M> operator[](int i) const { return elements[i]; }
  CONSTEXPR_17 std::array<T, M> &operator[](int i) { return elements[i]; }
  CONSTEXPR_17 std::array<std::array<T, M>, N> data() const { return elements; }
};

/***
 * GenericMatrix operators
 ***/
template <class T, int N, int M>
CONSTEXPR_17 bool operator==(const GenericMatrix<T, N, M> &A,
                             const GenericMatrix<T, N, M> &B) {
  for (int row = 0; row < N; row++) {
    for (int col = 0; col < M; col++) {
      if (std::abs(A[row][col] - B[row][col]) >
          static_cast<T>(ALGLIN_PRECISION)) {
        return false;
      }
    }
  }
  return true;
}

template <class T, int N, int M>
CONSTEXPR_17 GenericMatrix<T, N, M> operator*(const GenericMatrix<T, N, M> &A,
                                              const T a) {
  GenericMatrix<T, N, M> out{};
  const T _a = static_cast<T>(a);
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < M; j++) {
      out[i][j] = _a * A[i][j];
    }
  }
  return out;
}

template <class T, int N, int M>
CONSTEXPR_17 GenericMatrix<T, N, M> operator*(T a,
                                              const GenericMatrix<T, N, M> &A) {
  return A * a;
}

template <class T, int N, int M, int R>
CONSTEXPR_17 GenericMatrix<T, N, M> operator*(const GenericMatrix<T, N, R> &A,
                                              const GenericMatrix<T, R, M> &B) {
  GenericMatrix<T, N, M> out{};
  for (int row = 0; row < N; row++) {
    for (int col = 0; col < M; col++) {
      for (int inn = 0; inn < R; inn++) {
        out[row][col] += A[row][inn] * B[inn][col];
      }
    }
  }
  return out;
}

template <class T, int N, int M>
std::ostream &operator<<(std::ostream &sout, const GenericMatrix<T, N, M> &p) {
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < M; j++) {
      sout << p[i][j] << '\t';
    }
    sout << '\n';
  }
  return sout;
}

/**
 * @brief Calcula o transposto de A
 *
 * @param A Matrix NxM
 * @return CONSTEXPR_17 GenericMatrix<T, M, N>
 */
template <class T, int N, int M>
[[nodiscard]] CONSTEXPR_17 GenericMatrix<T, M, N>
transpose(const GenericMatrix<T, N, M> &A) {
  GenericMatrix<T, M, N> out{};
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < M; ++j) {
      out[j][i] = A[i][j];
    }
  }
  return out;
}

/**
 * @brief Calcula o Traço de A
 *
 * @tparam N linhas
 * @tparam M colunas
 * @param A Matrix
 * @return CONSTEXPR_17 T traço
 */
template <class T, int N, int M>
[[nodiscard]] CONSTEXPR_17 T trace(const GenericMatrix<T, N, M> &A) {
  T sum = 0;
  for (int i = 0; i < M; ++i) {
    sum = sum + A[i][i];
  }
  return sum;
}

/**
 ____                             __  __       _        _
/ ___|  __ _ _   _  __ _ _ __ ___|  \/  | __ _| |_ _ __(_)_  __
\___ \ / _` | | | |/ _` | '__/ _ \ |\/| |/ _` | __| '__| \ \/ /
 ___) | (_| | |_| | (_| | | |  __/ |  | | (_| | |_| |  | |>  <
|____/ \__, |\__,_|\__,_|_|  \___|_|  |_|\__,_|\__|_|  |_/_/\_\
          |_|
**/
/**
 * @brief SquareMatrix representa uma Matriz Quadrada NxN
 *
 */
template <class T, int N> struct SquareMatrix : public GenericMatrix<T, N, N> {

  CONSTEXPR_17 SquareMatrix(const GenericMatrix<T, N, N> &other) noexcept {
    this->elements = other.data();
  }
  CONSTEXPR_17 SquareMatrix() = default;
};

/**
 * @brief Rejeita o calculo de determinantes de matrizes
 * maiores que 3x3. Não compila.
 *
 * @param M Matrix
 * @return CONSTEXPR_17 T determinante
 */
template <class T, int N>
[[nodiscard]] CONSTEXPR_17 T det([[maybe_unused]] const SquareMatrix<T, N> &A) {
  static_assert(N < 4, "Matrix must be at most 3x3");
}

/**
 * @brief Calcula o determinante de uma Matriz 2x2
 *
 * @param A Matrix
 * @return CONSTEXPR_17 T determinante
 */
template <class T>
[[nodiscard]] CONSTEXPR_17 T det(const SquareMatrix<T, 2> &A) {
  return (A[0][0] * A[1][1]) - (A[0][1] * A[1][0]);
}
/**
 * @brief Calcula o determinante de uma Matriz 3x3
 *
 * @param A Matrix
 * @return CONSTEXPR_17 T determinante
 */
template <class T>
[[nodiscard]] CONSTEXPR_17 T det(const SquareMatrix<T, 3> &A) {
  return (A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) +
          A[0][1] * (A[1][2] * A[2][0] - A[1][0] * A[2][2]) +
          A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]));
}

/**
 * @brief Rejeita o calculo de inversos de matrizes
 * maiores que 3x3. Não compila.
 *
 * @param M Matrix
 * @return SquareMatrix<T,N> Matrix Inversa
 */
template <class T, int N>
[[nodiscard]] CONSTEXPR_17 SquareMatrix<T, N>
inverse([[maybe_unused]] const SquareMatrix<T, N> &M) {
  static_assert(N < 4, "Matrix must be at most 3x3");
}

/**
 * @brief Define o calculo do inverso de uma Matriz 2x2
 *
 * @param M Matrix
 * @return SquareMatrix<T,2> Matrix Inversa
 */
template <class T>
[[nodiscard]] CONSTEXPR_17 SquareMatrix<T, 2>
inverse(const SquareMatrix<T, 2> &M) {
  const auto d = det(M);
  auto A = M;
  if (d) {
    const auto a = 1 / d;
    std::swap(A[0][0], A[1][1]);
    A[1][0] *= -1;
    A[0][1] *= -1;
    A = A * a;
  }
  return A;
}

/**
 * @brief Rejeita calculo da matriz adjunta de matrizes
 * maiores que 3x3. Não compila.
 *
 * @return CONSTEXPR_17 SquareMatrix<T, N> Matriz Adjunta
 */
template <class T, int N>
[[nodiscard]] CONSTEXPR_17 SquareMatrix<T, N>
adjugate([[maybe_unused]] const SquareMatrix<T, N> &M) {
  static_assert(N == 3, "Matrix must be 3x3");
}

/**
 * @brief Calcula a Matriz Adjunta de M (3x3)
 *
 * @param M Matriz
 * @return CONSTEXPR_17 SquareMatrix<T, 3> Matriz Adjunta
 */
template <class T>
[[nodiscard]] CONSTEXPR_17 SquareMatrix<T, 3>
adjugate(const SquareMatrix<T, 3> &M) {
  auto cofactor = [&M](const int i, const int j) -> T {
    SquareMatrix<T, 2> m{};
    int u{};
    int v{};
    for (int r = 0; r < 3; ++r) {
      if (i == r) {
        continue;
      }
      for (int c = 0; c < 3; ++c) {
        if (j == c) {
          continue;
        }
        m[u][v++] = M[r][c];
      }
      ++u;
      v = 0;
    }
    return (1 + ((i + j) % 2 != 0) * (-2)) * det(m);
  };
  auto out = M;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out[i][j] = cofactor(i, j);
    }
  }
  return out;
}

/**
 * @brief Calcula o inverso de uma Matriz 3x3
 *
 * @param M Matrix
 * @return SquareMatrix<T,2> Matrix Inversa
 */
template <class T>
[[nodiscard]] CONSTEXPR_17 SquareMatrix<T, 3>
inverse(const SquareMatrix<T, 3> &M) {
  const auto d = det(M);
  auto A = M;
  if (d) {
    const auto a = 1 / d;
    A = transpose(adjugate(M)) * a;
  }
  return A;
}

/**
__     __        _
\ \   / /__  ___| |_ ___  _ __
 \ \ / / _ \/ __| __/ _ \| '__|
  \ V /  __/ (__| || (_) | |
   \_/ \___|\___|\__\___/|_|
**/
/***
 * Vetores são definidos como Matriz com uma linha e N colunas.
 **/
template <typename T, int N> struct Vector : public GenericMatrix<T, 1, N> {

  CONSTEXPR_17 Vector(const GenericMatrix<T, 1, N> &other) noexcept {
    this->elements = other.data();
  }

  CONSTEXPR_17 Vector(const std::initializer_list<T> &&l) noexcept {
    auto it = this->elements[0].begin();
    for (auto &&i : l) {
      *it++ = i;
    }
  }

  CONSTEXPR_17 operator std::array<T, N>() const { return this->elements[0]; }

  CONSTEXPR_17 Vector() = default;
  CONSTEXPR_17 T operator[](int i) const { return this->elements[0][i]; }
  CONSTEXPR_17 T &operator[](int i) { return this->elements[0][i]; }
};

/**
 * @brief Produto Vetorial de u x v
 *
 * @param u Vetor
 * @param v Vetor
 * @return CONSTEXPR_17 Vector<T, 3> u x v
 */
template <class T>
[[nodiscard]] CONSTEXPR_17 Vector<T, 3> cross(const Vector<T, 3> &u,
                                              const Vector<T, 3> &v) {
  Vector<T, 3> out({{(-u[2] * v[1] + u[1] * v[2])},
                    {(u[2] * v[0] - u[0] * v[2])},
                    {(-u[1] * v[0] + u[0] * v[1])}});
  return out;
}

/**
 * @brief Produto Tensorial. Equivalente a um produto matricial Nx1 * 1xN
 *
 * @param u Vetor
 * @param v Vetor
 * @return CONSTEXPR_17 SquareMatrix<T, N> Matriz NxN
 */
template <class T, int N>
[[nodiscard]] CONSTEXPR_17 SquareMatrix<T, N> outer(const Vector<T, N> &u,
                                                    const Vector<T, N> &v) {
  SquareMatrix<T, N> out{};
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) {
      out[i][j] = u[i] * v[j];
    }
  }
  return out;
}
/**
 * @brief Produto Interno de dois vetores de tamanho N
 *
 * @param lhs Vetor
 * @param rhs Vetor
 * @return CONSTEXPR_17 T Produto Interno
 */
template <class T, int N>
[[nodiscard]] CONSTEXPR_17 T operator*(const Vector<T, N> &lhs,
                                       const Vector<T, N> &rhs) {
  double sum{};
  for (int i = 0; i < N; ++i) {
    sum = sum + static_cast<double>(lhs[i] * rhs[i]);
  }
  return static_cast<T>(sum);
  // return std::inner_product(lhs[0].begin(), lhs[0].end(),
  //                           rhs[0].begin(), static_cast<T>(0));
}

/**
 * @brief Normaliza o vetor v
 *
 * @tparam N colunas
 * @param v vetor
 * @return const Vector<T, N> Vetor Unitario
 */
template <class T, int N>
[[nodiscard]] Vector<T, N> normalize(const Vector<T, N> &v) {

  const double n = 1 / std::sqrt(v * v);
  auto out = v;
  for (int i = 0; i < N; ++i) {
    out[i] = v[i] * n;
  }
  return out;
}

template <class T, int N>
[[nodiscard]] CONSTEXPR_17 Vector<T, N> operator*(const SquareMatrix<T, N> &lhs,
                                                  const Vector<T, N> &rhs) {
  const auto vT = transpose(rhs);
  return transpose(lhs * vT);
}

/**
 * @brief Retorna Matriz Identidade NxN
 *
 * @tparam N linhas = colunas
 * @return CONSTEXPR_17 SquareMatrix<T, N> Matrix Identidade
 */
template <class T, int N> [[nodiscard]] CONSTEXPR_17 SquareMatrix<T, N> eye() {
  SquareMatrix<T, N> out{};
  for (int i = 0; i < N; ++i) {
    out[i][i] = 1;
  }
  return out;
}

} // namespace alglin

/***
 * Helpers para tipos comuns
 ***/
// Vetor R3
using Vec3 = alglin::Vector<double, 3>;
// Quaternion
using Quat = alglin::Vector<double, 4>;
// Matrix 3x3
using Matrix3 = alglin::SquareMatrix<double, 3>;

#endif
