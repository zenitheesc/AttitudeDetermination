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

#include <alglin/array.hpp>// alglin::array
#include <cmath>
#include <cstring>
#include <initializer_list>
#include <ostream>

#if !defined(ALGLIN_PRECISION)
// Used in 'operator==' in floating-point comparison
#define ALGLIN_PRECISION (1E-14)
#endif// ALGLIN_PRECISION

#define CONSTEXPR_17
#define NODISCARD
#define MAYBE_UNUSED
// Is constexpr if C++17 or higher
#if __cplusplus >= 201703L
#undef CONSTEXPR_17
#define CONSTEXPR_17 constexpr
#if __cpp_attributes && __has_cpp_attribute(nodiscard)
#undef NODISCARD
#define NODISCARD [[nodiscard]]
#undef MAYBE_UNUSED
#define MAYBE_UNUSED [[maybe_unused]]
#endif
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
#if USE_FAST_INVSQRT
namespace {

template<class U, class T> U bit_cast(T t) {
	static_assert(sizeof(T) == sizeof(U), "Otherwise is undefined behavior");
	static_assert(std::is_trivially_copyable<T>::value, "Needed for memcpy ");
	static_assert(std::is_trivially_copyable<U>::value, "Needed for memcpy");
	U u{};
	std::memcpy(&u, &t, sizeof(T));
	return u;
}

inline float fast_invsqrt(float x) {
	int i = bit_cast<int>(x);
	i = 0x5f3759df - (i >> 1);
	float y = bit_cast<float>(i);
	y = y * (1.5f - 0.5f * x * y * y);
	y = y * (1.5f - 0.5f * x * y * y);
	return y;
}

}// namespace
#endif
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
template<typename T, int N, int M> struct GenericMatrix {
  protected:
	alglin::array<alglin::array<T, M>, N> elements{};

  public:
	// Somente usada nos testes
	CONSTEXPR_17
	GenericMatrix(std::initializer_list<std::initializer_list<T>> l) {
		auto row_it = this->elements.begin();
		for (auto &&i : l) {
			auto col_it = (*row_it).begin();
			for (auto &&j : i) { *col_it++ = j; }
			row_it++;
		}
	}

	constexpr GenericMatrix() = default;

	CONSTEXPR_17 GenericMatrix<T, N, M> operator+(
	  const GenericMatrix<T, N, M> &rhs) const {
		GenericMatrix<T, N, M> out{};
		for (int i = 0; i < N; ++i) {
			for (int j = 0; j < M; j++) {
				out[i][j] = elements[i][j] + rhs[i][j];
			}
		}
		return out;
	}
	constexpr GenericMatrix<T, N, M> operator-(
	  const GenericMatrix<T, N, M> &rhs) const {
		return (*this + (static_cast<T>(-1) * rhs));
	}
	constexpr alglin::array<T, M> operator[](int i) const {
		return elements[i];
	}
	CONSTEXPR_17 alglin::array<T, M> &operator[](int i) { return elements[i]; }
	constexpr alglin::array<alglin::array<T, M>, N> data() const {
		return elements;
	}
};

/***
 * GenericMatrix operators
 ***/
template<class T, int N, int M>
CONSTEXPR_17 bool operator==(
  const GenericMatrix<T, N, M> &A, const GenericMatrix<T, N, M> &B) {
	for (int row = 0; row < N; row++) {
		for (int col = 0; col < M; col++) {
			if (std::abs(A[row][col] - B[row][col])
				> static_cast<T>(ALGLIN_PRECISION)) {
				return false;
			}
		}
	}
	return true;
}

template<class T, int N, int M>
CONSTEXPR_17 GenericMatrix<T, N, M> operator*(
  const GenericMatrix<T, N, M> &A, const T a) {
	GenericMatrix<T, N, M> out{};
	const T _a = static_cast<T>(a);
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < M; j++) { out[i][j] = _a * A[i][j]; }
	}
	return out;
}

template<class T, int N, int M>
constexpr GenericMatrix<T, N, M> operator*(
  T a, const GenericMatrix<T, N, M> &A) {
	return A * a;
}

template<class T, int N, int M, int R>
CONSTEXPR_17 GenericMatrix<T, N, M> operator*(
  const GenericMatrix<T, N, R> &A, const GenericMatrix<T, R, M> &B) noexcept {
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
template<class T>
constexpr GenericMatrix<T, 3, 3> operator*(
  const GenericMatrix<T, 3, 3> &A, const GenericMatrix<T, 3, 3> &B) noexcept {
	return { { A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0],
			   A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1],
			   A[0][0] * B[0][2] + A[0][1] * B[1][2] + A[0][2] * B[2][2] },
		{ A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0],
		  A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1],
		  A[1][0] * B[0][2] + A[1][1] * B[1][2] + A[1][2] * B[2][2] },
		{ A[2][0] * B[0][0] + A[2][1] * B[1][0] + A[2][2] * B[2][0],
		  A[2][0] * B[0][1] + A[2][1] * B[1][1] + A[2][2] * B[2][1],
		  A[2][0] * B[0][2] + A[2][1] * B[1][2] + A[2][2] * B[2][2] } };
}

template<class T, int N, int M>
std::ostream &operator<<(std::ostream &sout, const GenericMatrix<T, N, M> &p) {
	for (int i = 0; i < N; ++i) {
		for (int j = 0; j < M; j++) { sout << p[i][j] << '\t'; }
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
template<class T, int N, int M>
NODISCARD CONSTEXPR_17 GenericMatrix<T, M, N> transpose(
  const GenericMatrix<T, N, M> &A) noexcept {
	GenericMatrix<T, M, N> out{};
	for (int i = 0; i < N; ++i) {
		for (int j = 0; j < M; ++j) { out[j][i] = A[i][j]; }
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
template<class T, int N, int M>
NODISCARD CONSTEXPR_17 T trace(const GenericMatrix<T, N, M> &A) {
	T sum = static_cast<T>(0);
	for (int i = 0; i < M; ++i) { sum = sum + A[i][i]; }
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

template<class T, int N> using SquareMatrix = GenericMatrix<T, N, N>;

/**
 * @brief Rejeita o calculo de determinantes de matrizes
 * maiores que 3x3. Não compila.
 *
 * @param M Matrix
 * @return CONSTEXPR_17 T determinante
 */
template<class T, int N>
NODISCARD CONSTEXPR_17 T det(MAYBE_UNUSED const SquareMatrix<T, N> &) {
	static_assert(N < 4, "Matrix must be at most 3x3");
}

/**
 * @brief Calcula o determinante de uma Matriz 2x2
 *
 * @param A Matrix
 * @return CONSTEXPR_17 T determinante
 */
template<class T> NODISCARD constexpr T det(const SquareMatrix<T, 2> &A) {
	return (A[0][0] * A[1][1]) - (A[0][1] * A[1][0]);
}
/**
 * @brief Calcula o determinante de uma Matriz 3x3
 *
 * @param A Matrix
 * @return CONSTEXPR_17 T determinante
 */
template<class T> NODISCARD constexpr T det(const SquareMatrix<T, 3> &A) {
	return (A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
			+ A[0][1] * (A[1][2] * A[2][0] - A[1][0] * A[2][2])
			+ A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]));
}

// - 48 bytes
template<class T, int N>
NODISCARD CONSTEXPR_17 SquareMatrix<T, N> transpose(
  const SquareMatrix<T, N> &A) noexcept {
	auto out = A;
	for (int i = 0; i < N; ++i) {
		for (int j = 0; j < i; ++j) { std::swap(out[i][j], out[j][i]); }
	}
	return out;
}

/**
 * @brief Rejeita o calculo de inversos de matrizes
 * maiores que 3x3. Não compila.
 *
 * @param M Matrix
 * @return SquareMatrix<T,N> Matrix Inversa
 */
template<class T, int N>
NODISCARD CONSTEXPR_17 SquareMatrix<T, N> inverse(
  MAYBE_UNUSED const SquareMatrix<T, N> &) {
	static_assert(N < 4, "Matrix must be at most 3x3");
}

/**
 * @brief Define o calculo do inverso de uma Matriz 2x2
 *
 * @param M Matrix
 * @return SquareMatrix<T,2> Matrix Inversa
 */
template<class T>
NODISCARD CONSTEXPR_17 SquareMatrix<T, 2> inverse(
  const SquareMatrix<T, 2> &M) noexcept {
	const auto d = det(M);
	if (d == static_cast<T>(0)) { return {}; }
	auto A = M;
	const auto a = 1. / d;
	using std::swap;
	swap(A[0][0], A[1][1]);
	A[1][0] *= -1;
	A[0][1] *= -1;
	return A * a;
}

/**
 * @brief Rejeita calculo da matriz adjunta de matrizes
 * maiores que 3x3. Não compila.
 *
 * @return CONSTEXPR_17 SquareMatrix<T, N> Matriz Adjunta
 */
template<class T, int N>
NODISCARD CONSTEXPR_17 SquareMatrix<T, N> adjugate(
  MAYBE_UNUSED const SquareMatrix<T, N> &) {
	static_assert(N == 3, "Matrix must be 3x3");
}

/**
 * @brief Calcula a Matriz Adjunta de M (3x3)
 *
 * @param M Matriz
 * @return CONSTEXPR_17 SquareMatrix<T, 3> Matriz Adjunta
 */
template<class T>
NODISCARD CONSTEXPR_17 SquareMatrix<T, 3> adjugate(
  const SquareMatrix<T, 3> &M) noexcept {
	SquareMatrix<T, 2> m{};
	auto cofactor = [&M, &m](const int i, const int j) -> T {
		int u{};
		int v{};
		for (int r = 0; r < 3; ++r) {
			if (i == r) { continue; }
			for (int c = 0; c < 3; ++c) {
				if (j == c) { continue; }
				m[u][v++] = M[r][c];
			}
			++u;
			v = 0;
		}

		if ((i + j) % 2)
			return -det(m);
		else
			return det(m);
	};
	SquareMatrix<T, 3> out{};
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) { out[i][j] = cofactor(i, j); }
	}
	return out;
}

template<class T>
constexpr SquareMatrix<T, 3> cofactor(SquareMatrix<T, 3> const &M) {
	return { { M[1][1] * M[2][2] - M[2][1] * M[1][2],
			   M[2][1] * M[0][2] - M[0][1] * M[2][2],
			   M[0][1] * M[1][2] - M[1][1] * M[0][2] },
		{ M[1][2] * M[2][0] - M[2][2] * M[1][0],
		  M[2][2] * M[0][0] - M[0][2] * M[2][0],
		  M[0][2] * M[1][0] - M[1][2] * M[0][0] },
		{ M[1][0] * M[2][1] - M[2][0] * M[1][1],
		  M[2][0] * M[0][1] - M[0][0] * M[2][1],
		  M[0][0] * M[1][1] - M[1][0] * M[0][1] } };
}
template<class T>
constexpr SquareMatrix<T, 3> fast_adjugate(SquareMatrix<T, 3> const &M) {
	return transpose(cofactor(M));
}


/**
 * @brief Calcula o inverso de uma Matriz 3x3
 *
 * @param M Matrix
 * @return SquareMatrix<T,2> Matrix Inversa
 */
template<class T>
NODISCARD CONSTEXPR_17 SquareMatrix<T, 3> inverse(
  const SquareMatrix<T, 3> &M) noexcept {
	const auto d = det(M);
	if (d == static_cast<T>(0)) { return {}; }
	const auto a = static_cast<T>(1.) / d;
	return transpose(fast_adjugate(M)) * a;
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
template<typename T, int N> struct Vector : public GenericMatrix<T, 1, N> {
	using base = GenericMatrix<T, 1, N>;
	constexpr Vector(const GenericMatrix<T, 1, N> &m) noexcept : base(m) {}

	constexpr Vector(std::initializer_list<T> l) noexcept : base({ l }) {}

	constexpr operator alglin::array<T, N>() const { return this->elements[0]; }

	constexpr Vector() = default;
	constexpr T operator[](int i) const { return this->elements[0][i]; }
	T &operator[](int i) { return this->elements[0][i]; }
};

/**
 * @brief Produto Vetorial de u x v
 *
 * @param u Vetor
 * @param v Vetor
 * @return CONSTEXPR_17 Vector<T, 3> u x v
 */
template<class T>
NODISCARD constexpr Vector<T, 3> cross(
  const Vector<T, 3> &u, const Vector<T, 3> &v) {
	return { { (-u[2] * v[1] + u[1] * v[2]) },
		{ (u[2] * v[0] - u[0] * v[2]) },
		{ (-u[1] * v[0] + u[0] * v[1]) } };
}

/**
 * @brief Produto Tensorial. Equivalente a um produto matricial Nx1 * 1xN
 *
 * @param u Vetor
 * @param v Vetor
 * @return CONSTEXPR_17 SquareMatrix<T, N> Matriz NxN
 */
template<class T, int N>
NODISCARD CONSTEXPR_17 SquareMatrix<T, N> outer(
  const Vector<T, N> &u, const Vector<T, N> &v) {
	SquareMatrix<T, N> out{};
	for (int i = 0; i < N; ++i) {
		for (int j = 0; j < N; ++j) { out[i][j] = u[i] * v[j]; }
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
template<class T, int N>
NODISCARD CONSTEXPR_17 T operator*(
  const Vector<T, N> &lhs, const Vector<T, N> &rhs) {
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
template<class T, int N>
NODISCARD Vector<T, N> normalize(const Vector<T, N> &v) {
#if USE_FAST_INVSQRT
	const auto n = fast_invsqrt(v * v);
#else
	const double n = 1. / std::sqrt(v * v);
#endif
	auto out = v;
	for (int i = 0; i < N; ++i) { out[i] = v[i] * n; }
	return out;
}

template<class T, int N>
NODISCARD constexpr Vector<T, N> operator*(
  const SquareMatrix<T, N> &lhs, const Vector<T, N> &rhs) {
	return transpose(lhs * transpose(rhs));
}

/**
 * @brief Retorna Matriz Identidade NxN
 *
 * @tparam N linhas = colunas
 * @return CONSTEXPR_17 SquareMatrix<T, N> Matrix Identidade
 */
template<class T, int N> NODISCARD CONSTEXPR_17 SquareMatrix<T, N> eye() {
	SquareMatrix<T, N> out{};
	for (int i = 0; i < N; ++i) { out[i][i] = 1; }
	return out;
}

}// namespace alglin

/***
 * Helpers para tipos comuns
 ***/
// Vetor R3
using Vec3 = alglin::Vector<double, 3>;
// Quaternion
using Quat = alglin::Vector<double, 4>;
// Matrix 3x3
using Matrix3 = alglin::SquareMatrix<double, 3>;
#undef CONSTEXPR_17
#undef NODISCARD
#undef MAYBE_UNUSED
#endif
