<h1 align="center" style="color:white; background-color:black">Attitude Determination</h1>
<h4 align="center">Library of Deterministic Methods of Attitude Determination</h4>

<p align="center">
	<a href="http://zenith.eesc.usp.br/">
    <img src="https://img.shields.io/badge/Zenith-Embarcados-black?style=for-the-badge"/>
    </a>
    <a href="https://eesc.usp.br/">
    <img src="https://img.shields.io/badge/Linked%20to-EESC--USP-black?style=for-the-badge"/>
    </a>
    <a href="https://github.com/zenitheesc/AttitudeDetermination/blob/main/LICENSE">
    <img src="https://img.shields.io/github/license/zenitheesc/AttitudeDetermination?style=for-the-badge"/>
    </a>
    <a href="https://github.com/zenitheesc/AttitudeDetermination/issues">
    <img src="https://img.shields.io/github/issues/zenitheesc/AttitudeDetermination?style=for-the-badge"/>
    </a>
    <a href="https://github.com/zenitheesc/AttitudeDetermination/commits/main">
    <img src="https://img.shields.io/github/commit-activity/m/zenitheesc/AttitudeDetermination?style=for-the-badge">
    </a>
    <a href="https://github.com/zenitheesc/AttitudeDetermination/graphs/contributors">
    <img src="https://img.shields.io/github/contributors/zenitheesc/AttitudeDetermination?style=for-the-badge"/>
    </a>
    <a href="https://github.com/zenitheesc/AttitudeDetermination/commits/main">
    <img src="https://img.shields.io/github/last-commit/zenitheesc/AttitudeDetermination?style=for-the-badge"/>
    </a>
    <a href="https://github.com/zenitheesc/AttitudeDetermination/issues">
    <img src="https://img.shields.io/github/issues-raw/zenitheesc/AttitudeDetermination?style=for-the-badge" />
    </a>
    <a href="https://github.com/zenitheesc/AttitudeDetermination/pulls">
    <img src = "https://img.shields.io/github/issues-pr-raw/zenitheesc/AttitudeDetermination?style=for-the-badge">
    </a>
</p>

<p align="center">
    <a href="#contents">Contents</a> •
    <a href="#environment-and-tools">Environment and Tools</a> •
    <a href="#steps-to-run-and-debug">Steps to run and debug</a> •
    <a href="#todo">TODO</a> •
    <a href="#how-to-contribute">How to contribute?</a> •
    <a href="#more">More</a> •
</p>

## Contents

- **attdet** - The main Attitude Determination Library.
- **attdet/alglin** - Internal Linear Algebra Library.
- **examples/quest** - QUaternion ESTimator algorithm demo.
- **examples/serial** - QUEST demo with serial port data.
- **misc** - Python implementation using Numpy

## Environment and tools

CMake. A C++11 Compiler (e.g. GCC 4.8).

## Steps to run and debug

Both libraries make use of ~~`std::array`~~ e `std::initializer_list`, so a C++11 compiler is necessary. To use `constexpr` modify `attdet/alglin/CMakeLists.txt` changing `gnu++11` to 17. Then use CMake:

```shell
mkdir build
cd build
cmake ..
cmake --build .. --config <CONFIG> --target <TARGET>
```

Where `CONFIG` can be `Debug`, `Release`, `MinSizeRel` and `TARGET` is A subproject: `attdet`, `quest`, `serial` or tests: `alglin-tests`, `attdet-tests`. The tests use the Catch2 library that is automatically fetched by CMake.

The `alglin` library is header-only so its not a direct target. But the `attdet` library is a static library.

## TODO:

- [x] Compile using STM32 Toolchain (impact\*: ~6KB)
- [ ] Use asserts in critical functions.
<!-- - [ ] STL Compatible Iterators
- [ ] `void_t` SFINAE Concepts ? -->

\*Compiling in Release mode, with C++14 manually copying files directly, not compiling and linking `libattdet.a`.

## How to contribute

1.  Fork.
2.  Branch.
3.  Commit.
4.  Pull Request.

## More:

Temos um texto sobre Determinação de Atitude e a história do algorítmo QUEST em nosso [Medium](https://zenith-eesc.medium.com/determina%C3%A7%C3%A3o-de-atitude-62d5e716631a)

---

<p align="center">
    <a href="http://zenith.eesc.usp.br">
    <img src="https://img.shields.io/badge/Check%20out-Zenith's Oficial Website-black?style=for-the-badge" />
    </a> 
    <a href="https://www.facebook.com/zenitheesc">
    <img src="https://img.shields.io/badge/Like%20us%20on-facebook-blue?style=for-the-badge"/>
    </a> 
    <a href="https://www.instagram.com/zenith_eesc/">
    <img src="https://img.shields.io/badge/Follow%20us%20on-Instagram-red?style=for-the-badge"/>
    </a>

</p>
<p align = "center">
<a href="zenith.eesc@gmail.com">zenith.eesc@gmail.com</a>
</p>
