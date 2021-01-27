# Attitude Determination
![Manual workflow](https://github.com/zenitheesc/AttitudeDetermination/workflows/Manual%20workflow/badge.svg)
## Conteúdo:
 - **attdet** - Biblioteca para Determinação de Atitude usando métodos determinísticos. 
 - **attdet/alglin** - Biblioteca de Álgebra Linear.
 - **examples/quest** - Demo do Algoritmo QUaternion ESTimator da biblioteca.
 - **examples/serial** - Demo do QUEST com captura de dados via serial.
 - **misc** - Implementação em Python

## Compilação:
Ambas bibliotecas fazem uso de `std::array` e `std::initializer_list`, logo um compilador com C++11 é necessário. Caso queira `constexpr` modifique o `attdet/alglin/CMakeLists.txt` e troque o `cxx_std_11` para 17. Então use o CMake:

```shell
mkdir build
cd build
cmake --build .. --config <CONFIG> --target <TARGET>
```

Onde a `CONFIG` pode ser `Debug`, `Release`, `MinSizeRel` e `TARGET` é o subprojeto, `attdet`, `quest`, `serial` ou os testes `alglin-tests` e `attdet-tests`. Os testes utilizam a biblioteca Catch2 que é baixada automaticamente pelo CMake. 

A biblioteca `alglin` é header-only e por isso não é um target direto, diferente da `attdet` que compila como biblioteca estática.

## TODO:
- [x] Compilar usando toolchain do STM32 (impacto*: ~6KB)
- [ ] Explicitar contratos de funções críticas

*Compilando em Release, com C++14 copiando diretamente os arquivos para o projeto, sem compilar a `libattdet.a` separadamente.  

## Saiba Mais:
Temos um texto sobre Determinação de Atitude e a história do algoritmo QUEST em nosso [Medium](https://zenith-eesc.medium.com/determina%C3%A7%C3%A3o-de-atitude-62d5e716631a)
