sudo perf record -g ./attdet-benchmark
sudo perf script > out.perf
 /home/leocelente/cloned/FlameGraph/stackcollapse-perf.pl out.perf > out.folded
 /home/leocelente/cloned/FlameGraph/flamegraph.pl out.folded > flgraph.svg
