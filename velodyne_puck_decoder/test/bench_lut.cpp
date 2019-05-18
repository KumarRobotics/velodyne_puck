#include <benchmark/benchmark.h>

template <int N = 1e5>
struct TrigLookupTable {
  static constexpr
  double sin_table[N];
  double cos_table[N];

  TrigLookupTable() {
    for (int i = 0; i < N; ++i) {
    }
  }
};

static void BM_TrigLookupTable(benchmark::State& state) {
  // Perform setup here
  const TrigLookupTable lut;

  for (auto _ : state) {
    // This code gets timed
    SomeFunction();
  }
}
// Register the function as a benchmark
BENCHMARK(BM_TrigLookupTable);
// Run the benchmark
BENCHMARK_MAIN();
