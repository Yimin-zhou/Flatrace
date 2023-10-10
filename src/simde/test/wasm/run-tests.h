#if defined(SIMDE_TESTS_WASM_RUN_TESTS_H)
  #error File already included.
#endif
#define SIMDE_TESTS_WASM_RUN_TESTS_H

#include "src/simde/test/test.h"
#include "src/simde/test/wasm/simd128/run-tests.h"
#include "src/simde/test/wasm/relaxed-simd/run-tests.h"

MunitSuite* simde_tests_wasm_get_suite(void);
