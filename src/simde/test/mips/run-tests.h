#if defined(SIMDE_TESTS_MIPS_RUN_TESTS_H)
  #error File already included.
#endif
#define SIMDE_TESTS_MIPS_RUN_TESTS_H

#include "src/simde/test/test.h"
#include "src/simde/test/mips/msa/run-tests.h"

MunitSuite* simde_tests_mips_get_suite(void);
