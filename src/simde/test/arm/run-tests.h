#if defined(SIMDE_TESTS_ARM_RUN_TESTS_H)
  #error File already included.
#endif
#define SIMDE_TESTS_ARM_RUN_TESTS_H

#include "src/simde/test/test.h"
#include "src/simde/test/arm/neon/run-tests.h"
#include "src/simde/test/arm/sve/run-tests.h"

MunitSuite* simde_tests_arm_get_suite(void);
