# See CMakeLists.txt in base dir for macro definition

# Anderson Darling Test
CC_LIBRARY(anderson_darling anderson_darling.cc)

# Kolmogorov Smirnoff Test
CC_LIBRARY(kolmogorov_smirnoff kolmogorov_smirnoff.cc)

# SPRT Test
CC_LIBRARY(sequential_probability_ratio sequential_probability_ratio.cc)

# Tests:
IF (${BUILD_TESTING})
  GTEST(anderson_darling_test math/probability/anderson_darling)
  GTEST(kolmogorov_smirnoff_test math/probability/kolmogorov_smirnoff)
  GTEST(sprt_test math/probability/sequential_probability_ratio)
ENDIF (${BUILD_TESTING})
