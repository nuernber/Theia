# See CMakeLists.txt in base dir for macro definition

# Kolmogorov Smirnoff Test
CC_LIBRARY(kolmogorov_smirnoff kolmogorov_smirnoff.cc)

# SPRT Test
CC_LIBRARY(sequential_probability_ratio sequential_probability_ratio.cc)

# Tests:
IF (${BUILD_TESTING})
  # Comment this out because it is unsupported right now!
  #GTEST(kolmogorov_smirnoff_test math/probability/kolmogorov_smirnoff)
  GTEST(sprt_test 
    math/probability/sequential_probability_ratio
    util/random)
ENDIF (${BUILD_TESTING})
