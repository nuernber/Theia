# See CMakeLists.txt in base dir for macro definition

# ARRSAC
HEADER_LIBRARY(arrsac arrsac.h 
  math/probability/sequential_probability_ratio)

# RECON
# Currently unsupported!
#HEADER_LIBRARY(recon recon.h math/probability/kolmogorov_smirnoff)

# Tests:
IF (${BUILD_TESTING})
  GTEST(arrsac_test 
    solvers/arrsac
    util/random)
  GTEST(mlesac_test)
  GTEST(prosac_test)
  GTEST(ransac_test 
    util/random)
  # Currently unsupported!
  #GTEST(recon_test solvers/recon)
ENDIF (${BUILD_TESTING})
