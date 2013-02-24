# See CMakeLists.txt in base dir for macro definition

# ARRSAC
# NOTE: arrsac.cc is a dummy file to make this compile!
CC_LIBRARY(arrsac arrsac.cc math/probability/sequential_probability_ratio)

# RECON
# NOTE: recon.cc is a dummy file to make this compile!
CC_LIBRARY(recon recon.cc math/probability/kolmogorov_smirnoff)

# Tests:
IF (${BUILD_TESTING})
  GTEST(arrsac_test solvers/arrsac)
  GTEST(mlesac_test)
  GTEST(prosac_test)
  GTEST(ransac_test)
  GTEST(recon_test solvers/recon)
ENDIF (${BUILD_TESTING})
