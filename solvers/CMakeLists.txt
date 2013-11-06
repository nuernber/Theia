#See CMakeLists.txt in base dir for macro definition

#Random Sampler
HEADER_LIBRARY(random_sampler random_sampler.h util/random)

#Ransac
HEADER_LIBRARY(ransac ransac.h solvers/random_sampler)

#MLESAC
HEADER_LIBRARY(mlesac mlesac.h solvers/random_sampler)

#PROSAC
HEADER_LIBRARY(prosac prosac.h solvers/random_sampler)

#ARRSAC
HEADER_LIBRARY(arrsac arrsac.h math/probability/sequential_probability_ratio)

#RECON
#Currently unsupported !
#HEADER_LIBRARY(recon recon.h math probability kolmogorov_smirnoff)

#Tests:
IF(${BUILD_TESTING})
  GTEST(arrsac_test solvers/arrsac util/random)
  GTEST(mlesac_test solvers/mlesac util/random)
  GTEST(prosac_test solvers/prosac util/random)
  GTEST(ransac_test solvers/ransac util/random)
  #Currently unsupported !
  #GTEST(recon_test solvers recon)
ENDIF(${BUILD_TESTING})
