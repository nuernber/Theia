IF (${BUILD_TESTING})
  ADD_SUBDIRECTORY(gtest)
ENDIF (${BUILD_TESTING})

# Add Agast keypoint detector
ADD_SUBDIRECTORY(agast)

# Add VLFeat
ADD_SUBDIRECTORY(vlfeat)
