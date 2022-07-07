if(ARGOS_BUILD_FOR_SIMULATOR)
	# Set up ARGoS compilation information
	include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
	link_directories(${ARGOS_LIBRARY_DIRS})

	# Headers
	set(HEADERS
		RVRDAO.h
	# 	RabMessageBuffer.h
	  	ReferenceModel1Dot1.h
	#   ReferenceModel1Dot2.h
	#   ReferenceModel2Dot0.h
	#   ReferenceModel2Dot1.h
    #   ReferenceModel2Dot2.h
    #   ReferenceModel2Dot3.h
	)

	# Sources
	set(SOURCES
		RVRDAO.cpp
	# 	RabMessageBuffer.cpp
	  	ReferenceModel1Dot1.cpp
	#   ReferenceModel1Dot2.cpp
	#   ReferenceModel2Dot0.cpp
	#   ReferenceModel2Dot1.cpp
    #   ReferenceModel2Dot2.cpp
    #   ReferenceModel2Dot3.cpp
	)
else()
endif(ARGOS_BUILD_FOR_SIMULATOR)


add_library(argos3_demiurge_rvr_dao SHARED ${HEADERS} ${SOURCES} )
target_link_libraries(argos3_demiurge_rvr_dao argos3plugin_${ARGOS_BUILD_FOR}_rvr)

install(
  FILES ${HEADERS}
  DESTINATION include/argos3/demiurge/rvr-dao)

# Install location for the rvr plugin
install(TARGETS argos3_demiurge_rvr_dao
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)