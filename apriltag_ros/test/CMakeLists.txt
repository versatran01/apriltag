#if(CATKIN_ENABLE_TESTING)
#    catkin_add_gtest(gtest_${PROJECT_NAME}
#        gtest_${PROJECT_NAME}.cpp
#        test_apriltag_detector.cpp
#        )
#    target_link_libraries(
#        gtest_${PROJECT_NAME}
#        ${catkin_LIBRARIES}
#        ${PROJECT_NAME}
#        )
#endif()
