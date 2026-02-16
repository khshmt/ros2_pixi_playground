include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Generate.cmake)

idlc_generate(TARGET Image FILES "Image.idl" WARNINGS no-implicit-extensibility)

add_executable(framePublisher framePublisher.cpp)
target_link_libraries(framePublisher Image CycloneDDS::ddsc ${OpenCV_LIBS} rerun_sdk)

add_executable(frameSubscriber frameSubscriber.cpp)
target_link_libraries(frameSubscriber Image CycloneDDS::ddsc ${OpenCV_LIBS} rerun_sdk)