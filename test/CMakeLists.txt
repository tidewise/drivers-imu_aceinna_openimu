rock_gtest(test_suite suite.cpp
   test_Protocol.cpp test_Driver.cpp test_Configuration.cpp test_PeriodicUpdate.cpp
   DEPS imu_aceinna_openimu)

rock_gtest(test_NMEAPublisher suite.cpp
   test_NMEAPublisher.cpp ../src/NMEAPublisher.cpp
   DEPS imu_aceinna_openimu)
