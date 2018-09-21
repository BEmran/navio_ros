CXX = g++
CFLAGS = -std=c++11
INC=-I "../include" -I"../include/lib" -I"../include/lib/Navio" -I"../include/testbed_navio" -I"../include/lib/Navio/Navio2"
default: main
main: 
	$(CXX) $(CFLAGS) motor_calibration.cpp $(INC) -o motor_calibration ../include/testbed_navio/navio_interface.cpp ../include/lib/Navio/Navio2/PWM.cpp ../include/lib/Navio/Common/Util.cpp -Llibnavio

clean:
	rm -r *.o
