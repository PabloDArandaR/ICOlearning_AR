g++ bin/main.cpp ../motor_control/motor_class.cpp  -lmatrix_creator_hal -o run
g++ bin/stop.cpp ../motor_control/motor_class.cpp  -lmatrix_creator_hal -o stop

echo "To execute the program, execute the following command: "
echo ""
echo "         ./run"