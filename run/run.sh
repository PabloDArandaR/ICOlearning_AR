# Run main file with the following structure
# main.out file_data cutoff sampling_time threshold speed

speed=30.0
cutoff=50
sampling_time=10
threshold=0

#cutoff = std::stof(argv[2]);
#sampling_time = std::stof(argv[3]);
#threshold = std::stof(argv[4]);
#speed[0] = std::stof(argv[5]);
#speed[1] = std::stof(argv[5]);

while true
do
    ./run $1 $cutoff $sampling_time $threshold $speed &
    echo "To STOP write anything."
    read $foo
    pkill run
    ./stop

    echo "Continue running? (y/n) "
    read ANSWER
    while [ "$ANSWER" != "y" ] && [ "$ANSWER" != "n" ]
    do
        echo "[ERROR] Repeat: Continue running? (y/n) "
        read ANSWER
    done

    if [ "$ANSWER" == "n" ]
    then
        break
    fi
done