#!/bin/zsh

# Check if an argument is provided
if [ $# -eq 0 ]; then
    arg=5
else
    arg="$1"
fi

current_date_hour=$(date '+%Y-%m-%d %H:%M:%S')
echo "Run time: $current_date_hour"
echo "Total simulations: $arg"
for ((i=1; i<=$arg; i++)); do
    echo "-----------------------------------"
    echo "Simulation $i of $arg"
    webots --no-rendering --mode=realtime --stdout --stderr --batch --minimize worlds/topic4.wbt &
    # webots --mode=realtime --stdout --stderr --batch worlds/topic4.wbt &
    sleep 200
    pkill webots
    sleep 10
done

echo "-----------------------------------"
echo "-----------------------------------"
