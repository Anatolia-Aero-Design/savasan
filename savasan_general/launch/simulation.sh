#open SITL
gnome-terminal --tab --title="Ardupilot 1" --command="bash -c 'cd ~; sim_vehicle.py -v ArduPlane -L ADANA -f gazebo-zephyr --console  --map --out=127.0.0.1:14550 -I0 --sysid=1; $SHELL'" 

#Start sim.launch file
gnome-terminal --tab --title="sim.launch" --command="bash -c 'cd ~;roslaunch savasan_general sim.launch; $SHELL'"
