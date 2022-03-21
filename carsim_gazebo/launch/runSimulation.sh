
source ~/aev/simulation_ws/devel/setup.bash
cd ~/aev/simulation_ws/catkin_ws
catkin_make

gnome-terminal --title="Simulation" -- bash -c '
gnome-terminal --tab --title="TTC" -- bash -c "roslaunch carsim_gazebo carsimTTC.launch; exec bash";
sleep 5;

gnome-terminal --tab --title="Obj" -- bash -c "roslaunch carsim_gazebo carsimObj.launch; exec bash";
sleep 3;

gnome-terminal --tab --title="CtrTTC" -- bash -c "rosrun carsim_gazebo teleopTTC.py; exec bash";
sleep 2;

gnome-terminal --title="CtrObj" -- bash -c "rosrun carsim_gazebo teleopObj.py; exec bash";

exec bash'




