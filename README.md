# icat single vehicle

1. Start laser and robot base  (on agent)

roslaunch icat_nav laser_bringup.launch namespace:=v1

2. Start edge map server (on server)

roslaunch ndt_localizer map_loader.launch

3. Start localization  (on agent)

roslaunch ndt_localizer ndt_localizer.launch namespace:=v1 

4. start visualization for mapping

rosrun rviz rviz -d /home/tby/icat/src/ndt_localizer/cfgs/rock-auto.rviz





Lower can be ignored for now.
--------------------------------
3. Start algorithm  (on agent)

 3.1  draw your own map and save your map file afterwards  (under icat_nav/maps folder) (Optional)

 roslaunch icat_nav icat_gmapping_ndtodom.launch

3.2 Start navigation  (on agent)

roslaunch icat_nav icat_navigation.launch use_rviz:=false map:=map_1

4. Visualize in rviz (on host)

4.1 start visualization for navigation
rosrun rviz rviz -d /home/tby/icat/src/icat_nav/rviz/navigate.rviz

4.2 start visualization for mapping
rosrun rviz rviz -d /home/tby/icat/src/ndt_localizer/cfgs/rock-auto.rviz




Source code originally developed by yahboomcar.
