# Setup
## Install Webots
* [Link](https://cyberbotics.com/doc/guide/installation-procedure)
  * "Installing the Debian Package with the Advanced Packaging Tool (APT)"
## Install webots_ros package
* Assuming that catkin workspace is already created and configured
* Better to build from source 

```bash
cd /path/to/catkin_ws/src

# retrieve the sources
git clone https://github.com/cyberbotics/webots_ros.git

cd /path/to/catkin_ws

# checking dependencies
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro noetic

# building
catkin_make

# source the workspace 
source /path/to/catkin_ws/devel/setup.bash
```

## Suggestions
### Bashrc file
* Add the following lines to the **~/.bashrc** file 
```bash
sudo nano ~/.bashrc
```

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

alias bash_setup='source ~/.bashrc'
alias ws_setup='source ~/catkin_ws/devel/setup.bash'
alias ros_ws='cd ~/catkin_ws'

export WEBOTS_HOME=/usr/local/webots
```

* The **source** commands set the *ROS* environment and the *ROS* workspace

* The **aliases** are for commonly used commands, like reloading the terminal (*bash_setup*), reloading the catkin workspace (*ws_setup*) and jumping to the workspace directory (*ros_ws*)

* The final **export** sets up the environment variable used so *ROS* knows where *webots* is

### Webots startup
* Go to **/usr/local/bin/webots** and add a line that sources ROS on the application startup

```bash
sudo nano /usr/local/bin/webots
```
```bash
source /opt/ros/noetic/setup.bash ##### add this line

###### No changes from here #######
# prepare for handling program termination
kill_webots() {
  kill -TERM "${webots_pid}" &> /dev/null
}
handle_termination() {
  if [ "${webots_pid}" ]; then
    kill_webots
  else
    term_kill_needed="yes"
  fi
}
...
```
* This way the ROS environment will be automatically integrated with Webots. An alternative is to open a terminal, source ROS and then run the command "webots", which opens webots in a ROS sourced environment
