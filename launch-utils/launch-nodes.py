from subprocess import Popen, PIPE, STDOUT

Popen(['roslaunch', 'augv_main', 'on-robot.launch'], stdout=PIPE, stderr=STDOUT,  close_fds=False)
