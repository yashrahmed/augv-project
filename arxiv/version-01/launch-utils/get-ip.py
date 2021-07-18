import subprocess as sp

cmd_output = sp.check_output(['hostname', '-I']).decode().split(' ')[0]
print(cmd_output)