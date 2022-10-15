import subprocess

subprocess.call(["../bin/Debug/rocket.exe","../test/test_rocket.srocket", "output.txt"])


#subprocess.Popen(["../bin/Debug/rocket.exe","../test/test_rocket.srocket", "output.txt"],stdout=subprocess.PIPE).communicate()
