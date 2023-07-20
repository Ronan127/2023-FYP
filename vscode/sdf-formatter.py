import re

sdfile = open("./my-robot/robot.sdf", "r")
data=sdfile.read()
data = data.replace(".stl", ".obj")
data = re.sub("((<frame\sname=).*(<\/frame>\n))|(( frame=)[^>]*)", "", data)

sdfile = open("./my-robot/robot.sdf", "w")
sdfile.write(data)
