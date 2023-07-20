import re

sdfile = open("./my-robot/robot.sdf", "r")
data=sdfile.read()
#replace references to stl files with references to obj files
data = data.replace(".stl", ".obj")
#remove frames (drake doesn't like them)
data = re.sub("((<frame\sname=).*(<\/frame>\n))|(( frame=)[^>]*)", "", data)
#fix file referencing
data = re.sub("(file://)(?!my-robot/)", "file://my-robot/", data)

sdfile = open("./my-robot/robot.sdf", "w")
sdfile.write(data)
