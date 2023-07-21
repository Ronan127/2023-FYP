import re
import argparse
import pathlib

parser = argparse.ArgumentParser("format sdformat for drake")
parser.add_argument("root", type=str, help="The root directory that contains sdf file")
args = parser.parse_args()

for file in pathlib.Path(args.root).glob('*.sdf'):
    print(f"Formatting {file}")
    sdfile = open(file, "r")
    data=sdfile.read()
    #replace references to stl files with references to obj files
    data = data.replace(".stl", ".obj")
    #remove frames (drake doesn't like them)
    data = re.sub("((<frame\sname=).*(<\/frame>\n))|(( frame=)[^>]*)", "", data)
    #fix file referencing
    data = re.sub("((file://)(?!{})|(file://{}))".format(args.root, args.root), "", data)

    sdfile = open(file, "w")
    sdfile.write(data)
