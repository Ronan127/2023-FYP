import regex as re
import argparse
import pathlib

parser = argparse.ArgumentParser("format sdformat for drake")
parser.add_argument("root", type=str, help="The root directory that contains sdf file")
parser.add_argument("--wheel", type=str, help="The coefficient of friction on the wheels")
parser.add_argument("--tail", type=str, help="The coeficient of friction on the tail")
parser.add_argument("--frame", type=str, help="The coeficient of friction on the frame")
args = parser.parse_args()


for file in pathlib.Path(args.root).glob('*.sdf'):
    
    sdfile = open(file, "r")
    data=sdfile.read()
    #add friction to wheel
    data = re.sub("(?<=<collision\sname=.wheel_shaft_.*_wheel_visual.>\n.*\n.*\n.*\n.*\n)<\/collision>", "<surface><friction><ode><mu>{0}</mu><mu2>{0}</mu2></ode></friction></surface></collision>".format(args.wheel), data)
    #add friction to tail
    data = re.sub("(?<=<collision\sname=.body_.*_tail_visual.>\n.*\n.*\n.*\n.*\n)<\/collision>", "<surface><friction><ode><mu>{0}</mu><mu2>{0}</mu2></ode></friction></surface></collision>".format(args.tail), data)
    #add friction to LIM frame
    data = re.sub("(?<=<collision\sname=.lim_frame_.*_frame_visual.>\n.*\n.*\n.*\n.*\n)<\/collision>", "<surface><friction><ode><mu>{0}</mu><mu2>{0}</mu2></ode></friction></surface></collision>".format(args.frame), data)

    print(f"Formatting {file}")
    sdfile = open(file, "w")
    sdfile.write(data)
