
@echo off

set argC=0
for %%x in (%*) do Set /A argC+=1

IF NOT "%argC%" == "2" (
	echo "Please supply two arguments: a Drake release (drake-YYYYMMDD), and a relative path to your preferred notebook directory"
	exit "1"
) ELSE (
	SET _INTERPOLATION_0=
  FOR /f "delims=" %%a in ('pwd') DO (SET "_INTERPOLATION_0=!_INTERPOLATION_0! %%a")
  echo "-e" "[32m" "Using" "!_INTERPOLATION_0!%~2" "as" "your" "notebook" "root" "directory." "[0m"
  docker "pull" "mit6881/drake-course:%~1"
  SET _INTERPOLATION_1=
  FOR /f "delims=" %%a in ('pwd') DO (SET "_INTERPOLATION_1=!_INTERPOLATION_1! %%a")
  docker "run" "-it" "-p" "8080:8080" "-p" "7000-7010:7000-7010" "--rm" "-v" "!_INTERPOLATION_1!/%~2"":"\psets "mit6881/drake-course:%~1" "\bin\bash" "-c" "cd /psets && jupyter notebook --ip 0.0.0.0 --port 8080 --allow-root --no-browser"
)