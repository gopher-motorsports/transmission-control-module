cd .. 							&:: Get out of the Debug folder
SET var=%cd%						&:: Store current path in var
for %%I in (.) do set ProjDirName=%%~nxI		&:: Get name of current directory
cd ..							
cd gophercan-lib/gophercannon
python gcan_auto_gen.py networks/go4-22c.yaml		&:: Run gcan_auto_gen with car configuration argument
cd ../..
cd Gopher_Sense
python sensor_cannon.py %var%/%ProjDirName%_config.yaml	&:: Run sensor_cannon with config.yaml from project directory