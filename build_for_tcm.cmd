cd ..
SET var=%cd%
for %%I in (.) do set ProjDirName=%%~nxI
cd ..							
cd gophercan-lib/gophercannon
python gcan_auto_gen.py networks/go4-22c.yaml
cd ../..
cd Gopher_Sense
python sensor_cannon.py %var%/%ProjDirName%_config.yaml