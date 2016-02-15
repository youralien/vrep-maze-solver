# start vrep
# timerunmsecs=$(expr 1000 \* 120) 
# bash ~/apps/V-REP_PRO_EDU_V3_2_3_rev4_64_Linux/vrep.sh ~/Documents/studyaway/DistAutoRobo/LAB2/A0149643X_LAB2/Lab2_Environment_NoDoors.ttt -s$timerunmsecs -gREMOTEAPISERVERSERVICE_19999_FALSE_FALSE &

python Lab2Program.py &
bash ~/apps/V-REP_PRO_EDU_V3_2_3_rev4_64_Linux/vrep.sh ~/Documents/studyaway/DistAutoRobo/LAB2/A0149643X_LAB2/Lab2_Environment_NoDoors.ttt -gREMOTEAPISERVERSERVICE_19999_FALSE_FALSE 
