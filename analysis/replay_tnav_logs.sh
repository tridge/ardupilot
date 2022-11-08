./waf configure --board sitl
./waf replay

rm logs/*
./build/sitl/tool/Replay /mnt/hgfs/Downloads/flight_logs/NavImprovements/00000111.BIN
./build/sitl/tool/Replay /mnt/hgfs/Downloads/flight_logs/NavImprovements/00000114.BIN
./build/sitl/tool/Replay /mnt/hgfs/Downloads/flight_logs/NavImprovements/00000116.BIN
./build/sitl/tool/Replay /mnt/hgfs/Downloads/flight_logs/NavImprovements/00000119.BIN
./build/sitl/tool/Replay /mnt/hgfs/Downloads/flight_logs/NavImprovements/SV-142.bin
./build/sitl/tool/Replay /mnt/hgfs/Downloads/flight_logs/NavImprovements/SV-145.bin
./build/sitl/tool/Replay /mnt/hgfs/Downloads/flight_logs/NavImprovements/SV-Striver-166.bin
mkdir -p /mnt/hgfs/Downloads/flight_logs/NavImprovements/Striver-SpringValley
cp ./logs/* /mnt/hgfs/Downloads/flight_logs/NavImprovements/Striver-SpringValley
./analysis/graph_logs.py /mnt/hgfs/Downloads/flight_logs/NavImprovements/Striver-SpringValley/*.BIN

rm logs/*
./build/sitl/tool/Replay --parm EK3_FLOW_USE=0 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/00000133.BIN
./build/sitl/tool/Replay --parm EK3_FLOW_USE=0 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/00000135.BIN
./build/sitl/tool/Replay --parm EK3_FLOW_USE=0 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/00000137.BIN
./build/sitl/tool/Replay --parm EK3_FLOW_USE=0 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/00000140.BIN
mkdir -p /mnt/hgfs/Downloads/flight_logs/NavImprovements/Striver-Winslade
cp ./logs/* /mnt/hgfs/Downloads/flight_logs/NavImprovements/Striver-Winslade
./analysis/graph_logs.py /mnt/hgfs/Downloads/flight_logs/NavImprovements/Striver-Winslade/*.BIN

rm logs/*
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/SV-PPDS-143.BIN
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/SV-PPDS-145.BIN
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/SV-PPDS-147.BIN
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/SV-PPDS-149.BIN
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/SV-PPDS-163.BIN
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/SV-PPDS-164.BIN
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/SV-PPDS-173.BIN
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/SV-PPDS-178.BIN
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/SV-PPDS-181.bin
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/SV-PPDS-4.3-5.bin
mkdir -p /mnt/hgfs/Downloads/flight_logs/NavImprovements/PPDS-SpringValley
cp ./logs/* /mnt/hgfs/Downloads/flight_logs/NavImprovements/PPDS-SpringValley
./analysis/graph_logs.py /mnt/hgfs/Downloads/flight_logs/NavImprovements/PPDS-SpringValley/*.BIN

rm logs/*
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/050_log026.BIN
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/050_log027.BIN
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/050_log034.BIN
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/050_log035.BIN
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/050_log037.BIN
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/050_log065.BIN
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/050_log067.BIN
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/050_log068.BIN
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/050_log069.BIN
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/050_log085.BIN
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/050_log086.BIN
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/050_log089.BIN
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/050_log165.BIN
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 --parm EK3_TNAV_RNG_DD=5 /mnt/hgfs/Downloads/flight_logs/NavImprovements/050_log169.BIN
mkdir -p /mnt/hgfs/Downloads/flight_logs/NavImprovements/PPDS-Tuki
cp ./logs/* /mnt/hgfs/Downloads/flight_logs/NavImprovements/PPDS-Tuki
./analysis/graph_logs.py /mnt/hgfs/Downloads/flight_logs/NavImprovements/PPDS-Tuki/*.BIN

rm logs/*
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 /mnt/hgfs/Downloads/flight_logs/NavImprovements/log_8_2022-10-28-11-22-02.bin
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 /mnt/hgfs/Downloads/flight_logs/NavImprovements/log_13_2022-10-28-12-41-48.bin
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 /mnt/hgfs/Downloads/flight_logs/NavImprovements/log_15_2022-10-28-13-00-54.bin
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 /mnt/hgfs/Downloads/flight_logs/NavImprovements/log_16_2022-10-28-13-11-42.bin
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 /mnt/hgfs/Downloads/flight_logs/NavImprovements/log_17_2022-10-28-13-24-22.bin
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 /mnt/hgfs/Downloads/flight_logs/NavImprovements/log_18_2022-10-28-13-35-38.bin
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 /mnt/hgfs/Downloads/flight_logs/NavImprovements/2022-11-02-11-19-12.bin
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 /mnt/hgfs/Downloads/flight_logs/NavImprovements/2022-11-02-13-32-32.bin
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 /mnt/hgfs/Downloads/flight_logs/NavImprovements/2022-11-09-14-26-52.bin
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 /mnt/hgfs/Downloads/flight_logs/NavImprovements/2022-11-09-15-36-41.bin
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 /mnt/hgfs/Downloads/flight_logs/NavImprovements/2022-11-10-12-02-24.bin
./build/sitl/tool/Replay --parm EK3_FLOW_USE=3 --parm EK3_FLOW_M_NSE=0.075 /mnt/hgfs/Downloads/flight_logs/NavImprovements/2022-11-10-13-21-50.bin
mkdir -p /mnt/hgfs/Downloads/flight_logs/NavImprovements/PPDS-UKR
cp ./logs/* /mnt/hgfs/Downloads/flight_logs/NavImprovements/PPDS-UKR
./analysis/graph_logs.py /mnt/hgfs/Downloads/flight_logs/NavImprovements/PPDS-UKR/*.BIN

# rm logs/*
# ./build/sitl/tool/Replay --parm EK3_TNAV_V_SIGMA=1.25  --parm EK3_TNAV_SLEW_MS=100 /mnt/hgfs/Downloads/flight_logs/NavImprovements/SV-PPDS-178.BIN
# ./build/sitl/tool/Replay --parm EK3_TNAV_V_SIGMA=1.25  --parm EK3_TNAV_SLEW_MS=110 /mnt/hgfs/Downloads/flight_logs/NavImprovements/SV-PPDS-178.BIN
# ./build/sitl/tool/Replay --parm EK3_TNAV_V_SIGMA=1.25  --parm EK3_TNAV_SLEW_MS=120 /mnt/hgfs/Downloads/flight_logs/NavImprovements/SV-PPDS-178.BIN
# ./build/sitl/tool/Replay --parm EK3_TNAV_V_SIGMA=1.25  --parm EK3_TNAV_SLEW_MS=130 /mnt/hgfs/Downloads/flight_logs/NavImprovements/SV-PPDS-178.BIN
# ./build/sitl/tool/Replay --parm EK3_TNAV_V_SIGMA=1.25  --parm EK3_TNAV_SLEW_MS=140 /mnt/hgfs/Downloads/flight_logs/NavImprovements/SV-PPDS-178.BIN
# ./build/sitl/tool/Replay --parm EK3_TNAV_V_SIGMA=1.25  --parm EK3_TNAV_SLEW_MS=150 /mnt/hgfs/Downloads/flight_logs/NavImprovements/SV-PPDS-178.BIN
# mkdir -p /mnt/hgfs/Downloads/flight_logs/NavImprovements/Tuning
# cp ./logs/* /mnt/hgfs/Downloads/flight_logs/NavImprovements/Tuning
# ./analysis/graph_logs.py /mnt/hgfs/Downloads/flight_logs/NavImprovements/Tuning/*.BIN
