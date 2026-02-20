cd ./foil_dynamics || exit
mkdir build
cd ./build || exit
cmake ..
make
cd ../..

cd ./rudder_dynamics || exit
mkdir build
cd ./build || exit
cmake ..
make
cd ../..

cd ./sail_limits || exit
mkdir build
cd ./build || exit
cmake ..
make
cd ../..

cd ./wind_arrow || exit
mkdir build
cd ./build || exit
cmake ..
make
cd ../..

