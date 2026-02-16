cd ./foil_dynamics
mkdir build
cd ./build
cmake ..
make
cd ../..

cd ./rudder_dynamics
mkdir build
cd ./build
cmake ..
make
cd ../..

cd ./sail_limits
mkdir build
cd ./build
cmake ..
make
cd ../..

cd ./wind_arrow
mkdir build
cd ./build
cmake ..
make
cd ../..

cd ./buoyancy
mkdir build
cd ./build
cmake ..
make
cd ../..