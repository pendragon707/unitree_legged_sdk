cd /unitree_legged_sdk
mkdir -p build
cd build
cmake ..
make 

cd /unitree_legged_sdk/python_wrapper
mkdir build
cd build
cmake ..
make 

sudo ldconfig -v