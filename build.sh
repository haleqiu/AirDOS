echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2 || exit
mkdir build
cd build || exit
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../g2o || exit

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build || exit
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary || exit
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building the system ..."

mkdir build
cd build || exit
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

echo "Generating binary file"

../Vocabulary/to_binary
