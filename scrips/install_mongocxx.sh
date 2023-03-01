# install dependencies
apt install libmongoc-dev libmongoc-1.0-0 -y # Ubuntu 20, mongoc 1.16.1

# download mongo-cxx
curl -OL https://github.com/mongodb/mongo-cxx-driver/releases/download/r3.7.0/mongo-cxx-driver-r3.7.0.tar.gz
tar -xzf mongo-cxx-driver-r3.7.0.tar.gz

# cmake and make
cd mongo-cxx-driver-r3.7.0/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DBSONCXX_POLY_USE_BOOST=1
cmake --build .
cmake --build . --target install

# env var
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >>~/.bashrc

# remove
cd ../..
rm mongo-cxx-driver-r3.7.0.tar.gz
rm -rf mongo-cxx-driver-r3.7.0
