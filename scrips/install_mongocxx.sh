# install dependencies
apt install libmongoc-dev libmongoc-1.0-0 -y # Ubuntu 20, mongoc 1.16.1

# download mongo-cxx
curl -OL https://github.com/mongodb/mongo-cxx-driver/archive/refs/tags/r3.4.2.tar.gz
tar -xzf r3.4.2.tar.gz

# cmake and make
cd mongo-cxx-driver-r3.4.2/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DBSONCXX_POLY_USE_BOOST=1
cmake --build .
cmake --build . --target install

# env var
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >>~/.bashrc

# remove
rm r3.4.2.tar.gz
rm -rf mongo-cxx-driver-r3.4.2
