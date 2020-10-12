
## Mongoengine Installation
```
sudo pip3 install mongoengine
sudo pip3 install dnspython
```
## Mongo Installation
```
wget -qO - https://www.mongodb.org/static/pgp/server-4.4.asc | sudo apt-key add -
sudo apt-get install gnupg
wget -qO - https://www.mongodb.org/static/pgp/server-4.4.asc | sudo apt-key add -
echo "deb [ arch=amd64,arm64 ] https://repo.mongodb.org/apt/ubuntu focal/mongodb-org/4.4 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-4.4.list
sudo apt-get update
sudo apt-get install -y mongodb-org
sudo systemctl start mongod
```

## Mongo Compass (Optional)
https://docs.mongodb.com/compass/master/install/

## Running Tests
```
cd ~/ros2_foxy/src/merlin2/pddl_dao/
pytest --cov=~/ros2_foxy/install/pddl_dao/lib/python3.8/site-packages/pddl_dao ./test/pddl_dao_tests/*.py
```
