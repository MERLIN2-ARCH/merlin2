
from mongoengine import connect, disconnect


class MongoengineConnector:
    def __init__(self, uri="mongodb://localhost:27017/merlin2"):
        self._uri = uri
        self.connection = None

    def connect(self):
        disconnect()
        self.connection = connect(host=self._uri)

    def close(self):
        self.connection.close()
