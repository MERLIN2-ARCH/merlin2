
from mongoengine import connect, disconnect


class MongoengineConnector:
    def __init__(self, uri: str = "mongodb://localhost:27017/merlin2"):
        self._uri = uri
        self.connection = None

    def connect(self) -> bool:
        disconnect()
        self.connection = connect(host=self._uri)
        return True

    def close(self) -> bool:
        self.connection.close()
        return True
