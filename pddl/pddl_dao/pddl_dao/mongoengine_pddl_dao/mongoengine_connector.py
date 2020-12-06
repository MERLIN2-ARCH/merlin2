
""" Mongoengine Connector """

from mongoengine import connect


class MongoengineConnector:
    """ Mongoengine Connector Class """

    def __init__(self, uri: str = "mongodb://localhost:27017/merlin2"):
        self._uri = uri
        self.connection = None

    def connect(self) -> bool:
        """ connect to mongo db

        Returns:
            bool: succeed
        """

        self.connection = connect(host=self._uri)
        return True

    def close(self) -> bool:
        """ close connection

        Returns:
            bool: succeed
        """

        if self.connection:
            self.connection.close()
        return True
