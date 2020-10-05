

class PDDL_DTO_Type:

    def __init__(self, type_name):

        self.set_type_name(type_name)

    def get_type_name(self):
        return self._type_name

    def set_type_name(self, type_name):
        self._type_name = type_name

    def __str__(self):
        return self._type_name
