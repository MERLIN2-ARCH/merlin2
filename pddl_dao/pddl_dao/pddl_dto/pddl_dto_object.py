

class PDDL_DTO_Object:

    def __init__(self, pddl_type, object_name):

        self.set_pddl_type(pddl_type)
        self.set_object_name(object_name)

    def get_pddl_type(self):
        return self._pddl_type

    def set_pddl_type(self, pddl_type):
        self._pddl_type = pddl_type

    def get_object_name(self):
        return self._object_name

    def set_object_name(self, object_name):
        self._object_name = object_name

    def __str__(self):
        return self._object_name + " - " + self._pddl_type.get_type_name()
