

class PDDL_DTO_Action:

    def __init__(self, action_name,
                 parameters_list=None, conditions_list=None, effects_list=None,
                 durative=True, duration=10):

        self.set_action_name(action_name)
        self.set_parameters_list(parameters_list)
        self.set_durative(durative)
        self.set_duration(duration)
        self.set_conditions_list(conditions_list)
        self.set_effects_list(effects_list)

    def get_action_name(self):
        return self._action_name

    def set_action_name(self, action_name):
        self._action_name = action_name

    def get_durative(self):
        return self._durative

    def set_durative(self, durative):
        self._durative = durative

    def get_duration(self):
        return self._duration

    def set_duration(self, duration):
        self._duration = duration

    def get_parameters_list(self):
        return self._parameters_list

    def set_parameters_list(self, parameters_list):
        if(parameters_list):
            self._parameters_list = parameters_list
        else:
            self._parameters_list = []

    def get_conditions_list(self):
        return self._conditions_list

    def set_conditions_list(self, conditions_list):
        if(conditions_list):
            self._conditions_list = conditions_list
        else:
            self._conditions_list = []

    def get_effects_list(self):
        return self._effects_list

    def set_effects_list(self, effects_list):
        if(effects_list):
            self._effects_list = effects_list
        else:
            self._effects_list = []

    def __str__(self):
        string = "(:"
        if(self._durative):
            string += "durative-"
        string += "action " + self._action_name

        string += "\n\t:parameters ("
        for parameter in self._parameters_list:
            string += " ?" + parameter.get_object_name() + " - " + \
                parameter.get_pddl_type().get_type_name()
        string += ")"

        if(self._durative):
            string += "\n\t:duration (= ?duration " + str(self._duration) + ")"

        if(self._durative):
            string += "\n\t:condition ("
        else:
            string += "\n\t:precondition ("
        if(len(self._conditions_list) > 1):
            string += "and"
        for condi in self._conditions_list:
            string += "\n\t\t" + str(condi)
        string += "\n\t)"

        string += "\n\t:effect ("
        if(len(self._effects_list) > 1):
            string += "and"
        for effect in self._effects_list:
            string += "\n\t\t" + str(effect)
        string += "\n\t)"

        string += "\n)"

        return string
