import mongoengine


class pddl_type(mongoengine.Document):
    type_name = mongoengine.StringField(unique=True)


class pddl_object(mongoengine.Document):
    object_name = mongoengine.StringField(unique=True)
    pddl_type = mongoengine.ReferenceField(
        pddl_type, reverse_delete_rule=mongoengine.CASCADE)


class pddl_predicate(mongoengine.Document):
    predicate_name = mongoengine.StringField(unique=True)
    pddl_types = mongoengine.ListField(
        mongoengine.ReferenceField(pddl_type, reverse_delete_rule=mongoengine.CASCADE), unique=True)


class pddl_proposition(mongoengine.Document):
    pddl_predicate = mongoengine.ReferenceField(
        pddl_predicate, reverse_delete_rule=mongoengine.CASCADE)
    pddl_objects = mongoengine.ListField(
        mongoengine.ReferenceField(pddl_object, reverse_delete_rule=mongoengine.CASCADE))
    is_goal = mongoengine.BooleanField()


class pddl_parameter(mongoengine.EmbeddedDocument):
    parameter_name = mongoengine.StringField(unique=True)
    pddl_type = mongoengine.ReferenceField(pddl_type)


class pddl_condition_effect(mongoengine.EmbeddedDocument):
    time = mongoengine.StringField()
    is_negative = mongoengine.BooleanField()
    pddl_predicate = mongoengine.ReferenceField(pddl_predicate)
    pddl_parameters = mongoengine.EmbeddedDocumentListField(pddl_parameter)


class pddl_action(mongoengine.Document):

    action_name = mongoengine.StringField(unique=True)
    duration = mongoengine.IntField(default=10)
    durative = mongoengine.BooleanField(default=True)

    _pddl_predicates_used = mongoengine.ListField(
        mongoengine.ReferenceField(pddl_predicate, reverse_delete_rule=mongoengine.CASCADE))

    pddl_parameters = mongoengine.EmbeddedDocumentListField(pddl_parameter)

    conditions = mongoengine.EmbeddedDocumentListField(pddl_condition_effect)
    effects = mongoengine.EmbeddedDocumentListField(pddl_condition_effect)
