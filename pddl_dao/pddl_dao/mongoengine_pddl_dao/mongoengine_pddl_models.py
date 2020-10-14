
""" mongoengine models"""

import mongoengine


class PddlTypeModel(mongoengine.Document):
    """ pddl type model """

    meta = {"collection": "pddl_type"}
    type_name = mongoengine.StringField(unique=True)


class PddlObjectModel(mongoengine.Document):
    """ pddl object model """

    meta = {"collection": "pddl_object"}
    object_name = mongoengine.StringField(unique=True)
    PddlTypeModel = mongoengine.ReferenceField(
        PddlTypeModel, reverse_delete_rule=mongoengine.CASCADE)


class PddlPredicateModel(mongoengine.Document):
    """ pddl predicate model """

    meta = {"collection": "pddl_predicate"}
    predicate_name = mongoengine.StringField(unique=True)
    pddl_types = mongoengine.ListField(
        mongoengine.ReferenceField(PddlTypeModel,
                                   reverse_delete_rule=mongoengine.CASCADE),
        unique=True)


class PddlPropositionModel(mongoengine.Document):
    """ pddl proposition model """

    meta = {"collection": "pddl_proposition"}
    PddlPredicateModel = mongoengine.ReferenceField(
        PddlPredicateModel, reverse_delete_rule=mongoengine.CASCADE)
    pddl_objects = mongoengine.ListField(
        mongoengine.ReferenceField(PddlObjectModel, reverse_delete_rule=mongoengine.CASCADE))
    is_goal = mongoengine.BooleanField()


class PddlParameterModel(mongoengine.EmbeddedDocument):
    """ pddl parameter model """

    meta = {"collection": "pddl_parameter"}
    parameter_name = mongoengine.StringField(unique=True)
    PddlTypeModel = mongoengine.ReferenceField(PddlTypeModel)


class PddlConditionEffectModel(mongoengine.EmbeddedDocument):
    """ pddl contion/effect model """

    meta = {"collection": "pddl_condition_effect"}
    time = mongoengine.StringField()
    is_negative = mongoengine.BooleanField()
    PddlPredicateModel = mongoengine.ReferenceField(PddlPredicateModel)
    pddl_parameters = mongoengine.EmbeddedDocumentListField(PddlParameterModel)


class PddlActionModel(mongoengine.Document):
    """ pddl action model """

    meta = {"collection": "pddl_action"}

    action_name = mongoengine.StringField(unique=True)
    duration = mongoengine.IntField(default=10)
    durative = mongoengine.BooleanField(default=True)

    _pddl_predicates_used = mongoengine.ListField(
        mongoengine.ReferenceField(PddlPredicateModel, reverse_delete_rule=mongoengine.CASCADE))

    pddl_parameters = mongoengine.EmbeddedDocumentListField(PddlParameterModel)

    conditions = mongoengine.EmbeddedDocumentListField(
        PddlConditionEffectModel)
    effects = mongoengine.EmbeddedDocumentListField(PddlConditionEffectModel)
