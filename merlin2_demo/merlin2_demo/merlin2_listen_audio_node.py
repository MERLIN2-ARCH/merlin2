
from genericpath import exists
import rclpy
from simple_node import Node

from std_msgs.msg import String

from kant_dto import PddlPropositionDto, PddlObjectDto
from kant_dao import ParameterLoader

from .pddl import sound_listened, sound_type


class Merlin2ListenAudio(Node):

    def __init__(self):

        super().__init__("merlin2_listen_audio")

        # loading parameters
        parameter_loader = ParameterLoader(self)
        self.__dao_factory = parameter_loader.get_dao_factory()
        self.__pddl_proposition_dao = self.__dao_factory.create_pddl_proposition_dao()

        sound_listened_list_prop_old = self.__pddl_proposition_dao.get_by_predicate(
            sound_listened.get_name())
        self._sound_listened_prop_old = None

        if sound_listened_list_prop_old:
            self._sound_listened_prop_old = sound_listened_list_prop_old[0]

        self._subscription = self.create_subscription(
            String,
            '/sound_recognition/sound_recognition',
            self.sound_listened,
            10)

    def sound_listened(self, msg):
        """ audio listen from sound_recognition node """
        self.get_logger().info("I Listen a "+msg.data)

        if not self._sound_listened_prop_old is None:
            self.__pddl_proposition_dao.delete(
                self._sound_listened_prop_old)


        self.sound = PddlObjectDto(sound_type, msg.data)

        sound_listened_prop = PddlPropositionDto(
            sound_listened, [self.sound])

        succeed = self.__pddl_proposition_dao.save(sound_listened_prop)
        self._sound_listened_prop_old = sound_listened_prop


def main(args=None):
    rclpy.init(args=args)

    node_listen = Merlin2ListenAudio()
    node_listen.join_spin()
    node_listen.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
