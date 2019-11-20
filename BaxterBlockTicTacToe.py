#!/usr/bin/env python

import time

import rospy
import baxter_interface

from TicTacToe import TicTacToe
from MiniMax_Player import MiniMax_Player
from Human_Player import Human_Player


class BaxterTicTacToe():
    def __init__(self):
        self.base_positions = [
            {'right_s0': 0.3616359707439863, 'right_s1': -0.3831117017743821, 'right_w0': -1.2816409482782631, 'right_w1': 1.2827914338691773, 'right_w2': 1.6379079862646506, 'right_e0': 1.5205584559914165, 'right_e1': 2.288315840328066},
            {'right_s0': 0.3616359707439863, 'right_s1': -0.3831117017743821, 'right_w0': -1.2816409482782631, 'right_w1': 1.2827914338691773, 'right_w2': 1.6379079862646506, 'right_e0': 1.5205584559914165, 'right_e1': 2.288315840328066},
            {'right_s0': -1.2494273517326695, 'right_s1': 1.045024411746938, 'right_w0': -1.164291418005029, 'right_w1': 0.22357769983429907, 'right_w2': 3.0426508927707183, 'right_e0': 1.3414661990057943, 'right_e1': 1.9059711289476267},
            {'right_s0': 0.3616359707439863, 'right_s1': -0.3831117017743821, 'right_w0': -1.2816409482782631, 'right_w1': 1.2827914338691773, 'right_w2': 1.6379079862646506, 'right_e0': 1.5205584559914165, 'right_e1': 2.288315840328066},
            {'right_s0': 0.3616359707439863, 'right_s1': -0.3831117017743821, 'right_w0': -1.2816409482782631, 'right_w1': 1.2827914338691773, 'right_w2': 1.6379079862646506, 'right_e0': 1.5205584559914165, 'right_e1': 2.288315840328066},
            {'right_s0': -1.240223467005357, 'right_s1': 1.039655478989339, 'right_w0': -1.1282428694897217, 'right_w1': 0.1695048770613382, 'right_w2': 3.0426508927707183, 'right_e0': 1.2789564818994636, 'right_e1': 1.7636944108712544},
            {'right_s0': 0.3616359707439863, 'right_s1': -0.3831117017743821, 'right_w0': -1.2816409482782631, 'right_w1': 1.2827914338691773, 'right_w2': 1.6379079862646506, 'right_e0': 1.5205584559914165, 'right_e1': 2.288315840328066},
            {'right_s0': 0.3616359707439863, 'right_s1': -0.3831117017743821, 'right_w0': -1.2816409482782631, 'right_w1': 1.2827914338691773, 'right_w2': 1.6379079862646506, 'right_e0': 1.5205584559914165, 'right_e1': 2.288315840328066},
            {'right_s0': -1.2613157028387814, 'right_s1': 1.0411894597772247, 'right_w0': -1.2317865726719872, 'right_w1': 0.31906800388016604, 'right_w2': 3.0430343879676895, 'right_e0': 1.2386894862174715, 'right_e1': 1.6382914814616218}
        ]
        self.change_positions = [
            {'right_s0': 0.3616359707439863, 'right_s1': -0.3831117017743821, 'right_w0': -1.2816409482782631, 'right_w1': 1.2827914338691773, 'right_w2': 1.6379079862646506, 'right_e0': 1.5205584559914165, 'right_e1': 2.288315840328066},
            {'right_s0': 0.3616359707439863, 'right_s1': -0.3831117017743821, 'right_w0': -1.2816409482782631, 'right_w1': 1.2827914338691773, 'right_w2': 1.6379079862646506, 'right_e0': 1.5205584559914165, 'right_e1': 2.288315840328066},
            {'right_s0': -0.8567282700340035, 'right_s1': 1.0515438300954512, 'right_w0': -1.145883648550404, 'right_w1': -0.3029612056073692, 'right_w2': 3.0418839023767754, 'right_e0': 1.4630341764457133, 'right_e1': 1.822752671204843},
            {'right_s0': 0.3616359707439863, 'right_s1': -0.3831117017743821, 'right_w0': -1.2816409482782631, 'right_w1': 1.2827914338691773, 'right_w2': 1.6379079862646506, 'right_e0': 1.5205584559914165, 'right_e1': 2.288315840328066},
            {'right_s0': 0.3616359707439863, 'right_s1': -0.3831117017743821, 'right_w0': -1.2816409482782631, 'right_w1': 1.2827914338691773, 'right_w2': 1.6379079862646506, 'right_e0': 1.5205584559914165, 'right_e1': 2.288315840328066},
            {'right_s0': -0.8429224429430349, 'right_s1': 1.0515438300954512, 'right_w0': -1.0526943156863653, 'right_w1': -0.3106311095467963, 'right_w2': 3.0426508927707183, 'right_e0': 1.4078108680818384, 'right_e1': 1.6858448858860697},
            {'right_s0': 0.3616359707439863, 'right_s1': -0.3831117017743821, 'right_w0': -1.2816409482782631, 'right_w1': 1.2827914338691773, 'right_w2': 1.6379079862646506, 'right_e0': 1.5205584559914165, 'right_e1': 2.288315840328066},
            {'right_s0': 0.3616359707439863, 'right_s1': -0.3831117017743821, 'right_w0': -1.2816409482782631, 'right_w1': 1.2827914338691773, 'right_w2': 1.6379079862646506, 'right_e0': 1.5205584559914165, 'right_e1': 2.288315840328066},
            {'right_s0': -0.8678496307461728, 'right_s1': 1.0515438300954512, 'right_w0': -1.0967962633380708, 'right_w1': -0.2757330466224031, 'right_w2': 3.0426508927707183, 'right_e0': 1.3176894967935704, 'right_e1': 1.5343642830823851}
        ]

    def move_to_position(self, position):
        baxter_interface.Limb('right').move_to_joint_positions(self.base_positions[position])
        print(baxter_interface.Limb('right').joint_angles())
        time.sleep(1)


    def change_block_to_x(self, position):
        baxter_interface.Limb('right').move_to_joint_positions(self.change_positions[position])
        print(baxter_interface.Limb('right').joint_angles())
        time.sleep(1)

    def change_block_to_o(self, position):
        pass

    def get_board(self):
        pass

    def reset_board(self):
        pass

    def start_game(self):
        pass

if __name__ == "__main__":
    baxter_tictactoe = BaxterTicTacToe()
    baxter_tictactoe.move_to_position(3)
    baxter_tictactoe.change_block_to_x(3)