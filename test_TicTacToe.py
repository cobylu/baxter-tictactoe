import unittest
from Human_Player import Human_Player
from TicTacToe import TicTacToe

class TicTacToeTestCase(unittest.TestCase):

    def setUp(self):
        self.tictactoe = TicTacToe(Human_Player(), Human_Player())

    def test_foo(self):
        self.assertTrue(self.tictactoe.is_same(['', '', '', '', '', '', '', '', '']))

        self.tictactoe.apply_move(1)
        self.assertTrue(self.tictactoe.is_same(['', 'O', '', '', '', '', '', '', '']))

    def test_is_cheater(self):
        self.assertFalse(self.tictactoe.is_cheater(['', '', '', '', '', '', '', '', '']))
        self.assertTrue(self.tictactoe.is_cheater(['', 'O', '', 'O', '', '', '', '', '']))
        self.assertTrue(self.tictactoe.is_cheater(['', 'X', '', '', '', '', '', '', '']))
        self.assertFalse(self.tictactoe.is_cheater(['', 'O', '', '', '', '', '', '', '']))

        self.tictactoe.apply_move(1)
        self.assertFalse(self.tictactoe.is_cheater(['', 'O', '', '', '', '', '', '', '']))
        self.assertFalse(self.tictactoe.is_cheater(['', 'O', '', '', '', '', '', '', '']))
        self.assertTrue(self.tictactoe.is_cheater(['', 'O', 'X', 'X', '', '', '', '', '']))


