from Human_Player import Human_Player
from MiniMax_Player import MiniMax_Player

class TicTacToe():
    """
    Tic-tac-toe game
    """

    def __init__(self, p1, p2):
        p1.set_player('O')
        p2.set_player('X')
        self.p1 = p1
        self.p2 = p2
        self.reset()
        self.winning_lines = [
            [0, 1, 2],
            [3, 4, 5],
            [6, 7, 8],
            [0, 3, 6],
            [1, 4, 7],
            [2, 5, 8],
            [0, 4, 8],
            [2, 4, 6]
        ]

    def reset(self):
        """
        Reset the state of the game.
        """
        self.board = ['' for i in range(9)]
        self.turn = 'O'

    def copy(self):
        """
        Return a copy of the game with identical state.
        """
        copy = TicTacToe(self.p1, self.p2)
        copy.board = [x for x in self.board]
        copy.turn = self.turn
        return copy

    def get_valid_moves(self):
        """
        Return the indicies of the board that are valid moves.
        """
        return [i for i in range(9) if self.is_valid_move(i)]

    def is_valid_move(self, move):
        """
        Returns whether a move is valid or not.
        """
        return 0 <= move < 9 and self.board[move] == ''

    def apply_move(self, move):
        """
        Applies a move to the game board and updates the turn.
        """
        if self.is_valid_move(move):
            self.board[move] = self.turn
            self.turn = 'X' if self.turn == 'O' else 'O'

    def forecast_move(self, move):
        """
        Creates a copy of the board, applies the proposed move, and returns the
        game's state. This is a utility for general game playing AI.
        """
        new_game = self.copy()
        new_game.apply_move(move)
        return new_game

    def is_game_over(self):
        """
        Returns 'X' or 'O' if that player has won the game. Returns '-' if there
        was a draw. Returns None if neither player has won.
        """
        for l in self.winning_lines:
            if self.board[l[0]] != '' and self.board[l[0]] == self.board[l[1]] == self.board[l[2]]:
                return self.board[l[0]]

        for p in self.board:
            if p == '':
                return None
        return '-'

    def is_cheater(self, board):
        """
        Returns true if the passed in board is different but not a valid state
        based on the next player's moves.
        """
        if board == self.board:
            return False
        
        diff = [board[i] if board[i] != self.board[i] else None for i in range(len(board))]
        diff = list(filter(lambda x: x is not None, diff))

        if len(diff) > 1:
            return True

        if diff[0] != self.turn:
            return True
        
        return False

    def is_same(self, board):
        """
        Returns true if the passed in board is the same as the current board.
        """
        return board == self.board

    def print_board(self):
        """
        Prints the game's state. The board is printed as a 3x3 grid with '_'
        where neither player has played a piece. Also print's the current
        player's turn.
        """
        for i in range(3):
            for j in range(3):
                idx = i * 3 + j
                print self.board[idx] if self.board[idx] != '' else '_',
            print ""
        print "Current player\'s turn: " + self.turn

    def play(self):
        """
        This runs the main game loop and returns 'X' or 'O' if a player won
        or '-' if there was a draw.
        """
        while self.is_game_over() is None:
            move = -1
            while not self.is_valid_move(move):
                self.print_board()
                if self.turn == 'O':
                    move = self.p1.action(self)
                else:
                    move = self.p2.action(self)
            self.apply_move(move)
        
        self.print_board()
        winner = self.is_game_over()
        if winner == '-':
            print "Draw!"
        else:
            print "Player " + winner + " won!"
        return winner

if __name__ == '__main__':
    tictactoe = TicTacToe(Human_Player(), MiniMax_Player(search_depth=2))
    tictactoe.play()