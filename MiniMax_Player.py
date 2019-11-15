import random
import time

class MiniMax_Player():
    def __init__(self, search_depth=4):
        self.search_depth = search_depth

    def set_player(self, player):
        self.player = player

    def action(self, game):
        """
        Returns the move the game should execute on behalf of the Minimax player
        """
        best_move, utility = self.minimax(game, depth=self.search_depth, maximizing_player=True)
        return best_move

    def utility(self, game):
        """
        Utility function for use with minimax. Return -inf if the move results in a loss, inf if
        the move results in a win. For other moves it returns a random value between 0 and 1 to
        allow for randomized play
        """
        if game.is_game_over() == self.player:
            return float("inf")
        elif game.is_game_over() == self.player:
            return float("-inf")
        else:
            return random.random()

    def minimax(self, game, depth=4, maximizing_player=True):
        """
        Minimax algorithm
        """
        if depth == 0 or game.is_game_over():
            return 0, self.utility(game)
        turn = game.turn
        if maximizing_player:
            best_val = float("-inf")
            best_move = game.get_valid_moves()[0]
            for m in game.get_valid_moves():
                forecast_board = game.forecast_move(m)
                forecast_move, forecast_score = self.minimax(forecast_board, depth - 1, turn == forecast_board.turn)
                if forecast_score > best_val:
                    best_val = forecast_score
                    best_move = m
            return best_move, best_val
        else:
            best_val = float("inf")
            best_move = game.get_valid_moves()[0]
            for m in game.get_valid_moves():
                forecast_board = game.forecast_move(m)
                forecast_move, forecast_score = self.minimax(forecast_board, depth - 1, turn != forecast_board.turn)
                if forecast_score < best_val:
                    best_val = forecast_score
                    best_move = m
            return best_move, best_val
