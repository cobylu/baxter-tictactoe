class Human_Player():
    def action(self, board):
        move = raw_input("Input an action: ")
        return int(move)

    def set_player(self, player):
        return self