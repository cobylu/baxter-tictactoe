class Human_Player():
    def action(self, board):
        print('Input an action:')
        move = input()
        return int(move)

    def set_player(self, player):
        return self