import cv2
import numpy as np
import BaxterCamera


class BaxterTicTacToeCv():
    def __init__(self, positions, x_lb, x_ub, multiple_x_bounds = False, o_lb, o_ub, multiple_o_bounds = False, threshold=0.5):
        self.positions = positions
        self.x_lb = x_lb
        self.x_ub = x_ub
        self.multiple_x_bounds = multiple_x_bounds
        self.o_lb = o_lb
        self.o_ub = o_ub
        self.multiple_o_bounds = multiple_o_bounds
        self.baxter_camera = BaxterCamera()
        self.threshold = threshold

    def get_board_state(self):
        img = cv2.imread("baxter_img.jpg")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        self.x_mask = inRange(hsv, self.x_lb, self.x_ub)
        self.o_mask = inRange(hsv, self.o_lb, self.o_ub)
        board = []
        for p in self.positions:
            board.append(self.get_color_at(p))
        return board

    def in_range(self, hsv, lb, ub, multiple):
        if multiple:
            mask = cv2.inRange(hsv, lb[0], ub[0])
            for i in range(1, len(lb)):
                mask += cv2.inRange(hsv, lb[i], ub[i])
            return mask
        else:
            return cv2.inRange(hsv, lb, ub)

    def get_color_at(self, position):
        x_slice = self.x_mask[position[0]:position[1],position[2]:position[3]]
        x_slice_total = np.sum(np.sum(x_slice > 0))
        o_slice = self.o_mask[position[0]:position[1],position[2]:position[3]]
        o_slice_total = np.sum(np.sum(o_slice > 0))
        total = (position[1] - position[0]) * (position[3] - position[2])

        if x_slice_total > o_slice_total and 1. * x_slice_total / total > self.threshold:
            return "X"
        elif o_slice_total > x_slice_total and 1. * o_slice_total / total > self.threshold:
            return "O"
        else:
            return ""

if __name__ == "__main__":
    positions = {(2550, 2700, 1030, 1160)}
    BLUE_LB = (80, 40, 40)
    BLUE_UB = (130, 255, 255)

    GREEN_LB = (35, 40, 40)
    GREEN_UB = (80, 255, 255)

    PURPLE_LB = (130, 40, 40)
    PURPLE_UB = (155, 255, 255)
    
    baxter_cv = BaxterTicTacToeCv(positions, GREEN_LB, GREEN_UB, PURPLE_LB, PURPLE_UB)
    print baxter_cv.get_board_state()