import nqueens
import random
import math


def decayRate(decay_rate, T):
    return T * decay_rate


def simulatedAnnealing(start, decay_rate, threshold):
    current = start
    T = 100
    while (T > threshold):
        cost = current.h = nqueens.numAttackingQueens(current)  # have to use function to print intial
        successors = nqueens.getSuccessorStates(current)
        next_board = successors[
            random.randint(0, len(successors) - 1)]  # return a random neighbor by selecting a random one
        cost_next = next_board.h = nqueens.numAttackingQueens(next_board)  # get the cost of the random neighbor
        diff = cost - cost_next
        if diff > 0:
            current = next_board
        elif random.randrange(0, 1) > math.exp((diff / T)):
            current = next_board
            # maybe move based on probabilty
        T = decayRate(decay_rate, T)
    return current  # T < threshold exits and returns the current state


def main():
    print('in main')
    pairs = [[0.9, 0.000001], [0.75, 0.0000001], [0.5, 0.00000001]]
    board_size = [4, 8, 16]
    for i in pairs:
        print("#############################################\n", "Decay Rate: ", i[0], "    T Threshold: ", i[1],
              "\n""#############################################")
        for n in board_size:
            print("*********************\n", "  Board Size: ", n, "\n" "*********************")
            average_h = 0.0
            for g in range(0, 10):
                startingBoard = nqueens.Board(n)
                startingBoard.rand()
                final_board = simulatedAnnealing(startingBoard, i[0], i[1])
                print("*********************\n", "    Run ", g + 1, "\n" "*********************")
                print("Initial Board: ")
                startingBoard.printBoard()
                print("Initial Board h-value ", startingBoard.h)
                print("Final Board: ")
                final_board.printBoard()
                print("Final Board h-value: ", final_board.h)
                average_h += final_board.h
            average_h = average_h/10
            print("Average h-cost of final solutions: ", average_h)


if __name__ == '__main__':
    main()
