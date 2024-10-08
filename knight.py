def is_safe(x, y, board, N, M):
    return 0 <= x < N and 0 <= y < M and board[x][y] == -1

def get_degree(x, y, board, moves_x, moves_y, N, M):
    count = 0
    for i in range(8):
        next_x = x + moves_x[i]
        next_y = y + moves_y[i]
        if is_safe(next_x, next_y, board, N, M):
            count += 1
    return count

def print_solution(board):
    result = []
    for i in range(len(board)):
        for j in range(len(board[i])):
            result.append((i, j, board[i][j]))
    result.sort(key=lambda x: x[2])
    for x, y, move in result:
        print(x, y)

def solve_knights_tour(N, M, start_x, start_y):
    board = [[-1 for _ in range(M)] for _ in range(N)]
    moves_x = [2, 1, -1, -2, -2, -1, 1, 2]
    moves_y = [1, 2, 2, 1, -1, -2, -2, -1]

    board[start_x][start_y] = 0
    if not solve_knights_tour_util(start_x, start_y, 1, board, moves_x, moves_y, N, M):
        print("No solution exists")
    else:
        print_solution(board)

def solve_knights_tour_util(x, y, move_count, board, moves_x, moves_y, N, M):
    if move_count == N * M:
        return True

    next_moves = []
    for i in range(8):
        next_x = x + moves_x[i]
        next_y = y + moves_y[i]
        if is_safe(next_x, next_y, board, N, M):
            degree = get_degree(next_x, next_y, board, moves_x, moves_y, N, M)
            next_moves.append((degree, next_x, next_y))

    next_moves.sort()

    for _, next_x, next_y in next_moves:
        board[next_x][next_y] = move_count
        if solve_knights_tour_util(next_x, next_y, move_count + 1, board, moves_x, moves_y, N, M):
            return True
        board[next_x][next_y] = -1

    return False

if __name__ == "__main__":
    N, M = map(int, input("Enter the size of the board (N M): ").split())
    start_x, start_y = map(int, input("Enter the starting position of the knight (x y): ").split())
    solve_knights_tour(N, M, start_x, start_y)