"""
Tic-Tac-Toe AI with alpha-beta minimax and depth-4 cutoff + heuristic.

- Board: 3x3, cells in {'.','X','O'}
- AI plays 'X' (Max), random agent plays 'O' (Min)
- Minimax uses alpha–beta pruning
- Cutoff at depth >= 4, then use heuristic()
- At terminal states (win/draw), return exact utilities

Run: python this_file.py
It will play 3 games vs a random agent, alternating who starts.
"""

import random
from copy import deepcopy

EMPTY, X, O = '.', 'X', 'O'
LINES = [
    # rows
    [(0,0),(0,1),(0,2)],
    [(1,0),(1,1),(1,2)],
    [(2,0),(2,1),(2,2)],
    # cols
    [(0,0),(1,0),(2,0)],
    [(0,1),(1,1),(2,1)],
    [(0,2),(1,2),(2,2)],
    # diags
    [(0,0),(1,1),(2,2)],
    [(0,2),(1,1),(2,0)],
]

def new_board():
    return [[EMPTY]*3 for _ in range(3)]

def available_moves(board):
    return [(r,c) for r in range(3) for c in range(3) if board[r][c]==EMPTY]

def place(board, r, c, p):
    b = deepcopy(board)
    b[r][c] = p
    return b

def winner(board):
    # returns 'X' or 'O' if a player has won, else None
    for line in LINES:
        vals = [board[r][c] for (r,c) in line]
        if vals[0] != EMPTY and vals.count(vals[0])==3:
            return vals[0]
    return None

def full(board):
    return all(board[r][c]!=EMPTY for r in range(3) for c in range(3))

def terminal(board):
    w = winner(board)
    return w is not None or full(board)

# ---------- Heuristic (used at cutoff depth) ----------
def line_score(cells, me, opp):
    """
    Score one 3-cell line from the perspective of 'me'.
    Rules of thumb (open = no opponent in the line):
      - 3 of me  -> big win (handled at terminal, but keep bonus)
      - 2 of me + 1 empty (open two) -> strong +10
      - 1 of me + 2 empty -> small +3 (future potential)
      - symmetric negatives for opponent
      - mixed lines (both me and opp present) -> 0
    """
    m = cells.count(me)
    o = cells.count(opp)
    e = cells.count(EMPTY)
    if m>0 and o>0:
        return 0  # blocked line
    if m==3:  # should be terminal, but give a big hint if seen at cutoff
        return 100
    if o==3:
        return -100
    # open lines
    if m==2 and e==1:
        return 10
    if m==1 and e==2:
        return 3
    if o==2 and e==1:
        return -10
    if o==1 and e==2:
        return -3
    return 0

def heuristic(board, me=X):
    opp = O if me==X else X
    score = 0
    for line in LINES:
        cells = [board[r][c] for (r,c) in line]
        score += line_score(cells, me, opp)
    # Slight bonus for center and corners control (classic Tic-Tac-Toe features)
    center = board[1][1]
    if center == me: score += 2
    elif center == opp: score -= 2
    corners = [(0,0),(0,2),(2,0),(2,2)]
    score += sum(1 for (r,c) in corners if board[r][c]==me)
    score -= sum(1 for (r,c) in corners if board[r][c]==opp)
    return score

# ---------- Minimax with alpha-beta + cutoff ----------
def minimax(board, depth, alpha, beta, maximizing, me=X):
    opp = O if me==X else X
    w = winner(board)
    if w == me:
        # prefer quicker wins / delay losses: scale by remaining moves
        return 1000 - depth, None
    if w == opp:
        return -1000 + depth, None
    if full(board):
        return 0, None
    # cutoff
    if depth >= 4:
        return heuristic(board, me), None

    moves = available_moves(board)
    if maximizing:
        best_val, best_move = -10**9, None
        for (r,c) in moves:
            val, _ = minimax(place(board,r,c,me), depth+1, alpha, beta, False, me)
            if val > best_val:
                best_val, best_move = val, (r,c)
            alpha = max(alpha, best_val)
            if beta <= alpha:
                break  # prune
        return best_val, best_move
    else:
        best_val, best_move = 10**9, None
        for (r,c) in moves:
            val, _ = minimax(place(board,r,c,opp), depth+1, alpha, beta, True, me)
            if val < best_val:
                best_val, best_move = val, (r,c)
            beta = min(beta, best_val)
            if beta <= alpha:
                break  # prune
        return best_val, best_move

# ---------- Play utilities ----------
def print_board(board):
    rows = [' '.join(board[r]) for r in range(3)]
    print('\n'.join(rows))
    print()

def random_move(board):
    return random.choice(available_moves(board))

def ai_move(board, me=X):
    _, move = minimax(board, depth=0, alpha=-10**9, beta=10**9, maximizing=True, me=me)
    # Fallback if heuristic cut off yields None (shouldn't happen, but just in case)
    if move is None:
        mv = available_moves(board)
        return random.choice(mv) if mv else None
    return move

def play_game(ai_first=True, seed=None, verbose=True):
    if seed is not None:
        random.seed(seed)
    board = new_board()
    current = X if ai_first else O
    if verbose:
        print("AI is 'X' | Random is 'O'")
        print("Starting player:", current)
        print_board(board)

    while True:
        if current == X:
            r, c = ai_move(board, me=X)
            board[r][c] = X
            if verbose:
                print("AI plays X at", (r,c))
                print_board(board)
        else:
            r, c = random_move(board)
            board[r][c] = O
            if verbose:
                print("Random plays O at", (r,c))
                print_board(board)

        w = winner(board)
        if w or full(board):
            if verbose:
                if w == X: print("Result: AI (X) WINS\n")
                elif w == O: print("Result: Random (O) WINS\n")
                else: print("Result: DRAW\n")
            return w

        current = O if current==X else X

# ---------- Demo: 3 example games vs random ----------
if __name__ == "__main__":
    results = []
    results.append(play_game(ai_first=True,  seed=1,  verbose=True))
    results.append(play_game(ai_first=False, seed=2,  verbose=True))
    results.append(play_game(ai_first=True,  seed=3,  verbose=True))

    wins = results.count(X)
    losses = results.count(O)
    draws = results.count(None)
    print(f"Summary vs Random (3 games): {wins} wins, {losses} losses, {draws} draws")
