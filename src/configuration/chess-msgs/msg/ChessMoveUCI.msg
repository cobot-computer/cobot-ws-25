# A UCI-formatted chess move. The move is represented as a string in long algebraic notation.
# For example: "e2e4", "e7e5", "e7e8q", etc. This may be empty if the move is a draw offer or a
# resign.
string move

# True if the move is a draw offer, false otherwise.
bool draw

# True if the move is a resign, false otherwise.
bool resign