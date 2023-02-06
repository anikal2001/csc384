from copy import deepcopy
from heapq import heappush, heappop
import time
import argparse
import sys

#====================================================================================

char_goal = '1'
char_single = '2'

class Piece:
    """
    This represents a piece on the Hua Rong Dao puzzle.
    """

    def __init__(self, is_goal, is_single, coord_x, coord_y, orientation):
        """
        :param is_goal: True if the piece is the goal piece and False otherwise.
        :type is_goal: bool
        :param is_single: True if this piece is a 1x1 piece and False otherwise.
        :type is_single: bool
        :param coord_x: The x coordinate of the top left corner of the piece.
        :type coord_x: int
        :param coord_y: The y coordinate of the top left corner of the piece.
        :type coord_y: int
        :param orientation: The orientation of the piece (one of 'h' or 'v') 
            if the piece is a 1x2 piece. Otherwise, this is None
        :type orientation: str
        """

        self.is_goal = is_goal
        self.is_single = is_single
        self.coord_x = coord_x
        self.coord_y = coord_y
        self.orientation = orientation

    def __repr__(self):
        return '{} {} {} {} {}'.format(self.is_goal, self.is_single, \
            self.coord_x, self.coord_y, self.orientation)
    
    def move_left(self):
        if(self.coord_x > 0):
            self.coord_x -= 1
        else:
            print('Left not Possible')
    
    def move_right(self):
        if(self.coord_x < 4):
            self.coord_x += 1
        else:
            print('Right not Possible')
    
    def move_up(self):
        if(self.coord_y < 5):
            self.coord_y += 1
        else:
            print('Up not Possible')
    
    def move_down(self):
        if(self.coord_y > 0):
            self.coord_y -= 1
        else:
            print('Up not Possible')

class Board:
    """
    Board class for setting up the playing board.
    """

    def __init__(self, pieces):
        """
        :param pieces: The list of Pieces
        :type pieces: List[Piece]
        """

        self.width = 4
        self.height = 5

        self.pieces = pieces

        # self.grid is a 2-d (size * size) array automatically generated
        # using the information on the pieces when a board is being created.
        # A grid contains the symbol for representing the pieces on the board.
        self.grid = []
        self.__construct_grid()


    def __construct_grid(self):
        """
        Called in __init__ to set up a 2-d grid based on the piece location information.

        """

        for i in range(self.height):
            line = []
            for j in range(self.width):
                line.append('.')
            self.grid.append(line)

        for piece in self.pieces:
            if piece.is_goal:
                self.grid[piece.coord_y][piece.coord_x] = char_goal
                self.grid[piece.coord_y][piece.coord_x + 1] = char_goal
                self.grid[piece.coord_y + 1][piece.coord_x] = char_goal
                self.grid[piece.coord_y + 1][piece.coord_x + 1] = char_goal
            elif piece.is_single:
                self.grid[piece.coord_y][piece.coord_x] = char_single
            else:
                if piece.orientation == 'h':
                    self.grid[piece.coord_y][piece.coord_x] = '<'
                    self.grid[piece.coord_y][piece.coord_x + 1] = '>'
                elif piece.orientation == 'v':
                    self.grid[piece.coord_y][piece.coord_x] = '^'
                    self.grid[piece.coord_y + 1][piece.coord_x] = 'v'

    def display(self):
        """
        Print out the current board.

        """
        for i, line in enumerate(self.grid):
            for ch in line:
                print(ch, end='')
            print()
    
    def isValid(self):
        pass

    def emptySpace(self):
        for row in self.grid:
            if '.' in row:
                return (self.grid.index(row),row.index('.'))
        

class State:
    """
    State class wrapping a Board with some extra current state information.
    Note that State and Board are different. Board has the locations of the pieces. 
    State has a Board and some extra information that is relevant to the search: 
    heuristic function, f value, current depth and parent.
    """

    def __init__(self, board, f, depth, parent=None):
        """
        :param board: The board of the state.
        :type board: Board
        :param f: The f value of current state.
        :type f: int
        :param depth: The depth of current state in the search tree.
        :type depth: int
        :param parent: The parent of current state.
        :type parent: Optional[State]
        :param succ: The successor states of current state.
        :type succ: Optional[List[State]]
        """
        self.board = board
        self.f = f
        self.depth = depth
        self.parent = parent
        self.id = hash(board)  # The id for breaking ties.
        self.succ = []
        for i in range(5):
            self.succ.append()

def isGoal(self):
    return 
def read_from_file(filename):
    """
    Load initial board from a given file.

    :param filename: The name of the given file.
    :type filename: str
    :return: A loaded board
    :rtype: Board
    """

    puzzle_file = open(filename, "r")

    line_index = 0
    pieces = []
    g_found = False

    for line in puzzle_file:

        for x, ch in enumerate(line):

            if ch == '^': # found vertical piece
                pieces.append(Piece(False, False, x, line_index, 'v'))
            elif ch == '<': # found horizontal piece
                pieces.append(Piece(False, False, x, line_index, 'h'))
            elif ch == char_single:
                pieces.append(Piece(False, True, x, line_index, None))
            elif ch == char_goal:
                if g_found == False:
                    pieces.append(Piece(True, False, x, line_index, None))
                    g_found = True
        line_index += 1

    puzzle_file.close()

    board = Board(pieces)
    # board.display()
    board.emptySpace()
    
    return board


def isGoal(state):
    if state.board.grid[3][1] == char_goal and state.board.grid[4][2] == char_goal: 
        return True 
    return False

def generateSuccessors(state):
        succs = []
        for m in state.moves:
            p = deepcopy(self.state)
            p.doMove(m) 
            if p.zero is not self.state.zero:
                succs.put(Node(p, self, m))
        return succs

def dfs(initialState):
    depth = 0
    result = None
    while result == None:
        result = depthLimited(initialState,2)
        depth +=1
    return result
    

def depthLimited(state, depth):
    frontier = []
    frontier.append(state)
    i=0
    while True:
        if len(frontier) == 0:
            return None
        actual = frontier.pop()
        if isGoal(actual):
            return actual
        elif actual.depth != depth:
            succ = generateSuccessors(state)
            while not len(succ) == 0:
                frontier.append(succ.pop())
        

# def aSearch(self, heuristic):
#     actual = self.start
#     leaves = PriorityQueue()
#     leaves.put((actual.costHeur(heuristic), actual))
#     closed = list()
#     while True:
#         if leaves.empty():
#             return None
#         actual = leaves.get()[1]
#         if actual.goalState():
#             return actual
#         elif actual.state.puzzle not in closed:
#             closed.append(actual.state.puzzle)
#             succ = actual.succ()
#             while not succ.empty():
#                 child = succ.get()
#                 leaves.put((child.costHeur(heuristic)+child.depth, child))

def reconstruct_path(cameFrom, current):
    total_path = {current}
    while current in cameFrom.Keys:
        current = cameFrom[current]
        total_path[current]
    return total_path

# A* finds a path from start to goal.
# h is the heuristic function. h(n) estimates the cost to reach goal from node n.
def A_Star(start, goal, h):
    # The set of discovered nodes that may need to be (re-)expanded.
    # Initially, only the start node is known.
    # This is usually implemented as a min-heap or priority queue rather than a hash-set.
    frontier = []

    # For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start
    # to n currently known.
    cameFrom = {}

    # For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
    gScore = {float('inf')}
    gScore[start] = 0

    # For node n, fScore[n] := gScore[n] + h(n). fScore[n] represents our current best guess as to
    # how cheap a path could be from start to finish if it goes through n.
    fScore = {float('inf')}
    fScore[start] = h(start)

    while frontier.length != 0:
        # This operation can occur in O(Log(N)) time if openSet is a min-heap or a priority queue
        current = frontier[0]
        if current == goal:
            return reconstruct_path(cameFrom, current)

        heappop(frontier)
        # for each neighbor of current
        #     // d(current,neighbor) is the weight of the edge from current to neighbor
        #     // tentative_gScore is the distance from start to the neighbor through current
        #     tentative_gScore := gScore[current] + d(current, neighbor)
        #     if tentative_gScore < gScore[neighbor]
        #         // This path to neighbor is better than any previous one. Record it!
        #         cameFrom[neighbor] := current
        #         gScore[neighbor] := tentative_gScore
        #         fScore[neighbor] := tentative_gScore + h(neighbor)
        #         if neighbor not in openSet
        #             openSet.add(neighbor)

    # // Open set is empty but goal was never reached
    # return failure


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--inputfile",
        type=str,
        required=True,
        help="The input file that contains the puzzle."
    )
    parser.add_argument(
        "--outputfile",
        type=str,
        required=True,
        help="The output file that contains the solution."
    )
    parser.add_argument(
        "--algo",
        type=str,
        required=True,
        choices=['astar', 'dfs'],
        help="The searching algorithm."
    )
    args = parser.parse_args()

    # read the board from the file
    board = read_from_file(args.inputfile)
    state = State(board, 0, 0, None)
    dfs(state)
    




