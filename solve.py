from board import *
import copy


def a_star(init_board, hfn):
    """
    Run the A_star search algorithm given an initial board and a heuristic function.

    If the function finds a goal state, it returns a list of states representing
    the path from the initial state to the goal state in order and the cost of
    the solution found.
    Otherwise, it returns am empty list and -1.

    :param init_board: The initial starting board.
    :type init_board: Board
    :param hfn: The heuristic function.
    :type hfn: Heuristic
    :return: (the path to goal state, solution cost)
    :rtype: List[State], int
    """
    frontier = [State(init_board, hfn, hfn(init_board), 0, None)]
    explored = set()
    while len(frontier):
        cur_state = frontier.pop()
        if cur_state.id not in explored:
            explored.add(cur_state.id)
            if is_goal(cur_state):
                return get_path(cur_state), cur_state.depth
            frontier.extend(get_successors(cur_state))
            frontier = sorted(frontier, key=lambda state: (state.f, state.id), reverse=True)
    return [], -1


def dfs(init_board):
    """
    Run the DFS algorithm given an initial board.

    If the function finds a goal state, it returns a list of states representing
    the path from the initial state to the goal state in order and the cost of
    the solution found.
    Otherwise, it returns am empty list and -1.

    :param init_board: The initial board.
    :type init_board: Board
    :return: (the path to goal state, solution cost)
    :rtype: List[State], int
    """
    frontier = [State(init_board, None, 0, 0)]
    explored = set()
    while len(frontier):
        cur_state = frontier.pop()
        if cur_state.id not in explored:
            explored.add(cur_state.id)
            if is_goal(cur_state):
                return get_path(cur_state), cur_state.depth
            successors = sorted(get_successors(cur_state), key=lambda state: state.id, reverse=True)
            frontier.extend(successors)
    return [], -1


def __get_new_state__(direction, state_list: List[State], state, car, offset: int):
    new_cars = copy.deepcopy(state.board.cars)
    for i in range(len(new_cars)):
        cur_car = new_cars[i]
        if cur_car.orientation == car.orientation and \
                cur_car.fix_coord == car.fix_coord and cur_car.var_coord == car.var_coord:
            if direction == 'left' or direction == 'upward':
                new_cars[i].var_coord = offset - 1
            elif direction == 'right' or direction == 'downward':
                new_cars[i].var_coord = offset - new_cars[i].length + 1
            new_board = Board(state.board.name, state.board.size, new_cars)
            new_state = State(new_board, state.hfn, state.depth + 1 + state.hfn(new_board) if state.hfn else 0,
                              state.depth + 1, state)
            state_list.append(new_state)
            break


def __board_to_table__(board):
    # Table tracks occupied space on board
    table = []
    for r in range(board.size):
        row = []
        for c in range(board.size):
            row.append(0)
        table.append(row)

    for car in board.cars:
        for a in range(car.length):
            if car.orientation == 'h':
                table[car.var_coord + a][car.fix_coord] = 1
            else:
                table[car.fix_coord][car.var_coord + a] = 1

    return table


def get_successors(state):
    """
    Return a list containing the successor states of the given state.
    The states in the list may be in any arbitrary order.

    :param state: The current state.
    :type state: State
    :return: The list of successor states.
    :rtype: List[State]
    """
    state_list = []
    table = __board_to_table__(state.board)

    # Iterate through cars. For each car, produce as many states as possible.
    for car in state.board.cars:
        temp_table = copy.deepcopy(table)

        if car.orientation == 'h':
            for a in range(car.length):  # Remove current car from the board
                temp_table[car.var_coord + a][car.fix_coord] = 0
            # Move car to the left
            for a in range(car.var_coord, 0, -1):
                if temp_table[a - 1][car.fix_coord] == 0:
                    __get_new_state__('left', state_list, state, car, a)
                else:
                    break

            # Move car to the right
            for a in range(car.var_coord + car.length, 6):
                if temp_table[a][car.fix_coord] == 0:
                    __get_new_state__('right', state_list, state, car, a)
                else:
                    break

        else:  # car is vertical
            for a in range(car.length):  # Remove current car from the board
                temp_table[car.fix_coord][car.var_coord + a] = 0

            # Move car upward
            for a in range(car.var_coord, 0, -1):
                if temp_table[car.fix_coord][a - 1] == 0:
                    __get_new_state__('upward', state_list, state, car, a)
                else:
                    break

            # Move car downward
            for a in range(car.var_coord + car.length, 6):
                if temp_table[car.fix_coord][a] == 0:
                    __get_new_state__('downward', state_list, state, car, a)
                else:
                    break

    # for state in stateList:
    #     state.board.display()

    return state_list


def is_goal(state):
    """
    Returns True if the state is the goal state and False otherwise.

    :param state: the current state.
    :type state: State
    :return: True or False
    :rtype: bool
    """
    goal_car = next(filter(lambda car: car.is_goal, state.board.cars), None)
    return True if goal_car.var_coord == 4 else False


def get_path(state):
    """
    Return a list of states containing the nodes on the path
    from the initial state to the given state in order.

    :param state: The current state.
    :type state: State
    :return: The path.
    :rtype: List[State]
    """
    cur_state = state
    rv_list = [cur_state]
    while cur_state.parent:
        rv_list.append(cur_state.parent)
        cur_state = cur_state.parent
    rv_list.reverse()
    return rv_list


def blocking_heuristic(board):
    """
    Returns the heuristic value for the given board
    based on the Blocking Heuristic function.

    Blocking heuristic returns zero at any goal board,
    and returns one plus the number of cars directly
    blocking the goal car in all other states.

    :param board: The current board.
    :type board: Board
    :return: The heuristic value.
    :rtype: int
    """

    count = 0
    goal_car = next(filter(lambda item: item.is_goal, board.cars), None)
    goal_x = goal_car.var_coord

    # goal state, return 0
    if goal_x == 4:
        return 0

    for car in board.cars:
        if car.orientation == 'h' and car.fix_coord == 2 and car.var_coord > goal_x:
            count = count + 1
        elif car.orientation == 'v' and car.fix_coord > goal_x + goal_car.length - 1 and \
                car.var_coord <= 2 and car.var_coord + car.length - 1 >= 2:
            count = count + 1

    return count


def advanced_heuristic(board):
    """
    An advanced heuristic of your own choosing and invention.

    :param board: The current board.
    :type board: Board
    :return: The heuristic value.
    :rtype: int
    """
    count = 0
    goal_car = next(filter(lambda item: item.is_goal, board.cars), None)
    goal_x = goal_car.var_coord
    table = __board_to_table__(board)
    deep_blocked = False

    # goal state, return 0
    if goal_x == 4:
        return 0

    for car in board.cars:
        # blocked by a horizontal car; there's no solution to this puzzle
        if car.orientation == 'h' and car.fix_coord == 2 and car.var_coord > goal_x:
            count = count + 1
        # blocked by a vertical car
        elif car.orientation == 'v' and car.fix_coord > goal_x + goal_car.length - 1 and \
                car.var_coord <= 2 and car.var_coord + car.length - 1 >= 2:
            count = count + 1

            if not deep_blocked:
                # Remove current car from the table.
                temp_table = copy.deepcopy(table)
                for a in range(car.length):
                    temp_table[car.fix_coord][car.var_coord + a] = 0

                # check if it could be moved downward:
                blocked_bot = False
                if car.length > 3:
                    blocked_bot = True
                else:
                    for a in range(3, 3 + car.length):
                        if temp_table[car.fix_coord][a]:
                            blocked_bot = True

                # check if it could be moved upward:
                blocked_top = False
                if car.length > 2:
                    blocked_top = True
                else:
                    for a in range(2 - car.length, 2):
                        if temp_table[car.fix_coord][a]:
                            blocked_top = True

                if blocked_top and blocked_bot:
                    count = count + 1
                    deep_blocked = True

    return count

#
# import time
# from os import system, name
#
# boards = from_file("jams_posted.txt")
# board = boards[3]
# board.display()
#
#
# listOfState = a_star(board, blocking_heuristic)
#
# listOfState2 = a_star(board, advanced_heuristic)
#
# print(len(listOfState2[0]))
# for state in listOfState[0]:
#     state.board.display()

#
# print(len(listOfState2))

# results = [[], [], [], [],[],[],[],[]]
# for i in range(40):
#     print("problem: ", i, "==========================================")
#     board = boards[i]
#
#     listOfState = a_star(board, blocking_heuristic)
#
#     listOfState2 = a_star(board, advanced_heuristic)
#
#     ls1 = []
#     ls2 = []
#     for state in listOfState[0]:
#         ls1.append(state.id)
#     for state in listOfState2[0]:
#         ls2.append(state.id)
#
#     results[0].append(ls1)
#     results[1].append(ls2)
#     results[2].append(listOfState[1])
#     results[3].append(listOfState2[1])
#     break
#
#
# print(results)
