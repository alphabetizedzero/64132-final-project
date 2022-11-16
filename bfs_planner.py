from itertools import product
from logging.handlers import QueueListener
from queue import LifoQueue
from turtle import pensize
from pddl_parser.PDDL import PDDL_Parser


def insert_params(predicates, params):
    """
    :param predicates: a collection of predicates. The predicates should be tuples of strings
    :param params: a dict of {"?<param_name>": "<value>"}
    :returns: a set of predicates. Unknown parameters in state (strings that start with a '?') are replaced with their corresponding value in params
    """
    output = set()
    for p in predicates:
        output.add(tuple(params.get(s, s) for s in p))
    return output


def get_valid_actions(state, actions):
    """
    :param state: the state as a frozenset of predicates. The predicates should be tuples of strings
    :param actions: a list of pddl_parser Action objects
    :returns: A list of tuples: [(Action, {param: value}), ...] where the tuples represent valid Action/parameter combos that could be taken
    """

    output = []

    for act in actions:
                    
        # Use the parameter types to generate all combinations of possible input parameters
        param_types = [param[1] for param in act.parameters]
        param_options = [PARSER.types[pt] for pt in param_types]
        param_combos = product(*param_options)  # This is an iterator of tuples. Each tuple is a set of parameters (in order) that could be plugged into the action

        # Convert the set of parameter tuples to a map from parameter name to parameter value (e.g. {'?obj': sugar})
        param_combo_dicts = []
        for pc in param_combos:
            d = {param[0]: mapped_value for param, mapped_value in zip(act.parameters, pc)}
            param_combo_dicts.append(d)

        # Check if the action/parameter combination is valid from the current state
        for pcd in param_combo_dicts:

            positive_preconditions = insert_params(act.positive_preconditions, pcd)
            negative_preconditions = insert_params(act.negative_preconditions, pcd)

            if positive_preconditions.issubset(state) and negative_preconditions.isdisjoint(state):
                output.append((act, pcd))

    return output


def apply_action(state, action, params):
    """
    :param state: the state as a frozenset of predicates. The predicates should be tuples of strings
    :param action: a pddl_parser Action object
    :param params: a dict of {"?<param_name>": "<value>"}
    :returns: a new frozenset of predicates representing the state after the action is taken

    Assumes that it's valid to take the action from the given state
    """
    add_effects = insert_params(action.add_effects, params)
    del_effects = insert_params(action.del_effects, params)

    return state.difference(del_effects).union(add_effects)


def bfs(state_init, goal_positive_preconditions, goal_negative_preconditions):
    """
    :param state_init: the initial state as a frozenset of predicates. The predicates should be tuples of strings
    :param goal_positive_preconditions: a frozenset of predicates representing the positive preconditions of the goal. The predicates should be tuples of strings
    :param goal_negative_preconditions: a frozenset of predicates representing the negative preconditions of the goal. The predicates should be tuples of strings
    :returns: a sequence of actions that will lead from `state_init` to `state_goal`
    """

    visited = {state_init}  # The set of states that have already been added to the queue

    search_queue = [(state_init, [])]  # [(state, [(action, params), ...]), ...]
    while search_queue:
        
        # Pop the next state from the queue
        popped_state, popped_actions = search_queue[0]
        search_queue = search_queue[1:]

        # print(f'popped_state: {popped_state}')

        if goal_positive_preconditions.issubset(popped_state) and goal_negative_preconditions.isdisjoint(popped_state):
            return popped_actions

        potential_actions = get_valid_actions(popped_state, PARSER.actions)

        for action, params in potential_actions:
            new_state = apply_action(popped_state, action, params)

            if new_state not in visited:
                search_queue.append((new_state, [*popped_actions, (action, params)]))
                visited.add(new_state)


if __name__ == '__main__':


    # Parse input files
    PARSER = PDDL_Parser()
    PARSER.parse_domain('domain.pddl')
    PARSER.parse_problem('activity_1_problem_sugar.pddl')

    print('PARSER STATE:')
    for k, v in PARSER.__dict__.items():
        print(f'{k}\t\t{v}')

    print('\n\n\n')

    init_state = PARSER.state

    print(f'Initial State:\n{init_state}')
    print(f'Pos Precond: {PARSER.positive_goals}')
    print(f'Neg Precond: {PARSER.negative_goals}\n\n')

    result = bfs(init_state, PARSER.positive_goals, PARSER.negative_goals)
    for i, (action, params) in enumerate(result):
        print(f'Action {i}:\n{action}')
        print(f'Params:\n{params}')
        print('\n\n')
