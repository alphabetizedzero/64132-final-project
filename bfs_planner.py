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


def bfs(state_init, state_goal):
    """
    :param state_init:
    :param state_goal:
    :returns: a sequence of actions that will lead from `state_init` to `state_goal`
    """

    visited = {state_init}  # The set of states that have already been added to the queue

    search_queue = [(state_init, [])]  # [(state, [(action, params), ...]), ...]
    while search_queue:
        
        # Pop the next state from the queue
        popped_state, popped_actions = search_queue[0]
        search_queue = search_queue[1:]

        # print(f'popped_state: {popped_state}')
        if state_goal.issubset(popped_state):
            return popped_actions

        potential_actions = get_valid_actions(popped_state, PARSER.actions)

        for action, params in potential_actions:
            new_state = apply_action(popped_state, action, params)

            if new_state not in visited:
                search_queue.append((new_state, [*popped_actions, (action, params)]))
                visited.add(new_state)


        # print(f'\nSearch Queue ({len(search_queue)}):\n')
        # for s, a in search_queue:
        #     print('Taking Action:')
        #     print(f'{a[0][0]}')
        #     print(f'{a[0][1]}')
        #     print(f'Resulting state: {s}')
        #     print('\n\n\n')

        
        # break

        


if __name__ == '__main__':


    # Parse input files
    PARSER = PDDL_Parser()
    PARSER.parse_domain('domain.pddl')
    PARSER.parse_problem('activity_1_problem_sugar.pddl')

    print('PARSER STATE:')
    for k, v in PARSER.__dict__.items():
        print(f'{k}\t\t{v}')

    # print('Types:')
    # print(f'{PARSER.types}')

    # print('\n\n')
    # print('Actions:')
    # for a in PARSER.actions:
    #     print(f'{a}')
    #     print(f'pos_pre: {a.positive_preconditions}\n\n')

    # print('Predicates:', PARSER.predicates)
    print('\n\n\n')

    init_state = PARSER.state


    # # Test get_valid_actions
    # va = get_valid_actions(init_state, PARSER.actions)
    # print(f'Init State: {init_state}')
    # print('Valid Actions:')
    # for i, a in enumerate(va):
    #     print(i, *a, '\n\n')


    print(f'Initial State:\n{init_state}\n\n')
    result = bfs(init_state, PARSER.positive_goals)
    for i, (action, params) in enumerate(result):
        print(f'Action {i}:\n{action}')
        print(f'Params:\n{params}')
        print('\n\n')
