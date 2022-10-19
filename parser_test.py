from pddl_parser.PDDL import PDDL_Parser

parser = PDDL_Parser()
parser.parse_domain('domain.pddl')
parser.parse_problem('activity_1_problem_sugar.pddl')

print('PARSER STATE:')
for k, v in parser.__dict__.items():
    print(f'{k}\t\t{v}')

print('\n\n')
print('Actions:')
for a in parser.actions:
    print(f'{a}')

print('Predicates:', parser.predicates)