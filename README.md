# Pacman Search
## Implementation of general graph search algorithms to solve the search problems modeled in Pacman

## Algorithms
- Iterative Deepening Search algorithm
- Weighted A* algorithm (W = 2)
- Agent eat the Capsule first, then eat the remaiding food dots

## Run
### Iterative Deepening Search algorithm
```
python pacman.py -l mediumMaze -p SearchAgent -a fn=ids
```

### Weighted A* algorithm (W = 2)
```
python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=wastar,heuristic=manhattanHeuristic
```

### Agent eat the Capsule first, then eat the remaiding food dots
```
python pacman.py -l capsuleSearch -p CapsuleSearchAgent -a fn=wastar,prob=CapsuleSearchProblem,heuristic=foodHeuristic
```
