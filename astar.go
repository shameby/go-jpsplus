package jps_plus

import (
	"container/heap"
	"fmt"
	"math"
)

var (
	directionMap = map[DirectionIdx][]DirectionIdx{
		IdxDown:      {IdxLeft, IdxDownLeft, IdxDown, IdxDownRight, IdxRight},
		IdxLeft:      {IdxLeft, IdxDownLeft, IdxDown, IdxUpLeft, IdxUp},
		IdxRight:     {IdxRight, IdxDownRight, IdxDown, IdxUpRight, IdxUp},
		IdxUp:        {IdxLeft, IdxUpLeft, IdxUp, IdxUpRight, IdxRight},
		IdxUpRight:   {IdxUpRight, IdxUp, IdxRight},
		IdxDownRight: {IdxRight, IdxDownRight, IdxDown},
		IdxDownLeft:  {IdxLeft, IdxDownLeft, IdxDown},
		IdxUpLeft:    {IdxLeft, IdxUpLeft, IdxUp},
		IdxAll:       {IdxUp, IdxDown, IdxLeft, IdxRight, IdxUpLeft, IdxUpRight, IdxDownLeft, IdxDownRight},
	}
)

type aStarNode struct {
	node      Node
	direction DirectionIdx
	gScore    float64
	fScore    float64
}

// aStarQueue is an A* priority queue.
type aStarQueue struct {
	indexOf map[Node]int
	nodes   []*aStarNode
}

func (q *aStarQueue) Less(i, j int) bool {
	return q.nodes[i].fScore < q.nodes[j].fScore
}

func (q *aStarQueue) Swap(i, j int) {
	q.indexOf[q.nodes[i].node] = j
	q.indexOf[q.nodes[j].node] = i
	q.nodes[i], q.nodes[j] = q.nodes[j], q.nodes[i]
}

func (q *aStarQueue) Len() int {
	return len(q.nodes)
}

func (q *aStarQueue) Push(x interface{}) {
	n := x.(*aStarNode)
	q.indexOf[n.node] = len(q.nodes)
	q.nodes = append(q.nodes, n)
}

func (q *aStarQueue) Pop() interface{} {
	n := q.nodes[len(q.nodes)-1]
	q.nodes = q.nodes[:len(q.nodes)-1]
	delete(q.indexOf, n.node)
	return n
}

func (q *aStarQueue) update(node Node, g, f float64, direction DirectionIdx) {
	i, ok := q.indexOf[node]
	if !ok {
		return
	}
	q.nodes[i].gScore = g
	q.nodes[i].fScore = f
	q.nodes[i].direction = direction
	heap.Fix(q, i)
}

func (q *aStarQueue) node(node Node) (*aStarNode, bool) {
	loc, ok := q.indexOf[node]
	if ok {
		return q.nodes[loc], true
	}
	return nil, false
}

type AStarMatrix DisMatrix

func NewMatrix(origin [][]uint8) AStarMatrix {
	return AStarMatrix(preCptDisMatrix(origin))
}

func (am AStarMatrix) getNode(row, col int64) Node {
	return am[row][col].Node
}

func (am AStarMatrix) getNodePlus(node Node) *NodePlus {
	return am[node.GetRow()][node.GetCol()]
}

func (am AStarMatrix) AStarJump(start, goal [2]int64) ([]Node, error) {
	cameFrom := make(map[Node]Node)
	visited := make(map[Node]bool)
	openList := &aStarQueue{indexOf: make(map[Node]int)}
	s, g := am.getNode(start[0], start[1]), am.getNode(goal[0], goal[1])
	heap.Push(openList, &aStarNode{node: s, direction: IdxAll, gScore: 0, fScore: heuristic(s, g)})
	// loop
	for openList.Len() != 0 {
		curr := heap.Pop(openList).(*aStarNode)
		currNode := curr.node
		if currNode == g {
			break
		}
		fmt.Println(curr.node.Arr(), curr.fScore)
		visited[currNode] = true
		jd := am.getNodePlus(currNode).jumpDistance
		for _, direction := range directionMap[curr.direction] {
			distance := jd[direction]
			if distance == 0 {
				continue
			}
			dRow, dCol := direction8[direction][0], direction8[direction][1]
			var to Node
			if distance < 0 {
				// 对于负数距离 -n（意味着距离边界或障碍 n 格），我们直接将n步远的节点进行一次跳点判断
				to = am.getNode(currNode.GetRow()-distance*int64(dRow), currNode.GetCol()-distance*int64(dCol))
				if isOnWay(currNode, to, g) {
					cameFrom[g] = currNode
					return genPath(cameFrom, g)
				}
				/*if !isStraightHasJp(am.getNodePlus(to), direction) {
					continue
				}*/
			} else {
				to = am.getNode(currNode.GetRow()+distance*int64(dRow), currNode.GetCol()+distance*int64(dCol))
				if isOnWay(currNode, to, g) {
					cameFrom[g] = currNode
					return genPath(cameFrom, g)
				}
			}
			if visited[to] {
				continue
			}
			gs := curr.gScore + moveWeight(currNode, to)
			if n, exist := openList.node(to); !exist {
				heap.Push(openList, &aStarNode{node: to, direction: direction, gScore: gs, fScore: gs + heuristic(to, g)})
				cameFrom[to] = currNode
			} else if gs < n.gScore {
				openList.update(to, gs, gs+heuristic(to, g), direction)
				cameFrom[to] = currNode
			}
		}
	}
	return genPath(cameFrom, g)
}

func genPath(cameFrom map[Node]Node, g Node) ([]Node, error) {
	pathNodes := make([]Node, 0)

	if _, exist := cameFrom[g]; !exist {
		return nil, fmt.Errorf("destination not reacheable from source")
	}
	cur := g
	for {
		pathNodes = append(pathNodes, cur)
		next, exist := cameFrom[cur]
		if !exist {
			break
		}
		cur = next
	}
	return getReverse(pathNodes), nil
}

func heuristic(a, b Node) float64 {
	xDist := float64(a.GetRow() - b.GetRow())
	yDist := float64(a.GetCol() - b.GetCol())
	return math.Sqrt(math.Pow(xDist, 2) + math.Pow(yDist, 2))
}

func moveWeight(a, b Node) float64 {
	xDist := float64(a.GetRow() - b.GetRow())
	yDist := float64(a.GetCol() - b.GetCol())
	return math.Sqrt(math.Pow(xDist, 2) + math.Pow(yDist, 2))
}

/*func isStraightHasJp(curr *NodePlus, direction DirectionIdx) bool {
	switch direction {
	case IdxDownRight:
		return curr.jumpDistance[IdxDown] > 0 || curr.jumpDistance[IdxRight] > 0
	case IdxDownLeft:
		return curr.jumpDistance[IdxDown] > 0 || curr.jumpDistance[IdxLeft] > 0
	case IdxUpRight:
		return curr.jumpDistance[IdxUp] > 0 || curr.jumpDistance[IdxRight] > 0
	case IdxUpLeft:
		return curr.jumpDistance[IdxUp] > 0 || curr.jumpDistance[IdxLeft] > 0
	}
	return false
}*/

func isOnWay(curr, to, goal Node) bool {
	if to == goal {
		return true
	}
	if to.GetCol()-goal.GetCol() == 0 {
		if goal.GetCol()-curr.GetCol() == 0 {
			return isBetween(curr.GetRow(), goal.GetRow(), to.GetRow())
		}
		return false
	}
	if goal.GetCol()-curr.GetCol() == 0 {
		return false
	}
	if float64(to.GetRow()-goal.GetRow())/float64(to.GetCol()-goal.GetCol()) == float64(goal.GetRow()-curr.GetRow())/float64(goal.GetCol()-curr.GetCol()) {
		return isBetween(curr.GetRow(), goal.GetRow(), to.GetRow()) && isBetween(curr.GetCol(), goal.GetCol(), to.GetCol())
	}
	return false
}

func isBetween(a, b, c int64) bool {
	return (a >= b && b >= c) || (a <= b && b <= c)
}

func getReverse(nodes []Node) []Node {
	length := len(nodes)
	reversedNodes := make([]Node, length)
	for i := range nodes {
		reversedNodes[i] = nodes[length-i-1]
	}
	return reversedNodes
}
