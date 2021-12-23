package jps_plus

const (
	DirectionUp = 1 << iota
	DirectionRight
	DirectionDown
	DirectionLeft
)

/*
------------------------- pre find jp --------------------------
---------首先对地图每个节点进行跳点判断，找出所有的主要跳点 ----------
*/

type JpMatrix [][]uint8

func preCptJpMatrix(origin [][]uint8) JpMatrix {
	jpm := make([][]uint8, len(origin))
	for row := range origin {
		jpm[row] = make([]uint8, len(origin[0]))
		for col, p := range origin[row] {
			if p == 0 {
				if isMainJumpPoint(row, col, 1, 0, origin) {
					jpm[row][col] |= DirectionDown
				}
				if isMainJumpPoint(row, col, -1, 0, origin) {
					jpm[row][col] |= DirectionUp
				}
				if isMainJumpPoint(row, col, 0, 1, origin) {
					jpm[row][col] |= DirectionRight
				}
				if isMainJumpPoint(row, col, 0, -1, origin) {
					jpm[row][col] |= DirectionLeft
				}
			}
		}
	}
	return jpm
}

func isBlock(row, col int, origin [][]uint8) bool {
	if row < 0 || col < 0 || row >= len(origin) || col >= len(origin[0]) {
		return true
	}
	if origin[row][col] == 1 {
		return true
	}
	return false
}

func isMainJumpPoint(row, col, dRow, dCol int, origin [][]uint8) bool {
	if isBlock(row-dRow, col-dCol, origin) {
		return false
	}
	return (!isBlock(row+dCol, col+dRow, origin) && isBlock(row-dRow+dCol, col-dCol+dRow, origin)) ||
		(!isBlock(row-dCol, col-dRow, origin) && isBlock(row-dRow-dCol, col-dCol-dRow, origin))
}

func (jm JpMatrix) get(row, col int) uint8 {
	return jm[row][col]
}

/*
------------------------- pre record distance to jump point --------------------------
-------------------------对地图每个节点从8个方向计算到跳点的距离 --------------------------
如果在对应的方向上移动1步后碰到障碍（或边界）则记为0，如果移动n+1步后会碰到障碍（或边界）的数据记为负数距离-n
*/

type DirectionIdx int

const (
	IdxUpLeft DirectionIdx = iota
	IdxUp
	IdxUpRight
	IdxRight
	IdxDownRight
	IdxDown
	IdxDownLeft
	IdxLeft
	IdxAll
)

var (
	straightDir = [4]DirectionIdx{IdxUp, IdxRight, IdxDown, IdxLeft}
	leanDir     = [4]DirectionIdx{IdxUpLeft, IdxUpRight, IdxDownRight, IdxDownLeft}

	direction8 = [8][2]int{{-1, -1}, {-1, 0}, {-1, 1}, {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}}
)

type NodePlus struct {
	Node
	originVal    uint8
	jumpDistance [8]int64
}

func (np *NodePlus) JumpDistance() [8]int64 {
	return np.jumpDistance
}

func (np *NodePlus) SetJD(idx int, val int64) {
	np.jumpDistance[idx] = val
}

type DisMatrix [][]*NodePlus

func preCptDisMatrix(origin [][]uint8) DisMatrix {
	jpm := preCptJpMatrix(origin)
	// fmt.Println(jpm)
	dm := DisMatrix(make([][]*NodePlus, len(origin)))
	for row := range origin {
		dm[row] = make([]*NodePlus, len(origin[0]))
		for col, v := range origin[row] {
			dm[row][col] = &NodePlus{
				Node:         newNode(int64(row), int64(col)),
				originVal:    v,
				jumpDistance: dm.searchStraightDis(row, col, origin, jpm), // 对每个节点进行跳点的直线可达性判断，并记录好跳点直线直线距离

			}
		}
	}
	dm.searchLeanDis(origin, jpm)
	return dm
}

func (dm DisMatrix) searchStraightDis(row, col int, origin [][]uint8, jpm JpMatrix) [8]int64 {
	res := [8]int64{}
	for _, directionIdx := range straightDir {
		var cnt int64 = 0
		dRow, dCol := direction8[directionIdx][0], direction8[directionIdx][1]
		currRow, currCol := row, col
		for !isBlock(currRow, currCol, origin) {
			if row == currRow && col == currCol {
				currRow, currCol = currRow+dRow, currCol+dCol
				continue
			}
			if isSameDirection(jpm.get(currRow, currCol), directionIdx) {
				res[directionIdx] = cnt + 1
				break
			}
			cnt++
			currRow, currCol = currRow+dRow, currCol+dCol
			if isBlock(currRow, currCol, origin) {
				res[directionIdx] = -cnt
				break
			}
		}
	}
	return res
}

func (dm DisMatrix) searchLeanDis(origin [][]uint8, jpm JpMatrix) {
	for row := range dm {
		for col, v := range dm[row] {
			for _, directionIdx := range leanDir {
				// 类似地，我们对每个节点进行跳点斜向距离的记录
				var cnt int64 = 0
				dRow, dCol := direction8[directionIdx][0], direction8[directionIdx][1]
				currRow, currCol := row, col
				if isNearByBlock(currRow, currCol, directionIdx, origin) {
					continue
				}
				for !isBlock(currRow, currCol, origin) {
					if row == currRow && col == currCol {
						currRow, currCol = currRow+dRow, currCol+dCol
						continue
					}
					if hasStraightWayToJp(jpm.get(currRow, currCol), dm[currRow][currCol].jumpDistance, directionIdx) {
						v.SetJD(int(directionIdx), cnt+1)
						break
					}
					cnt++
					currRow, currCol = currRow+dRow, currCol+dCol
					if isBlock(currRow, currCol, origin) {
						v.SetJD(int(directionIdx), -cnt)
						break
					}
				}
			}
		}
	}
}

func isNearByBlock(row, col int, direction DirectionIdx, origin [][]uint8) bool {
	return isBlock(row+direction8[direction][0], col, origin) || isBlock(row, col+direction8[direction][1], origin)
}

func isSameDirection(jpDirection uint8, currDirection DirectionIdx) bool {
	if jpDirection == 0 {
		return false
	}
	switch currDirection {
	case IdxDown:
		if jpDirection&DirectionDown != 0 {
			return true
		}
	case IdxLeft:
		if jpDirection&DirectionLeft != 0 {
			return true
		}
	case IdxUp:
		if jpDirection&DirectionUp != 0 {
			return true
		}
	case IdxRight:
		if jpDirection&DirectionRight != 0 {
			return true
		}
	}
	return false
}

func hasStraightWayToJp(jpDirection uint8, directionArr [8]int64, currDirection DirectionIdx) bool {
	if jpDirection != 0 {
		switch currDirection {
		case IdxDownLeft:
			if jpDirection&DirectionDown != 0 || jpDirection&DirectionLeft != 0 {
				return true
			}
		case IdxUpLeft:
			if jpDirection&DirectionLeft != 0 || jpDirection&DirectionUp != 0 {
				return true
			}
		case IdxUpRight:
			if jpDirection&DirectionUp != 0 || jpDirection&DirectionRight != 0 {
				return true
			}
		case IdxDownRight:
			if jpDirection&DirectionRight != 0 || jpDirection&DirectionDown != 0 {
				return true
			}
		}
	}
	switch currDirection {
	case IdxDownLeft:
		if directionArr[IdxDown] > 0 || directionArr[IdxLeft] > 0 {
			return true
		}
	case IdxUpLeft:
		if directionArr[IdxUp] > 0 || directionArr[IdxLeft] > 0 {
			return true
		}
	case IdxUpRight:
		if directionArr[IdxUp] > 0 || directionArr[IdxRight] > 0 {
			return true
		}
	case IdxDownRight:
		if directionArr[IdxDown] > 0 || directionArr[IdxRight] > 0 {
			return true
		}
	}
	return false
}
